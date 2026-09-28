#include "z_pos_estimator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "Robot.hpp"
#include "reliable_contact.hpp"

namespace state_estimator_hmb
{
namespace
{

constexpr Eigen::Index kNumJoints = 12;

// Период контура оценщика. Должен совпадать с
// `config/timesteps.yaml: state_estimator_dt` — счётчики ниже заданы в тиках.
constexpr double kEstimatorLoopDt = 0.002;

// Опора считается установившейся только после того, как флаг контакта держится
// непрерывно этот интервал. За время переходного процесса приземления стопа
// успевает дойти до грунта и нагрузиться, поэтому якорь ставится по уже
// нагруженной (а не по свободной) геометрии ноги.
//
// Значение подобрано по логу `log_260908_163158` (трот, период 0.32 с): именно
// при 40 мс обнуляется остаточный храповик от податливости ноги — свежая опора
// нагружена так же, как опоры, по которым строится оценка. Податливость
// измерена как ~0.07 мм/Н, поэтому при заметно другой походке или загрузке
// выдержку нужно перепроверить; структурно её снимает только компенсация
// прогиба по GRF или фильтр Калмана.
constexpr double kAnchorSettleSeconds = 0.04;
constexpr unsigned kAnchorSettleTicks =
    static_cast<unsigned>(kAnchorSettleSeconds / kEstimatorLoopDt + 0.5);

// Кратковременная потеря флага контакта считается дребезгом: якорь сохраняется,
// чтобы нога не переякоривалась по несколько раз за одну опорную фазу.
constexpr double kContactLossDebounceSeconds = 0.03;
constexpr unsigned kContactLossDebounceTicks =
    static_cast<unsigned>(kContactLossDebounceSeconds / kEstimatorLoopDt + 0.5);

// Если за время пропадания флага стопа сместилась больше, чем на эту величину,
// значит нога действительно оторвалась и старый якорь недействителен.
constexpr double kMaxAnchorFootDrift = 0.005;

// Порог доверия, начиная с которого нога может создать новый якорь. Доверие
// нарастает на первых 20 % опорной фазы, поэтому порог задаёт выдержку после
// планового начала опоры и не зависит от темпа походки.
constexpr float kAnchorTrustThreshold = 0.5F;

// Минимальное доверие, при котором нога ещё участвует в оценке высоты.
constexpr float kSupportTrustThreshold = 0.05F;

constexpr unsigned kTickCounterMax = 1000000U;

// Index by servo leg order (R1, L1, R2, L2); values refer to Pinocchio's
// toe order (L1, L2, R1, R2).
constexpr std::array<std::size_t, ZPosEstimator::kLegCount> kServoToPinocchio{
    2, 0, 3, 1};

}  // namespace

class ZPosEstimator::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using TrustCoefficients = std::array<float, ZPosEstimator::kLegCount>;
    using RelativeFootZ = std::array<double, ZPosEstimator::kLegCount>;

    explicit Impl(double initial_contact_z)
        : initial_contact_z_(initial_contact_z)
    {
        if (!std::isfinite(initial_contact_z_))
        {
            throw std::invalid_argument("initial_contact_z must be finite");
        }

        robot_.BuildPinocchioModel();
        q_ = Eigen::VectorXd::Zero(robot_.nq);
        v_ = Eigen::VectorXd::Zero(robot_.nv);
        if (q_.size() < 19 || v_.size() < 18)
        {
            throw std::runtime_error(
                "Pinocchio model has an incompatible configuration layout");
        }
        q_(6) = 1.0;
    }

    [[nodiscard]] std::optional<ZPosEstimate> Update(
        const Eigen::Ref<const Eigen::VectorXd>& joint_positions,
        const Eigen::Matrix3d& world_R_body,
        const Contacts& contacts,
        const GaitPhases& gait_phases,
        const ContactStates& contact_states)
    {
        if (joint_positions.size() != kNumJoints)
        {
            throw std::invalid_argument(
                "ZPosEstimator expects exactly 12 joint positions");
        }

        if (!joint_positions.allFinite() || !world_R_body.allFinite())
        {
            return EstimateWithoutUpdate();
        }

        q_.setZero();
        v_.setZero();
        q_(6) = 1.0;
        q_.segment<3>(7) = joint_positions.segment<3>(3);   // L1
        q_.segment<3>(10) = joint_positions.segment<3>(9);  // L2
        q_.segment<3>(13) = joint_positions.segment<3>(0);  // R1
        q_.segment<3>(16) = joint_positions.segment<3>(6);  // R2

        robot_.ComputeForwardKinematics(q_, v_);
        const std::vector<Eigen::Vector3d> pinocchio_foot_positions =
            robot_.GetToePositionsInBaseFrame();
        if (pinocchio_foot_positions.size() != kLegCount)
        {
            throw std::runtime_error(
                "Pinocchio model must provide exactly four toe frames");
        }

        RelativeFootZ relative_foot_z{};
        for (std::size_t leg = 0; leg < kLegCount; ++leg)
        {
            relative_foot_z[leg] = (world_R_body * pinocchio_foot_positions[kServoToPinocchio[leg]]).z();
            if (!std::isfinite(relative_foot_z[leg]))
            {
                return EstimateWithoutUpdate();
            }
        }

        TrustCoefficients trust{};
        for (std::size_t leg = 0; leg < kLegCount; ++leg)
        {
            trust[leg] = reliable_contact_.get_trust_coefficient(
                gait_phases[leg], contact_states[leg]);
        }

        UpdateContactBookkeeping(relative_foot_z, contacts, trust);

        if (!initialized_)
        {
            return Initialize(relative_foot_z, contacts, trust);
        }

        // Оценка строится только по якорям, существовавшим на начало такта, и
        // взвешивается доверием к контакту: нога на краях опорной фазы влияет
        // слабее, чем нога в середине опоры.
        double weighted_sum = 0.0;
        double weight_sum = 0.0;
        std::size_t contacts_used = 0;
        for (std::size_t leg = 0; leg < kLegCount; ++leg)
        {
            if (!contacts[leg] || !anchor_valid_[leg] ||
                trust[leg] < kSupportTrustThreshold)
            {
                continue;
            }

            const double weight = static_cast<double>(trust[leg]);
            weighted_sum += weight * (anchor_z_[leg] - relative_foot_z[leg]);
            weight_sum += weight;
            ++contacts_used;
        }

        const bool has_reliable_support =
            contacts_used >= kMinimumSupportLegs && weight_sum > 0.0;
        if (has_reliable_support)
        {
            position_z_ = weighted_sum / weight_sum;
        }

        TransferSupport(relative_foot_z, contacts, trust, has_reliable_support);

        return ZPosEstimate{
            position_z_,
            contacts_used,
            has_reliable_support};
    }

    void Reset() noexcept
    {
        anchor_z_.fill(0.0);
        anchor_valid_.fill(false);
        release_foot_z_.fill(0.0);
        contact_hold_ticks_.fill(0U);
        release_ticks_.fill(0U);
        position_z_ = 0.0;
        initialized_ = false;
    }

    [[nodiscard]] bool initialized() const noexcept
    {
        return initialized_;
    }

private:
    // Ведёт счётчики удержания контакта и гасит дребезг флага контакта. Якорь
    // переживает короткий пропуск флага, но только если стопа при этом осталась
    // на месте; настоящий отрыв ноги якорь сбрасывает.
    void UpdateContactBookkeeping(
        const RelativeFootZ& relative_foot_z,
        const Contacts& contacts,
        const TrustCoefficients& trust) noexcept
    {
        for (std::size_t leg = 0; leg < kLegCount; ++leg)
        {
            if (contacts[leg])
            {
                const bool foot_moved_while_released =
                    release_ticks_[leg] > 0U &&
                    std::abs(relative_foot_z[leg] - release_foot_z_[leg]) >
                        kMaxAnchorFootDrift;
                if (foot_moved_while_released)
                {
                    anchor_valid_[leg] = false;
                    contact_hold_ticks_[leg] = 0U;
                }

                release_ticks_[leg] = 0U;
                if (contact_hold_ticks_[leg] < kTickCounterMax)
                {
                    ++contact_hold_ticks_[leg];
                }
                continue;
            }

            if (!anchor_valid_[leg])
            {
                contact_hold_ticks_[leg] = 0U;
                release_ticks_[leg] = 0U;
                continue;
            }

            if (release_ticks_[leg] == 0U)
            {
                release_foot_z_[leg] = relative_foot_z[leg];
            }
            ++release_ticks_[leg];

            const bool debounce_expired =
                release_ticks_[leg] > kContactLossDebounceTicks;
            const bool foot_left_anchor =
                std::abs(relative_foot_z[leg] - release_foot_z_[leg]) >
                kMaxAnchorFootDrift;
            const bool swing_scheduled = trust[leg] <= 0.0F;
            if (debounce_expired || foot_left_anchor || swing_scheduled)
            {
                anchor_valid_[leg] = false;
                contact_hold_ticks_[leg] = 0U;
                release_ticks_[leg] = 0U;
            }
        }
    }

    // Планировщик уверенно считает ногу опорной.
    [[nodiscard]] bool IsTrustedSupport(
        std::size_t leg,
        const Contacts& contacts,
        const TrustCoefficients& trust) const noexcept
    {
        return contacts[leg] && trust[leg] >= kAnchorTrustThreshold;
    }

    // Нога готова принять перенос опоры: вдобавок к доверию флаг контакта
    // держится достаточно долго, чтобы переходный процесс приземления закончился.
    // На инициализации выдержка не нужна — там высота опоры берётся из внешнего
    // `initial_contact_z_`, а не переносится с текущей оценки.
    [[nodiscard]] bool CanAnchor(
        std::size_t leg,
        const Contacts& contacts,
        const TrustCoefficients& trust) const noexcept
    {
        return IsTrustedSupport(leg, contacts, trust) &&
               contact_hold_ticks_[leg] >= kAnchorSettleTicks;
    }

    void TransferSupport(
        const RelativeFootZ& relative_foot_z,
        const Contacts& contacts,
        const TrustCoefficients& trust,
        bool has_reliable_support) noexcept
    {
        // Новый якорь создаётся только от свежей оценки: если на этом такте
        // высота не обновлялась, ошибка устаревшего `position_z_` была бы
        // навсегда записана в высоту опоры.
        if (has_reliable_support)
        {
            for (std::size_t leg = 0; leg < kLegCount; ++leg)
            {
                if (anchor_valid_[leg] || !CanAnchor(leg, contacts, trust))
                {
                    continue;
                }

                anchor_z_[leg] = position_z_ + relative_foot_z[leg];
                anchor_valid_[leg] = true;
            }
        }

        const bool has_any_anchor = std::any_of(
            anchor_valid_.begin(),
            anchor_valid_.end(),
            [](bool valid) { return valid; });
        if (has_any_anchor)
        {
            return;
        }

        // Аварийное восстановление: без якорей оценщик замер бы навсегда, так
        // что опора переносится от устаревшей высоты. Такт помечается как
        // ненадёжный вызывающей стороной (`updated_from_contacts == false`).
        std::size_t recoverable = 0;
        for (std::size_t leg = 0; leg < kLegCount; ++leg)
        {
            if (contacts[leg] && contact_hold_ticks_[leg] >= kAnchorSettleTicks)
            {
                ++recoverable;
            }
        }
        if (recoverable < kMinimumSupportLegs)
        {
            return;
        }

        for (std::size_t leg = 0; leg < kLegCount; ++leg)
        {
            if (contacts[leg] && contact_hold_ticks_[leg] >= kAnchorSettleTicks)
            {
                anchor_z_[leg] = position_z_ + relative_foot_z[leg];
                anchor_valid_[leg] = true;
            }
        }
    }

    [[nodiscard]] std::optional<ZPosEstimate> Initialize(
        const RelativeFootZ& relative_foot_z,
        const Contacts& contacts,
        const TrustCoefficients& trust)
    {
        double weighted_sum = 0.0;
        double weight_sum = 0.0;
        std::size_t contacts_used = 0;
        for (std::size_t leg = 0; leg < kLegCount; ++leg)
        {
            if (!IsTrustedSupport(leg, contacts, trust))
            {
                continue;
            }

            const double weight = static_cast<double>(trust[leg]);
            weighted_sum += weight * (initial_contact_z_ - relative_foot_z[leg]);
            weight_sum += weight;
            ++contacts_used;
        }

        if (contacts_used < kMinimumSupportLegs || weight_sum <= 0.0)
        {
            return std::nullopt;
        }

        position_z_ = weighted_sum / weight_sum;
        for (std::size_t leg = 0; leg < kLegCount; ++leg)
        {
            if (IsTrustedSupport(leg, contacts, trust))
            {
                anchor_z_[leg] = initial_contact_z_;
                anchor_valid_[leg] = true;
            }
        }
        initialized_ = true;
        return ZPosEstimate{position_z_, contacts_used, true};
    }

    [[nodiscard]] std::optional<ZPosEstimate> EstimateWithoutUpdate() const
    {
        if (!initialized_)
        {
            return std::nullopt;
        }
        return ZPosEstimate{position_z_, 0, false};
    }

    Robot robot_;
    Eigen::VectorXd q_;
    Eigen::VectorXd v_;
    ReliableContact reliable_contact_;
    std::array<double, kLegCount> anchor_z_{};
    std::array<bool, kLegCount> anchor_valid_{};
    std::array<double, kLegCount> release_foot_z_{};
    std::array<unsigned, kLegCount> contact_hold_ticks_{};
    std::array<unsigned, kLegCount> release_ticks_{};
    double initial_contact_z_ = 0.0;
    double position_z_ = 0.0;
    bool initialized_ = false;
};

ZPosEstimator::ZPosEstimator(double initial_contact_z)
    : impl_(std::make_unique<Impl>(initial_contact_z))
{
}

ZPosEstimator::~ZPosEstimator() = default;

std::optional<ZPosEstimate> ZPosEstimator::Update(
    const Eigen::Ref<const Eigen::VectorXd>& joint_positions,
    const Eigen::Matrix3d& world_R_body,
    const Contacts& contacts,
    const GaitPhases& gait_phases,
    const ContactStates& contact_states)
{
    return impl_->Update(
        joint_positions,
        world_R_body,
        contacts,
        gait_phases,
        contact_states);
}

void ZPosEstimator::Reset() noexcept
{
    impl_->Reset();
}

bool ZPosEstimator::initialized() const noexcept
{
    return impl_->initialized();
}

}  // namespace state_estimator_hmb
