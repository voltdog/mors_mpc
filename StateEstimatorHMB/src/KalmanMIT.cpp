#include "KalmanMIT.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>

#include "Robot.hpp"
#include "reliable_contact.hpp"
#include "structs.hpp"

namespace state_estimator_hmb
{
namespace
{

// Индексируется сервоприводным порядком ног (R1, L1, R2, L2), значения — индексы
// в векторах порядка pinocchio (L1, L2, R1, R2). Совпадает с одноимённой
// таблицей в z_pos_estimator.cpp.
constexpr std::array<std::size_t, 4> kServoToPinocchio{
    static_cast<std::size_t>(PIN_R1),
    static_cast<std::size_t>(PIN_L1),
    static_cast<std::size_t>(PIN_R2),
    static_cast<std::size_t>(PIN_L2)};

// Смещения блоков суставов ноги в векторах конфигурации и скорости pinocchio,
// в порядке pinocchio (L1, L2, R1, R2).
constexpr std::array<Eigen::Index, 4> kPinocchioJointQIndex{7, 10, 13, 16};
constexpr std::array<Eigen::Index, 4> kPinocchioJointVIndex{6, 9, 12, 15};

// Допуск на ортогональность матрицы поворота.
constexpr double kRotationTolerance = 1.0e-3;

}  // namespace

class KalmanMIT::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using MeasurementVector = Eigen::Matrix<double, kMeasurementSize, 1>;

    explicit Impl(const KalmanMITConfig& config)
        : config_(config)
    {
        ValidateConfig();

        robot_.BuildPinocchioModel();
        q_config_ = Eigen::VectorXd::Zero(robot_.nq);
        v_config_ = Eigen::VectorXd::Zero(robot_.nv);
        if (q_config_.size() < 19 || v_config_.size() < 18)
        {
            throw std::runtime_error(
                "KalmanMIT: pinocchio model has an incompatible layout");
        }
        q_config_(6) = 1.0;

        BuildConstantMatrices();
        Reset();
    }

    void ValidateConfig() const
    {
        const bool finite =
            std::isfinite(config_.dt) &&
            std::isfinite(config_.imu_process_noise_position) &&
            std::isfinite(config_.imu_process_noise_velocity) &&
            std::isfinite(config_.foot_process_noise_position) &&
            std::isfinite(config_.accel_bias_process_noise) &&
            std::isfinite(config_.initial_accel_bias_covariance) &&
            std::isfinite(config_.max_accel_bias) &&
            std::isfinite(config_.foot_sensor_noise_position) &&
            std::isfinite(config_.foot_sensor_noise_velocity) &&
            std::isfinite(config_.foot_height_sensor_noise) &&
            std::isfinite(config_.high_suspect_number) &&
            std::isfinite(config_.ground_height) &&
            std::isfinite(config_.terrain_process_noise) &&
            std::isfinite(config_.terrain_swing_process_noise) &&
            std::isfinite(config_.terrain_relaxation_time_sec) &&
            std::isfinite(config_.initial_terrain_covariance) &&
            std::isfinite(config_.terrain_map_noise) &&
            std::isfinite(config_.initial_covariance) &&
            std::isfinite(config_.max_velocity) &&
            std::isfinite(config_.gravity) &&
            std::isfinite(config_.zupt_trust_threshold) &&
            std::isfinite(config_.zupt_accel_tolerance) &&
            std::isfinite(config_.zupt_omega_tolerance) &&
            std::isfinite(config_.zupt_hold_sec) &&
            std::isfinite(config_.zupt_measurement_noise) &&
            std::isfinite(config_.zupt_position_measurement_noise);
        if (!finite)
        {
            throw std::invalid_argument("KalmanMIT: config contains non-finite values");
        }
        if (config_.dt <= 0.0)
        {
            throw std::invalid_argument("KalmanMIT: dt must be positive");
        }
        if (config_.foot_sensor_noise_position <= 0.0 ||
            config_.foot_sensor_noise_velocity <= 0.0 ||
            config_.foot_height_sensor_noise <= 0.0)
        {
            throw std::invalid_argument(
                "KalmanMIT: measurement noise variances must be positive");
        }
        if (config_.initial_covariance <= 0.0)
        {
            throw std::invalid_argument(
                "KalmanMIT: initial_covariance must be positive");
        }
        if (config_.high_suspect_number < 0.0)
        {
            throw std::invalid_argument(
                "KalmanMIT: high_suspect_number must not be negative");
        }
        if (config_.accel_bias_process_noise < 0.0 ||
            config_.initial_accel_bias_covariance < 0.0)
        {
            throw std::invalid_argument(
                "KalmanMIT: accel bias noise parameters must not be negative");
        }
        if (config_.max_accel_bias <= 0.0)
        {
            throw std::invalid_argument(
                "KalmanMIT: max_accel_bias must be positive");
        }
        if (config_.terrain_process_noise < 0.0 ||
            config_.terrain_swing_process_noise < 0.0 ||
            config_.terrain_relaxation_time_sec < 0.0 ||
            config_.initial_terrain_covariance < 0.0)
        {
            throw std::invalid_argument(
                "KalmanMIT: terrain noise parameters must not be negative");
        }
        if (config_.terrain_map_noise <= 0.0)
        {
            throw std::invalid_argument(
                "KalmanMIT: terrain_map_noise must be positive");
        }
        if (config_.zupt_accel_tolerance < 0.0 ||
            config_.zupt_omega_tolerance < 0.0 ||
            config_.zupt_hold_sec < 0.0)
        {
            throw std::invalid_argument(
                "KalmanMIT: ZUPT gate parameters must not be negative");
        }
        if (config_.zupt_measurement_noise <= 0.0 ||
            config_.zupt_position_measurement_noise <= 0.0)
        {
            throw std::invalid_argument(
                "KalmanMIT: ZUPT noise variances must be positive");
        }
        if (config_.min_init_contacts > 4)
        {
            throw std::invalid_argument(
                "KalmanMIT: min_init_contacts must not exceed 4");
        }
    }

    void BuildConstantMatrices()
    {
        const double dt = config_.dt;
        const Eigen::Matrix3d identity3 = Eigen::Matrix3d::Identity();

        // Именно setZero, а не setIdentity: блоки ниже больше не покрывают всю
        // матрицу, и остаток единичной диагонали ушёл бы в шум процесса
        // смещения как 1.0 (м/с^2)^2 за такт.
        q_base_.setZero();
        q_base_.block<3, 3>(kPositionIdx, kPositionIdx) =
            (dt / 20.0) * config_.imu_process_noise_position * identity3;
        q_base_.block<3, 3>(kVelocityIdx, kVelocityIdx) =
            (dt * 9.8 / 20.0) * config_.imu_process_noise_velocity * identity3;
        q_base_.block<3, 3>(kAccelBiasIdx, kAccelBiasIdx) =
            (config_.estimate_accel_bias
                 ? dt * config_.accel_bias_process_noise
                 : 0.0) * identity3;
        q_base_.block<12, 12>(kFootIdx, kFootIdx) =
            dt * config_.foot_process_noise_position *
            Eigen::Matrix<double, 12, 12>::Identity();
        // Значение для доверенной ноги; недоверенная получает свой Q в Update,
        // потому что он зависит от доверия и потому меняется каждый такт.
        q_base_.block<4, 4>(kTerrainIdx, kTerrainIdx) =
            (config_.estimate_terrain_height
                 ? dt * config_.terrain_process_noise
                 : 0.0) * Eigen::Matrix4d::Identity();

        r_base_diag_.segment<12>(0).setConstant(config_.foot_sensor_noise_position);
        r_base_diag_.segment<12>(12).setConstant(config_.foot_sensor_noise_velocity);
        r_base_diag_.segment<4>(24).setConstant(config_.foot_height_sensor_noise);

        terrain_decay_ = (config_.estimate_terrain_height &&
                          config_.terrain_relaxation_time_sec > 0.0)
            ? std::exp(-dt / config_.terrain_relaxation_time_sec)
            : 1.0;

        const double timeout_ticks = config_.frozen_phase_timeout_sec / dt;
        frozen_phase_ticks_ = timeout_ticks <= 0.0
            ? 0U
            : static_cast<unsigned>(timeout_ticks + 0.5);

        const double hold_ticks = config_.zupt_hold_sec / dt;
        zupt_hold_ticks_ = hold_ticks <= 0.0
            ? 0U
            : static_cast<unsigned>(hold_ticks + 0.5);
    }

    void Reset() noexcept
    {
        xhat_.setZero();
        p_.setZero();
        prev_phi_.fill(0.0);
        phase_stall_ticks_.fill(0U);
        has_prev_phi_ = false;
        zupt_static_ticks_ = 0U;
        zupt_engaged_ = false;
        zupt_anchor_.setZero();
        initialized_ = false;
        output_ = KalmanMITOutput{};
    }

    [[nodiscard]] const KalmanMITOutput& Update(const KalmanMITInput& input)
    {
        if (!InputIsFinite(input))
        {
            return RejectTick();
        }

        // Доверие считается до кинематики: счётчики застоя фазы обязаны идти
        // каждый такт, иначе таймаут будет зависеть от качества входа.
        std::array<double, 4> trust{};
        std::array<bool, 4> frozen{};
        ComputeTrust(input, trust, frozen);
        output_.trust = trust;
        output_.frozen_phase = frozen;

        // Ворота ZUPT считаются здесь же и по той же причине: счётчик выдержки
        // обязан идти каждый такт, иначе выдержка зависела бы от того, сколько
        // тактов отбраковала кинематика.
        const bool zupt = UpdateZuptGate(input, trust);

        const LegKinematicsInternal kinematics =
            ComputeLegKinematics(input.joint_positions, input.joint_velocities);
        if (!kinematics.valid)
        {
            return RejectTick();
        }

        std::array<Eigen::Vector3d, 4> p_f{};
        std::array<Eigen::Vector3d, 4> dp_f{};
        for (int leg = 0; leg < kLegCount; ++leg)
        {
            p_f[leg] = input.world_R_body * kinematics.p_rel[leg];
            dp_f[leg] = input.world_R_body *
                (input.omega_body.cross(kinematics.p_rel[leg]) +
                 kinematics.dp_rel[leg]);
            if (!p_f[leg].allFinite() || !dp_f[leg].allFinite())
            {
                return RejectTick();
            }
        }

        if (!initialized_)
        {
            return Initialize(input, p_f);
        }

        // Снимок ДО предсказания: MIT строит псевдоизмерения от оценки на начало
        // такта. Если взять их после предсказания, самоссылочная часть
        // измерения скорости уходит на шаг вперёд и вносит систематическую
        // ошибку у ног с малым доверием.
        const Eigen::Vector3d p0 = xhat_.head<3>();
        const Eigen::Vector3d v0 = xhat_.segment<3>(kVelocityIdx);

        // Строка высоты стопы получает своё доверие, отдельное от общего, и
        // только при оценке рельефа.
        //
        // Оконное доверие MIT открывается от начала фазы STANCE, то есть ещё на
        // снижении стопы, и закрывается до отрыва. Плоскости это не мешало:
        // снижающуюся стопу правильно тянуть к грунту, он никуда не денется, а
        // просевший на передаче диагонали якорь на следующем шаге возвращала та
        // же плоскость. Состояние g_i так не умеет: на провале доверия корпус
        // успевает всплыть, g_i садится на всплывшую высоту и замерзает на ней,
        // а следующий шаг повторяет это заново — ратчет вверх +2.8 мм/с на
        // ровном участке лога log_260910_135248.
        //
        // Для ВЫСОТЫ фазовое окно и не нужно: стоящая на грунте стопа даёт
        // верную высоту независимо от того, что думает планировщик, — в отличие
        // от скорости, где проскальзывание и перекат как раз и есть то, от чего
        // окно защищает. Поэтому здесь работает бинарный признак контакта, и
        // ровный участок того же лога даёт +0.19 мм/с.
        std::array<double, 4> height_trust{};
        // Раздувать Q у g_i можно только на НАСТОЯЩЕМ переносе. Разгруженная
        // нога — это ещё не перенос: когда робот ложится, контакты пропадают
        // все сразу, а грунт под ногами никуда не девается. Раздутие по одному
        // лишь отсутствию контакта стирало высоты опор за время лежания и
        // лишало фильтр вертикального якоря — на логах log_260909_135813 и
        // log_260910_134406 это давало разброс 42 и 145 мм против 1 и 23 мм у
        // плоскости. Без планировщика перенос неотличим от разгрузки, поэтому
        // там опоры не забываются вовсе.
        std::array<bool, 4> swinging{};
        for (int leg = 0; leg < kLegCount; ++leg)
        {
            if (!config_.estimate_terrain_height)
            {
                height_trust[leg] = trust[leg];
                continue;
            }
            height_trust[leg] = input.contacts[leg] ? 1.0 : 0.0;
            swinging[leg] = !input.contacts[leg] && input.has_gait_phase &&
                input.contact_states[leg] == SWING;
        }

        q_ = q_base_;
        r_diag_ = r_base_diag_;
        for (int leg = 0; leg < kLegCount; ++leg)
        {
            const double suspect =
                1.0 + (1.0 - trust[leg]) * config_.high_suspect_number;
            q_.block<3, 3>(kFootIdx + 3 * leg, kFootIdx + 3 * leg) *= suspect;
            r_diag_.segment<3>(12 + 3 * leg) *= suspect;
            r_diag_(24 + leg) *=
                1.0 + (1.0 - height_trust[leg]) * config_.high_suspect_number;
            // На переносе высота опоры неизвестна: нога встанет неизвестно
            // куда. В остальное время g_i заморожена и служит якорем.
            if (config_.estimate_terrain_height && swinging[leg])
            {
                q_(kTerrainIdx + leg, kTerrainIdx + leg) =
                    config_.dt * config_.terrain_swing_process_noise;
            }
            if (config_.scale_position_measurement_noise)
            {
                r_diag_.segment<3>(3 * leg) *= suspect;
            }
        }

        for (int leg = 0; leg < kLegCount; ++leg)
        {
            y_.segment<3>(3 * leg) = -p_f[leg];
            y_.segment<3>(12 + 3 * leg) =
                (1.0 - trust[leg]) * v0 + trust[leg] * (-dp_f[leg]);
            if (config_.use_foot_height_measurement)
            {
                // Строка разностная: p_foot_i.z - g_i. Доверенная нога стоит на
                // своей опоре (0), недоверенной остаётся прежнее у MIT
                // самоссылочное слагаемое — кинематическая высота стопы,
                // пересчитанная в ту же разность.
                const double ground = xhat_(kTerrainIdx + leg);
                y_(24 + leg) = (1.0 - height_trust[leg]) *
                    (p0.z() + p_f[leg].z() - ground);
            }
            else
            {
                y_(24 + leg) = 0.0;
            }
        }

        xhat_prev_ = xhat_;
        p_prev_ = p_;

        // Без вычета смещения: его текущая оценка вычитается уже в системе
        // корпуса внутри Predict, потому что там же она входит в матрицу
        // перехода.
        const Eigen::Vector3d accel_world_raw =
            input.world_R_body * input.accel_body -
            Eigen::Vector3d(0.0, 0.0, config_.gravity);

        Predict(accel_world_raw, input.world_R_body);

        // Карта высот, если она что-то знает про эту ногу, поправляет опору до
        // того, как по ней сработает строка 24+i.
        if (!ApplyTerrainMap(input, trust))
        {
            xhat_ = xhat_prev_;
            p_ = p_prev_;
            return RejectTick();
        }

        // R диагональна, поэтому пакетное обновление эквивалентно
        // последовательному скалярному по строкам. Так фильтр обходится
        // произведениями матрицы на вектор и симметричными обновлениями ранга 1,
        // без единого вызова блочного GEMM Eigen. Это не только дешевле
        // (~20 вместо ~75 кфлоп на такт), но и обязательно здесь: сборка идёт с
        // -mavx2 при EIGEN_MAX_ALIGN_BYTES=16 (требование ABI pinocchio), и
        // блочные ядра Eigen в такой комбинации делают 32-байтовые выровненные
        // записи в буфер, выровненный лишь на 16 байт.
        for (int row = 0; row < kMeasurementSize; ++row)
        {
            if (!config_.use_foot_height_measurement && row >= 24)
            {
                break;
            }
            if (!ScalarUpdate(row, y_(row), r_diag_(row)))
            {
                xhat_ = xhat_prev_;
                p_ = p_prev_;
                return RejectTick();
            }
        }

        // Псевдоизмерения неподвижности идут после штатных строк: сначала
        // фильтр вбирает всё, что знает кинематика, и только потом узнаёт, что
        // корпус вообще не движется.
        output_.zupt_active = false;
        if (zupt)
        {
            if (!ApplyZupt(p0))
            {
                xhat_ = xhat_prev_;
                p_ = p_prev_;
                return RejectTick();
            }
            output_.zupt_active = true;
        }

        {
            // Обновления ранга 1 симметричны по построению; симметризация тут
            // только гасит накопление ошибок округления. Явный временный
            // обязателен: Eigen присваивает поэлементно, и
            // p_ = 0.5 * (p_ + p_.transpose()) читает уже перезаписанные ячейки.
            const StateMatrix p_transposed = p_.transpose();
            p_ = 0.5 * (p_ + p_transposed);
        }

        // Декорреляция x/y: абсолютное положение в плоскости ничем не
        // наблюдается, поэтому её ковариация растёт неограниченно и утягивает
        // обусловленность всей матрицы.
        if (p_.block<2, 2>(0, 0).determinant() >
            config_.xy_covariance_reset_threshold)
        {
            p_.block<2, kStateSize - 2>(0, 2).setZero();
            p_.block<kStateSize - 2, 2>(2, 0).setZero();
            p_.block<2, 2>(0, 0) *= config_.xy_covariance_reset_factor;
        }

        // Насыщение смещения. Пока опоры нет, смещение ненаблюдаемо и держится
        // на последнем значении; ограничение нужно на случай, когда длительная
        // ненаблюдаемость всё же даёт ему уползти.
        if (config_.estimate_accel_bias)
        {
            const double bias_norm = xhat_.segment<3>(kAccelBiasIdx).norm();
            if (std::isfinite(bias_norm) && bias_norm > config_.max_accel_bias)
            {
                xhat_.segment<3>(kAccelBiasIdx) *=
                    config_.max_accel_bias / bias_norm;
            }
        }

        if (!xhat_.allFinite() || !p_.allFinite() ||
            xhat_.segment<3>(kVelocityIdx).norm() > config_.max_velocity)
        {
            Reset();
            ++resets_;
            output_.trust = trust;
            output_.frozen_phase = frozen;
            output_.zupt_active = false;
            output_.valid = false;
            return output_;
        }

        PublishState();
        return output_;
    }

    // Строка матрицы наблюдений имеет вид h = e_plus - e_minus, где minus < 0
    // означает, что вычитаемого слагаемого нет. Строки 0..11 измеряют
    // p - p_foot_i, строки 12..23 — скорость корпуса, строки 24..27 — высоту
    // стопы над грунтом.
    struct MeasurementRow
    {
        int plus;
        int minus;
    };

    [[nodiscard]] static MeasurementRow RowOf(int row) noexcept
    {
        if (row < 12)
        {
            const int leg = row / 3;
            const int axis = row % 3;
            return MeasurementRow{
                kPositionIdx + axis, kFootIdx + 3 * leg + axis};
        }
        if (row < 24)
        {
            return MeasurementRow{kVelocityIdx + (row - 12) % 3, -1};
        }
        // Высота стопы НАД СВОЕЙ опорой. При estimate_terrain_height == false
        // столбец g_i нулевой, и строка вырождается в прежнюю абсолютную.
        const int leg = row - 24;
        return MeasurementRow{kFootIdx + 3 * leg + 2, kTerrainIdx + leg};
    }

    // x = A x + B u и P = A P A^T + Q для
    //
    //       | I  dt*I  -0.5*dt^2*R  0 |
    //   A = | 0   I       -dt*R     0 |
    //       | 0   0         I       0 |
    //       | 0   0         0       I |
    //
    // где R = world_R_body переносит смещение акселерометра из системы корпуса,
    // где оно постоянно, в мировую, где интегрируется скорость. Состояния стоп
    // не двигаются. Записано структурно: это O(n^2) вместо O(n^3) и не
    // задействует блочные ядра Eigen (см. пояснение про -mavx2 в Update).
    void Predict(
        const Eigen::Vector3d& accel_world_raw,
        const Eigen::Matrix3d& world_R_body) noexcept
    {
        const double dt = config_.dt;
        const double half_dt_squared = 0.5 * dt * dt;

        const Eigen::Vector3d accel_world =
            accel_world_raw - world_R_body * xhat_.segment<3>(kAccelBiasIdx);

        xhat_.head<3>() +=
            dt * xhat_.segment<3>(kVelocityIdx) + half_dt_squared * accel_world;
        xhat_.segment<3>(kVelocityIdx) += dt * accel_world;
        // Смещение моделируется постоянным, его строка в A единичная.

        // Сначала строки (A * P), затем столбцы уже обновлённой матрицы
        // ((A * P) * A^T). Порядок внутри прохода обязателен: блок положения
        // читает исходный блок скорости, поэтому связь со смещением идёт
        // вторым проходом — там оба блока читают только блок смещения, а он не
        // меняется никогда.
        for (int axis = 0; axis < 3; ++axis)
        {
            p_.row(kPositionIdx + axis) += dt * p_.row(kVelocityIdx + axis);
        }
        for (int axis = 0; axis < 3; ++axis)
        {
            for (int component = 0; component < 3; ++component)
            {
                const double rotation = world_R_body(axis, component);
                p_.row(kPositionIdx + axis) -=
                    (half_dt_squared * rotation) * p_.row(kAccelBiasIdx + component);
                p_.row(kVelocityIdx + axis) -=
                    (dt * rotation) * p_.row(kAccelBiasIdx + component);
            }
        }

        for (int axis = 0; axis < 3; ++axis)
        {
            p_.col(kPositionIdx + axis) += dt * p_.col(kVelocityIdx + axis);
        }
        for (int axis = 0; axis < 3; ++axis)
        {
            for (int component = 0; component < 3; ++component)
            {
                const double rotation = world_R_body(axis, component);
                p_.col(kPositionIdx + axis) -=
                    (half_dt_squared * rotation) * p_.col(kAccelBiasIdx + component);
                p_.col(kVelocityIdx + axis) -=
                    (dt * rotation) * p_.col(kAccelBiasIdx + component);
            }
        }

        // Возврат высоты опоры к ground_height: строка A у g_i равна не 1, а
        // exp(-dt/tau). Блок терраина в A диагонален и с остальными не связан,
        // поэтому строку и столбец достаточно домножить по отдельности —
        // диагональ получает множитель дважды, как и должно быть у a * P * a.
        if (terrain_decay_ != 1.0)
        {
            for (int leg = 0; leg < kLegCount; ++leg)
            {
                const int terrain = kTerrainIdx + leg;
                xhat_(terrain) = config_.ground_height +
                    terrain_decay_ * (xhat_(terrain) - config_.ground_height);
                p_.row(terrain) *= terrain_decay_;
                p_.col(terrain) *= terrain_decay_;
            }
        }

        p_ += q_;
    }

    // Скалярное обновление по одной строке. Возвращает false, если строка
    // вырождена: вызывающая сторона откатывает такт целиком.
    [[nodiscard]] bool ScalarUpdate(int row, double measurement, double noise) noexcept
    {
        return ScalarUpdateRow(RowOf(row), measurement, noise);
    }

    [[nodiscard]] bool ScalarUpdateRow(
        const MeasurementRow h, double measurement, double noise) noexcept
    {
        ph_ = p_.col(h.plus);
        double predicted = xhat_(h.plus);
        double innovation_variance = 0.0;
        if (h.minus >= 0)
        {
            ph_ -= p_.col(h.minus);
            predicted -= xhat_(h.minus);
            innovation_variance = ph_(h.plus) - ph_(h.minus);
        }
        else
        {
            innovation_variance = ph_(h.plus);
        }

        const double s = innovation_variance + noise;
        const double innovation = measurement - predicted;
        if (!std::isfinite(s) || s <= 0.0 || !std::isfinite(innovation) ||
            !ph_.allFinite())
        {
            return false;
        }

        const StateVector gain = ph_ / s;
        xhat_ += innovation * gain;
        // P -= (P h)(P h)^T / s — обновление ранга 1, симметричное по построению.
        p_.noalias() -= gain * ph_.transpose();
        return true;
    }

    // Высота грунта из карты высот как прямое измерение состояния g_i.
    [[nodiscard]] bool ApplyTerrainMap(
        const KalmanMITInput& input,
        const std::array<double, 4>& trust) noexcept
    {
        if (!config_.estimate_terrain_height)
        {
            return true;
        }

        for (int leg = 0; leg < kLegCount; ++leg)
        {
            // Только по доверенной ноге: под маховой ногой карта говорит не о
            // той точке, где нога встанет.
            if (!input.terrain_height[leg].has_value() ||
                trust[leg] <= 0.0)
            {
                continue;
            }
            const double height = *input.terrain_height[leg];
            if (!std::isfinite(height))
            {
                continue;
            }
            if (!ScalarUpdateRow(
                    MeasurementRow{kTerrainIdx + leg, -1},
                    height,
                    config_.terrain_map_noise))
            {
                return false;
            }
        }
        return true;
    }

    // Ворота ZUPT: опоры нет ни под одной ногой И корпус физически неподвижен.
    // Держит счётчик выдержки и защёлку якоря, поэтому вызывается ровно раз за
    // такт, до любой отбраковки.
    [[nodiscard]] bool UpdateZuptGate(
        const KalmanMITInput& input,
        const std::array<double, 4>& trust) noexcept
    {
        if (!config_.zupt_enabled)
        {
            zupt_static_ticks_ = 0U;
            zupt_engaged_ = false;
            return false;
        }

        bool unsupported = true;
        for (int leg = 0; leg < kLegCount; ++leg)
        {
            if (trust[leg] > config_.zupt_trust_threshold)
            {
                unsupported = false;
                break;
            }
        }

        // Модуль удельной силы, а не её проекция: в покое акселерометр меряет
        // только реакцию на гравитацию при любом наклоне корпуса, поэтому
        // ворота не зависят от ориентации и не наследуют её ошибку. Свободный
        // полёт даёт |a| ~ 0 и ворота не проходит.
        const double accel_error =
            std::fabs(input.accel_body.norm() - config_.gravity);
        const bool still = accel_error <= config_.zupt_accel_tolerance &&
            input.omega_body.norm() <= config_.zupt_omega_tolerance;

        if (!unsupported || !still)
        {
            zupt_static_ticks_ = 0U;
            zupt_engaged_ = false;
            return false;
        }

        if (zupt_static_ticks_ < kStallCounterMax)
        {
            ++zupt_static_ticks_;
        }
        return zupt_static_ticks_ >= zupt_hold_ticks_;
    }

    // Псевдоизмерения неподвижности: v = 0 и, если включено, p = якорь.
    // Строки наблюдения те же по форме, что у штатных измерений скорости и
    // положения (h = e_i), поэтому идут через тот же скалярный путь и так же не
    // задействуют блочные ядра Eigen.
    //
    // Якорь защёлкивается один раз на входе в режим и берётся от оценки на
    // начало такта. Пересчитывать его каждый такт нельзя: это превратило бы
    // измерение в тождество и вернуло бы уход, ради которого всё и затевалось.
    [[nodiscard]] bool ApplyZupt(const Eigen::Vector3d& anchor_candidate) noexcept
    {
        if (!zupt_engaged_)
        {
            zupt_engaged_ = true;
            zupt_anchor_ = anchor_candidate;
        }

        for (int axis = 0; axis < 3; ++axis)
        {
            if (!ScalarUpdateRow(
                    MeasurementRow{kVelocityIdx + axis, -1},
                    0.0,
                    config_.zupt_measurement_noise))
            {
                return false;
            }
        }

        if (!config_.zupt_hold_position)
        {
            return true;
        }

        for (int axis = 0; axis < 3; ++axis)
        {
            if (!ScalarUpdateRow(
                    MeasurementRow{kPositionIdx + axis, -1},
                    zupt_anchor_(axis),
                    config_.zupt_position_measurement_noise))
            {
                return false;
            }
        }
        return true;
    }

    struct LegKinematicsInternal
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        std::array<Eigen::Vector3d, 4> p_rel{};
        std::array<Eigen::Vector3d, 4> dp_rel{};
        bool valid{false};
    };

    [[nodiscard]] LegKinematicsInternal ComputeLegKinematics(
        const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& joint_positions,
        const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& joint_velocities)
    {
        LegKinematicsInternal result;
        if (!joint_positions.allFinite() || !joint_velocities.allFinite())
        {
            return result;
        }

        q_config_.setZero();
        q_config_(6) = 1.0;
        v_config_.setZero();
        for (int leg = 0; leg < kLegCount; ++leg)
        {
            const std::size_t pin = kServoToPinocchio[leg];
            q_config_.segment<3>(kPinocchioJointQIndex[pin]) =
                joint_positions.segment<3>(3 * leg);
            v_config_.segment<3>(kPinocchioJointVIndex[pin]) =
                joint_velocities.segment<3>(3 * leg);
        }

        // Порядок обязателен: Robot::GetFootJacobian читает ориентацию базы из
        // data_->oMf, а computeJointJacobians её не обновляет. Якобиан верен
        // только сразу после прямой кинематики с тем же q.
        robot_.ComputeForwardKinematics(q_config_, v_config_);
        const std::vector<Eigen::Vector3d> positions =
            robot_.GetToePositionsInBaseFrame();
        const std::vector<Eigen::Matrix3d> jacobians =
            robot_.GetFootJacobian(q_config_);
        if (positions.size() != static_cast<std::size_t>(kLegCount) ||
            jacobians.size() != static_cast<std::size_t>(kLegCount))
        {
            return result;
        }

        for (int leg = 0; leg < kLegCount; ++leg)
        {
            const std::size_t pin = kServoToPinocchio[leg];
            result.p_rel[leg] = positions[pin];
            result.dp_rel[leg] =
                jacobians[pin] * joint_velocities.segment<3>(3 * leg);
            if (!result.p_rel[leg].allFinite() || !result.dp_rel[leg].allFinite())
            {
                return result;
            }
        }
        result.valid = true;
        return result;
    }

    void ComputeTrust(
        const KalmanMITInput& input,
        std::array<double, 4>& trust,
        std::array<bool, 4>& frozen) noexcept
    {
        for (int leg = 0; leg < kLegCount; ++leg)
        {
            const double phi = input.gait_phases[leg];
            const bool stalled = has_prev_phi_ && std::isfinite(phi) &&
                std::fabs(phi - prev_phi_[leg]) < config_.frozen_phase_epsilon;
            if (stalled)
            {
                if (phase_stall_ticks_[leg] < kStallCounterMax)
                {
                    ++phase_stall_ticks_[leg];
                }
            }
            else
            {
                phase_stall_ticks_[leg] = 0U;
            }
            prev_phi_[leg] = phi;

            frozen[leg] = config_.frozen_phase_fallback &&
                (!input.has_gait_phase ||
                 phase_stall_ticks_[leg] >= frozen_phase_ticks_);

            if (frozen[leg])
            {
                trust[leg] = input.contacts[leg] ? 1.0 : 0.0;
            }
            else
            {
                const double scheduled = KalmanMIT::PhaseTrust(
                    phi, input.contact_states[leg]);
                const double floor =
                    (input.contacts[leg] && input.contact_states[leg] != SWING)
                        ? config_.contact_trust_floor
                        : 0.0;
                trust[leg] = std::max(scheduled, floor);
            }
            trust[leg] = std::clamp(trust[leg], 0.0, 1.0);
        }
        has_prev_phi_ = true;
    }

    [[nodiscard]] const KalmanMITOutput& Initialize(
        const KalmanMITInput& input,
        const std::array<Eigen::Vector3d, 4>& p_f)
    {
        std::size_t contacts = 0;
        double summed_foot_z = 0.0;
        for (int leg = 0; leg < kLegCount; ++leg)
        {
            if (input.contacts[leg])
            {
                ++contacts;
                summed_foot_z += p_f[leg].z();
            }
        }
        if (contacts < config_.min_init_contacts || contacts == 0)
        {
            output_.valid = false;
            return output_;
        }

        xhat_.setZero();
        xhat_(2) = config_.ground_height -
            summed_foot_z / static_cast<double>(contacts);
        for (int leg = 0; leg < kLegCount; ++leg)
        {
            xhat_.segment<3>(kFootIdx + 3 * leg) = xhat_.head<3>() + p_f[leg];
            // На старте про рельеф ничего не известно, а корпус только что
            // поставлен от плоскости ground_height — значит и опоры на ней.
            // Дальше каждая нога уточнит свою при первой же постановке.
            xhat_(kTerrainIdx + leg) = config_.ground_height;
        }
        p_ = config_.initial_covariance * StateMatrix::Identity();
        // Собственная начальная дисперсия по тем же причинам, что у смещения
        // акселерометра: initial_covariance = 100 м^2 означал бы, что про
        // высоту опоры не известно ничего, и первые такты были бы рывковыми.
        // При выключенной оценке нулевой блок навсегда обнуляет усиление по
        // g_i, и фильтр такт-в-такт повторяет прежний 21-мерный.
        p_.block<4, 4>(kTerrainIdx, kTerrainIdx) =
            (config_.estimate_terrain_height
                 ? config_.initial_terrain_covariance
                 : 0.0) * Eigen::Matrix4d::Identity();
        // Смещение стартует с нуля и со своей собственной начальной ковариацией:
        // initial_covariance = 100 (м/с^2)^2 сделал бы первые такты рывковыми.
        // При выключенной оценке нулевой блок навсегда обнуляет усиление по
        // смещению, и фильтр такт-в-такт повторяет прежний 18-мерный.
        p_.block<3, 3>(kAccelBiasIdx, kAccelBiasIdx) =
            (config_.estimate_accel_bias
                 ? config_.initial_accel_bias_covariance
                 : 0.0) * Eigen::Matrix3d::Identity();
        initialized_ = true;

        PublishState();
        return output_;
    }

    void PublishState() noexcept
    {
        output_.position = xhat_.head<3>();
        output_.velocity = xhat_.segment<3>(kVelocityIdx);
        output_.accel_bias = xhat_.segment<3>(kAccelBiasIdx);
        for (int leg = 0; leg < kLegCount; ++leg)
        {
            output_.foot_positions_world[leg] =
                xhat_.segment<3>(kFootIdx + 3 * leg);
            output_.terrain_heights[leg] = xhat_(kTerrainIdx + leg);
        }
        output_.valid = true;
    }

    [[nodiscard]] const KalmanMITOutput& RejectTick() noexcept
    {
        ++rejected_updates_;
        output_.zupt_active = false;
        output_.valid = false;
        return output_;
    }

    [[nodiscard]] static bool InputIsFinite(const KalmanMITInput& input) noexcept
    {
        if (!input.joint_positions.allFinite() ||
            !input.joint_velocities.allFinite() ||
            !input.world_R_body.allFinite() ||
            !input.omega_body.allFinite() ||
            !input.accel_body.allFinite())
        {
            return false;
        }
        const Eigen::Matrix3d should_be_identity =
            input.world_R_body.transpose() * input.world_R_body;
        return (should_be_identity - Eigen::Matrix3d::Identity())
                   .cwiseAbs()
                   .maxCoeff() < kRotationTolerance;
    }

    static constexpr unsigned kStallCounterMax = 1000000U;

    KalmanMITConfig config_;
    Robot robot_;
    Eigen::VectorXd q_config_;
    Eigen::VectorXd v_config_;

    StateMatrix q_base_{StateMatrix::Zero()};
    StateMatrix q_{StateMatrix::Zero()};
    StateMatrix p_{StateMatrix::Zero()};
    StateMatrix p_prev_{StateMatrix::Zero()};
    StateVector xhat_{StateVector::Zero()};
    StateVector xhat_prev_{StateVector::Zero()};
    StateVector ph_{StateVector::Zero()};
    MeasurementVector y_{MeasurementVector::Zero()};
    MeasurementVector r_base_diag_{MeasurementVector::Zero()};
    MeasurementVector r_diag_{MeasurementVector::Zero()};

    std::array<double, 4> prev_phi_{};
    std::array<unsigned, 4> phase_stall_ticks_{};
    unsigned frozen_phase_ticks_{0U};
    // exp(-dt / terrain_relaxation_time_sec); 1.0 означает «возврат отключён».
    double terrain_decay_{1.0};
    bool has_prev_phi_{false};
    bool initialized_{false};

    Eigen::Vector3d zupt_anchor_{Eigen::Vector3d::Zero()};
    unsigned zupt_hold_ticks_{0U};
    unsigned zupt_static_ticks_{0U};
    bool zupt_engaged_{false};

    KalmanMITOutput output_{};
    std::uint64_t rejected_updates_{0};
    std::uint64_t resets_{0};
};

KalmanMIT::KalmanMIT(const KalmanMITConfig& config)
    : impl_(std::make_unique<Impl>(config))
{
}

KalmanMIT::~KalmanMIT() = default;

const KalmanMITOutput& KalmanMIT::Update(const KalmanMITInput& input)
{
    return impl_->Update(input);
}

void KalmanMIT::Reset() noexcept
{
    impl_->Reset();
}

bool KalmanMIT::initialized() const noexcept
{
    return impl_->initialized_;
}

const KalmanMITOutput& KalmanMIT::output() const noexcept
{
    return impl_->output_;
}

const KalmanMIT::StateVector& KalmanMIT::state() const noexcept
{
    return impl_->xhat_;
}

const KalmanMIT::StateMatrix& KalmanMIT::covariance() const noexcept
{
    return impl_->p_;
}

std::uint64_t KalmanMIT::rejected_updates() const noexcept
{
    return impl_->rejected_updates_;
}

std::uint64_t KalmanMIT::resets() const noexcept
{
    return impl_->resets_;
}

KalmanMIT::LegKinematics KalmanMIT::ComputeLegKinematics(
    const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& joint_positions,
    const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& joint_velocities)
{
    const Impl::LegKinematicsInternal internal =
        impl_->ComputeLegKinematics(joint_positions, joint_velocities);
    LegKinematics result;
    result.p_rel = internal.p_rel;
    result.dp_rel = internal.dp_rel;
    result.valid = internal.valid;
    return result;
}

double KalmanMIT::PhaseTrust(double phi, int contact_state) noexcept
{
    // Единственный источник истины для оконного доверия — общий ReliableContact,
    // чтобы оценщики не разъезжались при правке ширины окна.
    static const ReliableContact reliable_contact{};
    return static_cast<double>(
        reliable_contact.get_trust_coefficient(phi, contact_state));
}

}  // namespace state_estimator_hmb
