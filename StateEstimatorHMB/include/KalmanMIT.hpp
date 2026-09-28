#ifndef STATE_ESTIMATOR_HMB_KALMAN_MIT_HPP
#define STATE_ESTIMATOR_HMB_KALMAN_MIT_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

#include <Eigen/Core>

namespace state_estimator_hmb
{

// Линейный фильтр Калмана положения и скорости корпуса в стиле MIT Cheetah
// (`LinearKFPositionVelocityEstimator` из mit-biomimetics/Cheetah-Software).
//
// Состояние: x = [p(3), v(3), b_a(3), p_foot_i(3) x4, g_i(1) x4] = 25.
// Положение, скорость и стопы — в мировой системе, смещение акселерометра b_a —
// в системе корпуса, где оно физически постоянно. g_i — мировая высота опорной
// поверхности под i-й стопой (см. estimate_terrain_height).
// Измерения (28): относительное положение стопы (12), скорость корпуса по
// кинематике ног (12), высота стопы над её опорой (4).
//
// Фильтр использует только ИМУ и кинематику ног — внешняя одометрия (T265) в
// него не входит, поэтому его оценку можно честно сравнивать с одометрией.
// Дрейф x/y при этом ожидаем: абсолютного датчика положения у фильтра нет.
//
// Все массивы длины 4 индексируются сервоприводным порядком ног: R1, L1, R2, L2.
struct KalmanMITConfig
{
    // Период контура. Источник истины — `config/timesteps.yaml: state_estimator_dt`.
    double dt{0.002};

    // --- Шум процесса. Это ДИСПЕРСИИ, а не СКО (соглашение MIT). ---
    double imu_process_noise_position{0.02};
    double imu_process_noise_velocity{0.02};
    double foot_process_noise_position{0.002};

    // --- Смещение акселерометра. ---
    // В оригинале MIT состояния смещения нет: там опора не теряется, и якорь по
    // высоте стопы держит Z при любом смещении. Как только все ноги
    // разгружаются (робот ложится), измерения становятся самоссылочными,
    // и некомпенсированное смещение интегрируется дважды: по логам
    // ~/mors_logs/log_260909_13* это -0.06 м/с^2 по Z и -0.31 м/с^2 по X, что
    // за 3.3 с даёт -0.32 м по Z и -1.7 м по X. Смещение наблюдаемо, пока хотя
    // бы одна нога доверенная, а на разгруженных ногах замирает на последнем
    // значении и продолжает компенсировать.
    //
    // false отключает оценку: блоки P и Q смещения зануляются, b_a остаётся
    // ровно нулём, и фильтр такт-в-такт совпадает с прежним 18-мерным.
    bool estimate_accel_bias{true};
    // Дисперсия случайного блуждания смещения, (м/с^2)^2 в секунду.
    double accel_bias_process_noise{1.0e-6};
    double initial_accel_bias_covariance{0.25};
    // Насыщение оценки смещения, м/с^2. Защита от увода при длительном
    // отсутствии наблюдаемости.
    double max_accel_bias{1.0};

    // --- Шум измерений (тоже дисперсии). ---
    double foot_sensor_noise_position{0.001};
    double foot_sensor_noise_velocity{0.1};
    double foot_height_sensor_noise{0.001};

    // Раздувание Q и R недоверенной ноги: (1 + (1 - trust) * high_suspect_number).
    double high_suspect_number{100.0};

    // MIT раздувает только блоки скорости (12..23) и высоты стопы (24..27), а
    // блок относительного положения (0..11) оставляет нетронутым: это чистая
    // кинематика, одинаково верная и для опорной, и для маховой ноги, и именно
    // она позволяет состоянию маховой стопы следовать за корпусом. Включение
    // делает маховую стопу дважды ненаблюдаемой (большие Q и R сразу) и портит
    // обусловленность S через несколько циклов походки.
    bool scale_position_measurement_noise{false};

    // --- Привязка к грунту. ---
    // Именно измерение высоты стопы убирает дрейф Z. Без него оценка опирается
    // только на относительные величины и медленно уходит.
    bool use_foot_height_measurement{true};
    // Высота грунта при estimate_terrain_height == false, а также начальное
    // значение состояний g_i и уровень, от которого отсчитывается корпус при
    // инициализации.
    double ground_height{0.0};

    // --- Оценка высоты грунта под каждой ногой. ---
    // false воспроизводит прежний 21-мерный фильтр такт-в-такт: g_i жёстко
    // равны ground_height, их блоки P и Q нулевые, и строки 24..27 вырождаются
    // в прежнее «стопа на плоскости».
    //
    // true делает g_i состояниями. Тогда строка 24+i измеряет не абсолютную
    // высоту стопы, а её высоту НАД СВОЕЙ опорой: p_foot_i.z - g_i = 0. Это
    // снимает допущение плоского грунта, из-за которого на ступеньке фильтр
    // сажал корпус на среднюю высоту опор: по логу log_260910_135248 (платформа
    // 50 мм) ошибка z доходила до -49 мм, когда все четыре ноги стояли на
    // платформе, и до -28 мм, когда на ней стояла половина ног.
    //
    // Наблюдаемость сохраняется за счёт того, что g_i почти заморожена (Q ~ 0),
    // пока нога доверенная: замороженная g_i через жёсткую строку 24+i держит
    // стопу, а стопа через строки 0..11 держит корпус. Абсолютную высоту помнит
    // именно g_i, а не допущение о плоскости.
    bool estimate_terrain_height{true};

    // Дисперсия случайного блуждания высоты опоры под ДОВЕРЕННОЙ ногой, м^2/с.
    // Это не рельеф, а просадка опоры и уползание стопы в контакте. Большие
    // значения возвращают дрейф Z: g_i перестаёт быть якорем.
    double terrain_process_noise{1.0e-8};

    // То же под НЕДОВЕРЕННОЙ ногой, м^2/с. Пока нога в переносе, высота опоры
    // под ней просто неизвестна — где нога встанет, кинематика не знает.
    // Раздутый Q и есть всё «перезакрепление»: за перенос дисперсия g_i успевает
    // вырасти на порядки выше дисперсии стопы, поэтому на постановке жёсткая
    // строка 24+i переносит поправку почти целиком на g_i, а не на корпус, и
    // g_i сама садится на новый уровень. Плавно, по окну доверия, без детектора
    // фронта: дискретное защёлкивание в момент касания ловило стопу то на
    // снижении, то уже на отрыве и давало ратчет вверх до +4 мм/с на ровном
    // участке лога log_260910_135248.
    //
    // Фактическое смещение за перенос: sqrt(Q * T_sw). 1.0 м^2/с при переносе
    // 0.15 с — это СКО около 0.4 м, то есть заведомо больше любой ступеньки.
    double terrain_swing_process_noise{1.0};

    // Постоянная времени возврата g_i к ground_height, с. 0 отключает возврат.
    //
    // Возврат обязателен, и вот почему. Если высота опоры под каждой ногой —
    // свободный параметр, у вертикали появляется калибровочная свобода:
    // одновременный сдвиг корпуса, всех стоп и всех g_i не меняет ни одного
    // измерения. ИМУ и кинематика такой сдвиг не видят в принципе, поэтому он
    // накапливается. На логе log_260910_134406 (120 с непрерывной рыси) без
    // возврата оценка уезжала на +0.6 м, хотя T265 держался на 0.22.
    //
    // Возврат делает g_i процессом Орнштейна — Уленбека вместо случайного
    // блуждания: постоянный увод r оседает на r * tau вместо роста без предела.
    // Цена — ступенька забывается с той же постоянной: простояв на платформе
    // время tau, оценка потеряет примерно 63 % её высоты. То есть tau — это
    // прямой размен «сколько держим ступеньку» против «насколько уезжаем», и
    // выбирать его нужно по логам, а не из общих соображений.
    //
    // Настоящее решение размена — не tau, а абсолютный источник высоты грунта:
    // заполненный KalmanMITInput::terrain_height из карты высот убирает
    // калибровочную свободу, и tau можно уводить в бесконечность.
    double terrain_relaxation_time_sec{10.0};

    // Начальная дисперсия g_i, м^2. Применяется на инициализации, когда высота
    // опоры под каждой ногой берётся равной ground_height.
    double initial_terrain_covariance{1.0e-4};

    // Дисперсия измерения высоты грунта из карты высот, м^2. Работает только по
    // тем ногам, для которых заполнено KalmanMITInput::terrain_height, и только
    // пока нога доверенная. 2.5e-3 — это СКО 5 см. Заполненное поле снимает
    // калибровочную свободу вертикали, ради которой существует
    // terrain_relaxation_time_sec.
    double terrain_map_noise{2.5e-3};

    // --- Инициализация и численная защита. ---
    double initial_covariance{100.0};
    std::size_t min_init_contacts{1};
    double xy_covariance_reset_threshold{1.0e-6};
    double xy_covariance_reset_factor{0.1};
    double max_velocity{6.0};
    double gravity{9.81};

    // --- Политика доверия к ноге. ---
    // В режиме стойки планировщик паркует phi на 0.992..0.996 при phase = STANCE,
    // из-за чего оконное доверие падает до ~0.03: измерение скорости становится
    // на 97 % самоссылочным, и фильтр уходит в счисление по ИМУ. Если phi не
    // двигается дольше frozen_phase_timeout_sec, переходим на бинарный флаг
    // контакта. Пока phi идёт, поведение бит-в-бит совпадает с MIT.
    bool frozen_phase_fallback{true};
    double frozen_phase_epsilon{1.0e-6};
    double frozen_phase_timeout_sec{0.2};
    // Нижняя граница доверия по замкнутому датчику контакта вне режима SWING.
    double contact_trust_floor{0.0};

    // --- ZUPT: псевдоизмерение неподвижности при полной потере опоры. ---
    // Когда доверия нет ни к одной ноге, измерения скорости и высоты стопы
    // становятся самоссылочными, и фильтр переходит в чистое счисление по ИМУ
    // вместе с накопленной ошибкой смещения. По логу ~/mors_logs/log_260909_135813
    // это 0.07 м/с^2 в горизонте: за 5.4 с лежания скорость доходит до 0.39 м/с,
    // а положение уезжает на 1.08 м, хотя сырой ИМУ в том же окне даёт
    // горизонтальное ускорение 0.003 м/с^2 — уезжает именно фильтр.
    //
    // Но опоры нет ещё не значит, что корпус движется. Если удельная сила равна
    // g, а угловая скорость нулевая, робот физически неподвижен (лежит на
    // брюхе), и это само по себе измерение: v = 0. Оно же возвращает
    // наблюдаемость горизонтальному смещению акселерометра, которое ходьба
    // испортила: неподвижный корпус с известной ориентацией — эталонное условие
    // для его оценки.
    bool zupt_enabled{true};
    // «Опоры нет» = доверие каждой ноги не выше порога. 0.0 — строго нулевое.
    double zupt_trust_threshold{0.0};
    // Ворота неподвижности. Сравнение идёт по модулю удельной силы, поэтому
    // ворота не зависят от ориентации корпуса, а свободный полёт (|a| ~ 0) их
    // не проходит. Удельная сила берётся сырой, без вычета b_a: смысл ворот в
    // том, чтобы не зависеть от состояния, которое они же и лечат. По тому же
    // логу на лежащем роботе | |a| - g | не превышает 0.30 м/с^2 (95-й
    // процентиль), |w| — 0.009 рад/с, тогда как в ходьбе медианы 0.83 м/с^2 и
    // 0.49 рад/с: запас почти в два порядка.
    //
    // Единственный случай, который ворота пропускают ошибочно, — робота несут
    // строго равномерно и без поворота. Инерциальными датчиками он от покоя
    // неотличим в принципе, и опоры в этот момент всё равно нет.
    double zupt_accel_tolerance{0.4};
    double zupt_omega_tolerance{0.1};
    // Сколько времени ворота должны держаться подряд до включения ZUPT.
    // Отсекает удары о грунт и одиночные выбросы.
    double zupt_hold_sec{0.1};
    // Дисперсия псевдоизмерения скорости, (м/с)^2. 1e-4 — это СКО 1 см/с.
    double zupt_measurement_noise{1.0e-4};
    // Дополнительно держать положение на защёлкнутом якоре. Одна только
    // нулевая скорость оставляет уход второго порядка 0.5*dt*a_err*T (доли
    // миллиметра за такт), якорь убирает и его.
    bool zupt_hold_position{true};
    double zupt_position_measurement_noise{1.0e-4};
};

struct KalmanMITInput
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Порядок сервоприводов R1, L1, R2, L2.
    Eigen::Matrix<double, 12, 1> joint_positions{
        Eigen::Matrix<double, 12, 1>::Zero()};
    Eigen::Matrix<double, 12, 1> joint_velocities{
        Eigen::Matrix<double, 12, 1>::Zero()};

    Eigen::Matrix3d world_R_body{Eigen::Matrix3d::Identity()};
    // Угловая скорость и удельная сила ИМУ, обе в системе корпуса. Ускорение
    // подаётся сырым, вместе с гравитацией: фильтр сам считает
    // a_world = R * a_body + (0, 0, -g).
    Eigen::Vector3d omega_body{Eigen::Vector3d::Zero()};
    Eigen::Vector3d accel_body{Eigen::Vector3d::Zero()};

    std::array<bool, 4> contacts{};
    std::array<double, 4> gait_phases{};
    std::array<int, 4> contact_states{1, 1, 1, 1};  // STANCE
    bool has_gait_phase{false};

    // Покадровая высота грунта под стопой из карты высот. Пустое значение
    // означает, что карта по этой ноге ничего не говорит, и высота опоры
    // остаётся на том, что фильтр запомнил при постановке ноги. Заполненное —
    // измерение состояния g_i с дисперсией config.terrain_map_noise.
    std::array<std::optional<double>, 4> terrain_height{};
};

struct KalmanMITOutput
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    // Смещение акселерометра в системе корпуса, м/с^2.
    Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
    std::array<Eigen::Vector3d, 4> foot_positions_world{};
    // Оценка высоты опорной поверхности под каждой стопой, мировая система.
    // При estimate_terrain_height == false всегда равна ground_height.
    std::array<double, 4> terrain_heights{};
    std::array<double, 4> trust{};
    std::array<bool, 4> frozen_phase{};
    // Псевдоизмерение неподвижности применено на этом такте.
    bool zupt_active{false};
    bool valid{false};
};

class KalmanMIT final
{
public:
    static constexpr int kLegCount = 4;
    static constexpr int kStateSize = 25;
    static constexpr int kMeasurementSize = 28;

    // Смещения блоков в векторе состояния.
    static constexpr int kPositionIdx = 0;
    static constexpr int kVelocityIdx = 3;
    static constexpr int kAccelBiasIdx = 6;
    static constexpr int kFootIdx = 9;
    static constexpr int kTerrainIdx = 21;

    using StateVector = Eigen::Matrix<double, kStateSize, 1>;
    using StateMatrix = Eigen::Matrix<double, kStateSize, kStateSize>;

    struct LegKinematics
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        std::array<Eigen::Vector3d, kLegCount> p_rel{};   // система корпуса
        std::array<Eigen::Vector3d, kLegCount> dp_rel{};  // система корпуса, только суставы
        bool valid{false};
    };

    explicit KalmanMIT(const KalmanMITConfig& config = KalmanMITConfig{});
    ~KalmanMIT();

    KalmanMIT(const KalmanMIT&) = delete;
    KalmanMIT& operator=(const KalmanMIT&) = delete;

    // В установившемся режиме не бросает: на плохом входе возвращает выход с
    // valid == false и не трогает состояние фильтра.
    [[nodiscard]] const KalmanMITOutput& Update(const KalmanMITInput& input);

    void Reset() noexcept;

    [[nodiscard]] bool initialized() const noexcept;
    [[nodiscard]] const KalmanMITOutput& output() const noexcept;
    [[nodiscard]] const StateVector& state() const noexcept;
    [[nodiscard]] const StateMatrix& covariance() const noexcept;
    [[nodiscard]] std::uint64_t rejected_updates() const noexcept;
    [[nodiscard]] std::uint64_t resets() const noexcept;

    // Открыто ради тестов: позволяет проверить ремап сервопривод -> pinocchio и
    // якобиан ноги, не поднимая весь фильтр.
    [[nodiscard]] LegKinematics ComputeLegKinematics(
        const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& joint_positions,
        const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& joint_velocities);

    // Оконное доверие MIT по фазе походки. Совпадает с
    // `ReliableContact::get_trust_coefficient`, но доступно как чистая функция.
    [[nodiscard]] static double PhaseTrust(double phi, int contact_state) noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace state_estimator_hmb

#endif  // STATE_ESTIMATOR_HMB_KALMAN_MIT_HPP
