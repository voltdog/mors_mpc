#ifndef STATE_ESTIMATOR_HMB_ROBOT_STATE_SOURCE_HPP
#define STATE_ESTIMATOR_HMB_ROBOT_STATE_SOURCE_HPP

#include <string_view>

namespace state_estimator_hmb
{

// Источник данных о позе и линейной скорости корпуса для публикуемого канала.
// Ориентация и угловая скорость в обоих случаях берутся из ИМУ.
enum class RobotStateSource
{
    Kalman,
    T265,
};

RobotStateSource ParseRobotStateSource(std::string_view value);

std::string_view ToString(RobotStateSource source) noexcept;

// Противоположный источник. Канал ROBOT_STATE_CHECK всегда формируется той
// оценкой, которая не выбрана для ROBOT_STATE, поэтому правило «check — это
// другой источник» задано ровно здесь и больше нигде не дублируется.
RobotStateSource Other(RobotStateSource source) noexcept;

// Ошибка до первого pose-кадра допускает резервный Kalman. После первого
// кадра потеря основной T265 фатальна, даже если state loop не успел его прочитать.
enum class T265State
{
    WaitingForPose,
    Streaming,
    Unavailable,
    Disconnected,
};

struct RobotStateSources
{
    RobotStateSource primary;
    RobotStateSource check;
    bool kalman_enabled;
    bool t265_valid;
    bool fatal;
};

RobotStateSources SelectRobotStateSources(
    RobotStateSource requested, bool kalman_enabled, T265State t265_state) noexcept;

}  // namespace state_estimator_hmb

#endif  // STATE_ESTIMATOR_HMB_ROBOT_STATE_SOURCE_HPP
