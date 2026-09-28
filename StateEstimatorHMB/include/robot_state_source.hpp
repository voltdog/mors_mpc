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

}  // namespace state_estimator_hmb

#endif  // STATE_ESTIMATOR_HMB_ROBOT_STATE_SOURCE_HPP
