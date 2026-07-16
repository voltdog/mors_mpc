#ifndef STATE_ESTIMATOR_HMB_CONTACT_SOURCE_HPP
#define STATE_ESTIMATOR_HMB_CONTACT_SOURCE_HPP

#include <string_view>

namespace state_estimator_hmb
{

enum class ContactSource
{
    GrfObserver,
    Sensor,
};

ContactSource ParseContactSource(std::string_view value);

std::string_view ToString(ContactSource source) noexcept;

bool SelectContactState(
    ContactSource source,
    bool grf_observer_contact,
    bool sensor_contact) noexcept;

}  // namespace state_estimator_hmb

#endif  // STATE_ESTIMATOR_HMB_CONTACT_SOURCE_HPP
