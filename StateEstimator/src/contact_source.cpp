#include "contact_source.hpp"

#include <stdexcept>
#include <string>

namespace state_estimator_hmb
{

ContactSource ParseContactSource(std::string_view value)
{
    if (value == "grf_observer")
    {
        return ContactSource::GrfObserver;
    }
    if (value == "sensor")
    {
        return ContactSource::Sensor;
    }

    throw std::invalid_argument(
        "contact_source must be 'sensor' or 'grf_observer'; got '" +
        std::string(value) + "'.");
}

std::string_view ToString(ContactSource source) noexcept
{
    switch (source)
    {
        case ContactSource::GrfObserver:
            return "grf_observer";
        case ContactSource::Sensor:
            return "sensor";
    }

    return "unknown";
}

bool SelectContactState(
    ContactSource source,
    bool grf_observer_contact,
    bool sensor_contact) noexcept
{
    switch (source)
    {
        case ContactSource::GrfObserver:
            return grf_observer_contact;
        case ContactSource::Sensor:
            return sensor_contact;
    }

    return grf_observer_contact;
}

}  // namespace state_estimator_hmb
