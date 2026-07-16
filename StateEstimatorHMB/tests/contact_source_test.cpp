#include "contact_source.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

using state_estimator_hmb::ContactSource;

void Check(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void TestParsing()
{
    Check(
        state_estimator_hmb::ParseContactSource("sensor") == ContactSource::Sensor,
        "sensor was parsed incorrectly");
    Check(
        state_estimator_hmb::ParseContactSource("grf_observer") ==
            ContactSource::GrfObserver,
        "grf_observer was parsed incorrectly");
    Check(
        state_estimator_hmb::ToString(ContactSource::Sensor) == "sensor",
        "sensor string representation is incorrect");
    Check(
        state_estimator_hmb::ToString(ContactSource::GrfObserver) ==
            "grf_observer",
        "grf_observer string representation is incorrect");
}

void TestInvalidSource()
{
    bool rejected = false;
    try
    {
        static_cast<void>(state_estimator_hmb::ParseContactSource("grf"));
    }
    catch (const std::invalid_argument&)
    {
        rejected = true;
    }

    Check(rejected, "unknown contact source was accepted");
}

void TestContactSelection()
{
    constexpr std::array<bool, 2> states{false, true};
    for (const bool grf_contact : states)
    {
        for (const bool sensor_contact : states)
        {
            Check(
                state_estimator_hmb::SelectContactState(
                    ContactSource::GrfObserver,
                    grf_contact,
                    sensor_contact) == grf_contact,
                "grf_observer did not select the observer contact");
            Check(
                state_estimator_hmb::SelectContactState(
                    ContactSource::Sensor,
                    grf_contact,
                    sensor_contact) == sensor_contact,
                "sensor did not select the sensor contact");
        }
    }

    const std::array<bool, 4> contacts_before_first_sensor_message{};
    for (const bool contact : contacts_before_first_sensor_message)
    {
        Check(
            !state_estimator_hmb::SelectContactState(
                ContactSource::Sensor,
                true,
                contact),
            "sensor mode fell back to the GRF observer before the first message");
    }
}

}  // namespace

int main()
{
    try
    {
        TestParsing();
        TestInvalidSource();
        TestContactSelection();
    }
    catch (const std::exception& error)
    {
        std::cerr << "contact_source_test failed: " << error.what() << '\n';
        return 1;
    }

    std::cout << "contact_source_test passed\n";
    return 0;
}
