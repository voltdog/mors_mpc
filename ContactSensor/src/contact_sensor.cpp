#include "contact_sensor/core.hpp"

#include "mors_msgs/contact_sensor_msg.hpp"

#include <lcm/lcm-cpp.hpp>
#include <yaml-cpp/yaml.h>

#include <asm/termbits.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>

namespace
{

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

constexpr unsigned int kBaudRate = 250000;
constexpr auto kReconnectPeriod = 1s;
constexpr std::size_t kReadBufferSize = 4096;

volatile std::sig_atomic_t stop_requested = 0;

void HandleSignal(int)
{
    stop_requested = 1;
}

std::string GetRequiredEnv(const char* name)
{
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0')
    {
        throw std::runtime_error(std::string(name) + " must be set");
    }
    return value;
}

std::string LoadContactChannel(const std::filesystem::path& path)
{
    const YAML::Node channels = YAML::LoadFile(path.string());
    const YAML::Node contact_channel = channels["contact_state"];
    if (!contact_channel || !contact_channel.IsScalar())
    {
        throw std::runtime_error(
            "missing scalar 'contact_state' in " + path.string());
    }

    const std::string channel = contact_channel.as<std::string>();
    if (channel.empty())
    {
        throw std::runtime_error(
            "'contact_state' must not be empty in " + path.string());
    }
    return channel;
}

class SerialPort
{
public:
    SerialPort() = default;

    ~SerialPort()
    {
        Close();
    }

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    void Open(const std::string& path)
    {
        Close();

        const int fd = ::open(
            path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0)
        {
            throw std::system_error(
                errno, std::generic_category(), "unable to open " + path);
        }

        try
        {
            Configure(fd, path);
        }
        catch (...)
        {
            ::close(fd);
            throw;
        }

        fd_ = fd;
    }

    void Close() noexcept
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
            fd_ = -1;
        }
    }

    [[nodiscard]] bool IsOpen() const noexcept
    {
        return fd_ >= 0;
    }

    [[nodiscard]] int Get() const noexcept
    {
        return fd_;
    }

private:
    static void Configure(int fd, const std::string& path)
    {
        termios2 settings{};
        if (::ioctl(fd, TCGETS2, &settings) < 0)
        {
            throw std::system_error(
                errno,
                std::generic_category(),
                "TCGETS2 failed for " + path);
        }

        // Raw 8N1, no software or hardware flow control. BOTHER is required
        // because glibc does not expose a standard B250000 baud constant.
        settings.c_iflag = 0;
        settings.c_oflag = 0;
        settings.c_lflag = 0;
        settings.c_cflag = BOTHER | CS8 | CREAD | CLOCAL;
        // O_NONBLOCK prevents this from blocking. VMIN=1 ensures that a
        // fully drained TTY reports EAGAIN instead of a zero-length read.
        settings.c_cc[VMIN] = 1;
        settings.c_cc[VTIME] = 0;
        settings.c_ispeed = kBaudRate;
        settings.c_ospeed = kBaudRate;

        if (::ioctl(fd, TCSETS2, &settings) < 0)
        {
            throw std::system_error(
                errno,
                std::generic_category(),
                "TCSETS2 failed for " + path);
        }
    }

    int fd_ = -1;
};

mors_msgs::contact_sensor_msg MakeMessage(
    const contact_sensor::ScaledRawData& raw_data,
    const contact_sensor::ContactStates& contact_states)
{
    mors_msgs::contact_sensor_msg message;
    for (std::size_t index = 0; index < contact_sensor::kLegCount; ++index)
    {
        message.raw_data[index] = raw_data[index];
        message.contact_states[index] = contact_states[index] ? 1 : 0;
    }
    return message;
}

mors_msgs::contact_sensor_msg MakeFallbackMessage()
{
    return MakeMessage(
        contact_sensor::ScaledRawData{}, contact_sensor::ContactStates{});
}

void Publish(
    lcm::LCM& publisher,
    const std::string& channel,
    const mors_msgs::contact_sensor_msg& message)
{
    if (publisher.publish(channel, &message) < 0)
    {
        throw std::runtime_error("failed to publish LCM message on " + channel);
    }
}

int PollTimeout(
    Clock::time_point now,
    Clock::time_point fallback_deadline,
    bool port_is_open,
    Clock::time_point reconnect_deadline)
{
    Clock::time_point deadline = fallback_deadline;
    if (!port_is_open)
    {
        deadline = std::min(deadline, reconnect_deadline);
    }

    if (deadline <= now)
    {
        return 0;
    }

    const auto timeout =
        std::chrono::ceil<std::chrono::milliseconds>(deadline - now);
    return static_cast<int>(std::min<std::int64_t>(
        timeout.count(), std::numeric_limits<int>::max()));
}

void LogSerialError(const std::exception& error)
{
    std::cerr << "[ContactSensor]: " << error.what()
              << "; retrying in " << kReconnectPeriod.count() << " s\n";
}

void InstallSignalHandlers()
{
    struct sigaction action
    {
    };
    action.sa_handler = HandleSignal;
    ::sigemptyset(&action.sa_mask);
    action.sa_flags = 0;

    if (::sigaction(SIGINT, &action, nullptr) < 0 ||
        ::sigaction(SIGTERM, &action, nullptr) < 0)
    {
        const int error_number = errno;
        throw std::system_error(
            error_number,
            std::generic_category(),
            "unable to install signal handlers");
    }
}

int Run()
{
    const std::filesystem::path config_directory = GetRequiredEnv("CONFIGPATH");
    const std::string lcm_url = GetRequiredEnv("LCM_CONTROL_URL");
    const std::string contact_channel =
        LoadContactChannel(config_directory / "channels.yaml");
    const contact_sensor::Config config =
        contact_sensor::LoadConfig(config_directory / "contact_sensor.yaml");

    lcm::LCM publisher(lcm_url);
    if (!publisher.good())
    {
        throw std::runtime_error("unable to initialize LCM at " + lcm_url);
    }

    InstallSignalHandlers();

    std::cout << "[ContactSensor]: control LCM URL: " << lcm_url << '\n'
              << "[ContactSensor]: channel: " << contact_channel << '\n'
              << "[ContactSensor]: serial port: " << config.serial_port << '\n'
              << "[ContactSensor]: raw data divisor: "
              << config.raw_data_divisor << '\n'
              << "[ContactSensor]: calibration samples: "
              << config.calibration_samples << '\n'
              << "[ContactSensor]: starting\n";

    SerialPort serial_port;
    contact_sensor::FrameAccumulator frame_accumulator;
    contact_sensor::ZeroOffsetCalibrator calibrator(
        config.calibration_samples);
    contact_sensor::FallbackScheduler fallback_scheduler(Clock::now());
    Clock::time_point next_reconnect = Clock::now();
    const mors_msgs::contact_sensor_msg fallback_message = MakeFallbackMessage();

    while (stop_requested == 0)
    {
        auto now = Clock::now();
        if (!serial_port.IsOpen() && now >= next_reconnect)
        {
            try
            {
                serial_port.Open(config.serial_port);
                frame_accumulator.Reset();
                std::cout << "[ContactSensor]: serial port opened\n";
            }
            catch (const std::exception& error)
            {
                LogSerialError(error);
                fallback_scheduler.OnIoError(now);
                next_reconnect = now + kReconnectPeriod;
            }
        }

        const int timeout_ms = PollTimeout(
            now,
            fallback_scheduler.NextDeadline(),
            serial_port.IsOpen(),
            next_reconnect);

        pollfd descriptor{};
        descriptor.fd = serial_port.Get();
        descriptor.events = POLLIN;

        const int poll_result = ::poll(
            serial_port.IsOpen() ? &descriptor : nullptr,
            serial_port.IsOpen() ? 1 : 0,
            timeout_ms);

        bool io_error = false;
        if (poll_result < 0)
        {
            if (errno != EINTR)
            {
                const int error_number = errno;
                std::cerr << "[ContactSensor]: poll failed: "
                          << std::strerror(error_number) << '\n';
                io_error = true;
            }
        }
        else if (poll_result > 0)
        {
            if ((descriptor.revents & POLLIN) != 0)
            {
                std::array<std::uint8_t, kReadBufferSize> read_buffer{};
                while (stop_requested == 0)
                {
                    const ssize_t bytes_read = ::read(
                        serial_port.Get(), read_buffer.data(), read_buffer.size());

                    if (bytes_read > 0)
                    {
                        const auto frames = frame_accumulator.Append(std::span(
                            read_buffer.data(),
                            static_cast<std::size_t>(bytes_read)));

                        for (const contact_sensor::RawData& serial_data : frames)
                        {
                            const contact_sensor::ScaledRawData raw_data =
                                contact_sensor::ScaleRawData(
                                    serial_data, config.raw_data_divisor);
                            fallback_scheduler.OnFrame(Clock::now());
                            const bool was_calibrated =
                                calibrator.IsCalibrated();
                            const std::optional<contact_sensor::ScaledRawData>
                                corrected_data = calibrator.Process(raw_data);

                            if (!was_calibrated && calibrator.IsCalibrated())
                            {
                                const contact_sensor::Offsets& offset =
                                    calibrator.offset();
                                std::cout
                                    << "[ContactSensor]: calibration complete; "
                                    << "offsets: " << offset[0] << ' '
                                    << offset[1] << ' ' << offset[2] << ' '
                                    << offset[3] << '\n';
                            }

                            if (!corrected_data)
                            {
                                continue;
                            }

                            const contact_sensor::ScaledRawData absolute_data =
                                contact_sensor::AbsoluteRawData(
                                    *corrected_data);
                            const contact_sensor::ContactStates contacts =
                                contact_sensor::DetectContacts(
                                    absolute_data, config.thresholds);
                            Publish(
                                publisher,
                                contact_channel,
                                MakeMessage(absolute_data, contacts));
                        }
                        continue;
                    }

                    if (bytes_read == 0)
                    {
                        std::cerr
                            << "[ContactSensor]: serial connection closed\n";
                        io_error = true;
                        break;
                    }

                    if (errno == EINTR)
                    {
                        continue;
                    }
                    if (errno == EAGAIN || errno == EWOULDBLOCK)
                    {
                        break;
                    }

                    const int error_number = errno;
                    std::cerr << "[ContactSensor]: read failed: "
                              << std::strerror(error_number) << '\n';
                    io_error = true;
                    break;
                }
            }

            if (!io_error &&
                (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
            {
                std::cerr << "[ContactSensor]: serial connection lost\n";
                io_error = true;
            }
        }

        now = Clock::now();
        if (io_error)
        {
            serial_port.Close();
            frame_accumulator.Reset();
            if (!calibrator.IsCalibrated())
            {
                calibrator.Reset();
            }
            fallback_scheduler.OnIoError(now);
            next_reconnect = now + kReconnectPeriod;
        }

        if (stop_requested != 0)
        {
            break;
        }

        if (fallback_scheduler.IsDue(now))
        {
            Publish(publisher, contact_channel, fallback_message);
            fallback_scheduler.MarkPublished(now);
        }
    }

    std::cout << "[ContactSensor]: stopped\n";
    return EXIT_SUCCESS;
}

} // namespace

int main()
{
    try
    {
        return Run();
    }
    catch (const std::exception& error)
    {
        std::cerr << "[ContactSensor]: fatal error: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
