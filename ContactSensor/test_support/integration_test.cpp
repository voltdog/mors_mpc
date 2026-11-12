#include "contact_sensor/core.hpp"
#include "mors_msgs/contact_sensor_msg.hpp"
#include <lcm/lcm-cpp.hpp>
#include <pty.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using Values = std::array<float, 4>;
using Readiness = contact_sensor::ChannelReadiness;
constexpr Readiness kAllReady{true, true, true, true};
static void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

struct Receiver {
    std::vector<mors_msgs::contact_sensor_msg> messages;
    void OnMessage(const lcm::ReceiveBuffer*, const std::string&,
                   const mors_msgs::contact_sensor_msg* message) {
        messages.push_back(*message);
    }
};

class Driver {
public:
    explicit Driver(const char* executable) {
        char directory[] = "/tmp/contact_sensor_integration_XXXXXX";
        const char* created = ::mkdtemp(directory);
        check(created != nullptr, "mkdtemp");
        path = created;
        MakePty();
        std::ofstream(path / "channels.yaml") << "contact_state: CONTACT_SENSOR_TEST\n";
        std::ofstream(path / "sensors.yaml")
            << "contact_sensor:\n  serial_port: " << (path / "serial").string()
            << "\n  raw_data_divisor: 55\n  calibration_samples: 2\n"
            << "  thresholds: {r1: 1, l1: 1, r2: 1, l2: 1}\n";
        url = "udpm://239.255.76.67:" + std::to_string(40000 + ::getpid() % 10000) + "?ttl=0";
        pid = ::fork();
        check(pid >= 0, "fork");
        if (pid == 0) {
            const int log = ::open((path / "driver.log").c_str(), O_CREAT | O_WRONLY, 0600);
            ::dup2(log, STDOUT_FILENO); ::dup2(log, STDERR_FILENO); ::close(log);
            ::close(master); ::close(slave);
            ::setenv("CONFIGPATH", path.c_str(), 1);
            ::setenv("LCM_LOCOMOTION_URL", url.c_str(), 1);
            ::execl(executable, executable, static_cast<char*>(nullptr));
            ::_exit(127);
        }
    }
    ~Driver() {
        if (pid > 0) { ::kill(pid, SIGTERM); ::waitpid(pid, nullptr, 0); }
        if (master >= 0) ::close(master);
        if (slave >= 0) ::close(slave);
        std::error_code ignored; std::filesystem::remove_all(path, ignored);
    }
    void MakePty() {
        char name[128]{};
        check(::openpty(&master, &slave, name, nullptr, nullptr) == 0, "openpty");
        tty = name;
        std::error_code ignored;
        std::filesystem::remove(path / "serial", ignored);
        std::filesystem::create_symlink(tty, path / "serial");
    }
    void WaitOpen() {
        const auto deadline = Clock::now() + 3s;
        while (Clock::now() < deadline) {
            for (const auto& entry : std::filesystem::directory_iterator(
                     "/proc/" + std::to_string(pid) + "/fd")) {
                std::error_code ignored;
                if (std::filesystem::read_symlink(entry.path(), ignored) == tty) {
                    std::this_thread::sleep_for(10ms);
                    ::close(slave); slave = -1;
                    return;
                }
            }
            std::this_thread::sleep_for(5ms);
        }
        throw std::runtime_error("driver did not open PTY; log: " + Log());
    }
    std::string Log() const {
        std::ifstream input(path / "driver.log");
        return {std::istreambuf_iterator<char>(input), {}};
    }
    void Disconnect() { ::close(master); master = -1; }
    void Send(const contact_sensor::RawData& raw,
              const Readiness& readiness = kAllReady,
              bool corrupt = false, bool split = false,
              size_t bytes_to_send = contact_sensor::kFrameSize) {
        std::array<uint8_t, contact_sensor::kFrameSize> frame{};
        std::copy(contact_sensor::kFrameHeader.begin(),
                  contact_sensor::kFrameHeader.end(), frame.begin());
        for (size_t i = 0; i < 4; ++i) {
            const uint16_t value = static_cast<uint16_t>(raw[i]);
            const size_t offset = contact_sensor::kPayloadOffset + 2 * i;
            frame[offset] = static_cast<uint8_t>(value);
            frame[offset + 1] = static_cast<uint8_t>(value >> 8);
            if (readiness[i]) {
                frame[contact_sensor::kReadinessOffset] |=
                    static_cast<uint8_t>(1U << i);
            }
        }
        const auto crc = contact_sensor::ComputeCrc16CcittFalse(
            std::span(frame).first(contact_sensor::kCrcOffset));
        frame[contact_sensor::kCrcOffset] = static_cast<uint8_t>(crc >> 8);
        frame[contact_sensor::kCrcOffset + 1] = static_cast<uint8_t>(crc);
        std::copy(contact_sensor::kFrameTerminator.begin(),
                  contact_sensor::kFrameTerminator.end(),
                  frame.begin() + contact_sensor::kTerminatorOffset);
        if (corrupt) frame[5] ^= 0x10;
        if (split) {
            check(::write(master, frame.data(), 5) == 5, "split write");
            std::this_thread::sleep_for(1ms);
            const size_t suffix_size = frame.size() - 5;
            check(::write(master, frame.data() + 5, suffix_size) ==
                      static_cast<ssize_t>(suffix_size),
                  "split suffix");
        } else check(::write(master, frame.data(), bytes_to_send) ==
                         static_cast<ssize_t>(bytes_to_send), "frame write");
    }
    std::filesystem::path path;
    std::string url, tty;
    int master = -1, slave = -1;
    pid_t pid = -1;
};

static void pump(lcm::LCM& subscriber, std::chrono::milliseconds duration) {
    const auto end = Clock::now() + duration;
    while (Clock::now() < end) check(subscriber.handleTimeout(5) >= 0, "LCM receive");
}

struct ExpectedMessage {
    Values values;
    Readiness readiness;
};

static ExpectedMessage expected(Values values,
                                Readiness readiness = kAllReady) {
    return ExpectedMessage{values, readiness};
}

static void expect(lcm::LCM& subscriber, Receiver& receiver,
                   const std::vector<ExpectedMessage>& messages) {
    const auto end = Clock::now() + 80ms;
    while (receiver.messages.size() < messages.size() && Clock::now() < end)
        check(subscriber.handleTimeout(5) >= 0, "LCM receive");
    while (subscriber.handleTimeout(0) > 0) {}
    check(receiver.messages.size() == messages.size(),
          "frame/publication count mismatch");
    for (size_t frame = 0; frame < messages.size(); ++frame) {
        for (size_t i = 0; i < 4; ++i) {
            check(std::abs(receiver.messages[frame].raw_data[i] -
                           messages[frame].values[i]) < 1e-5F,
                  "LCM value/marker mismatch");
            const bool contact = messages[frame].readiness[i] &&
                                 messages[frame].values[i] >= 1;
            check(receiver.messages[frame].contact_states[i] == contact,
                  "LCM contact mismatch");
            check(receiver.messages[frame].readiness[i] ==
                      messages[frame].readiness[i],
                  "LCM readiness mismatch");
        }
    }
    receiver.messages.clear();
}
int main(int argc, char** argv) {
    check(argc == 2, "expected driver executable");
    try {
        Driver driver(argv[1]);
        lcm::LCM subscriber(driver.url);
        check(subscriber.good(), "LCM init");
        Receiver receiver;
        subscriber.subscribe("CONTACT_SENSOR_TEST", &Receiver::OnMessage, &receiver);
        driver.WaitOpen();
        const Values invalid{-1, -1, -1, -1};
        const Readiness none_ready{};
        pump(subscriber, 130ms);
        expect(subscriber, receiver, {expected(invalid, none_ready)});
        pump(subscriber, 100ms);
        expect(subscriber, receiver, {expected(invalid, none_ready)});

        // A break before calibration completes must discard the partial zero.
        driver.Send({110, 220, 330, 440});
        expect(subscriber, receiver, {expected(invalid, none_ready)});
        pump(subscriber, 130ms);
        expect(subscriber, receiver, {expected(invalid, none_ready)});

        const Readiness first_pair{true, true, false, false};
        driver.Send({55, 110, -1, -1}, first_pair, false, true);
        driver.Send({55, 110, -1, -1}, first_pair);
        expect(subscriber, receiver,
               {expected(invalid, none_ready),
                expected(invalid, none_ready)});
        driver.Send({110, 165, -1, -1}, first_pair);
        expect(subscriber, receiver,
               {expected({1, 1, -1, -1}, first_pair)});

        // Channel 0 fails after calibration. Channel 2 calibrates independently.
        const Readiness middle_pair{false, true, true, false};
        driver.Send({-1, 220, 330, -1}, middle_pair);
        driver.Send({-1, 220, 330, -1}, middle_pair);
        expect(subscriber, receiver,
               {expected({-1, 2, -1, -1},
                         Readiness{false, true, false, false}),
                expected({-1, 2, -1, -1},
                         Readiness{false, true, false, false})});

        const Readiness first_three{true, true, true, false};
        driver.Send({165, 220, 385, -1}, first_three);
        expect(subscriber, receiver,
               {expected({2, 2, 1, -1}, first_three)});

        // An interrupted initial calibration of channel 3 starts from scratch.
        driver.Send({165, 220, 385, 440});
        expect(subscriber, receiver,
               {expected({2, 2, 1, -1}, first_three)});
        driver.Send({165, 220, 385, -1}, first_three);
        expect(subscriber, receiver,
               {expected({2, 2, 1, -1}, first_three)});
        driver.Send({165, 220, 385, 55});
        driver.Send({165, 220, 385, 55});
        expect(subscriber, receiver,
               {expected({2, 2, 1, -1}, first_three),
                expected({2, 2, 1, -1}, first_three)});
        driver.Send({165, 220, 385, 110});
        expect(subscriber, receiver,
               {expected({2, 2, 1, 1})});

        // Completed offsets survive every combination of channel failures.
        for (unsigned mask = 0; mask < 16; ++mask) {
            contact_sensor::RawData raw{110, 165, 385, 110};
            Values output{1, 1, 1, 1};
            Readiness readiness{};
            for (unsigned i = 0; i < 4; ++i) {
                readiness[i] = (mask & (1U << i)) != 0U;
                if (!readiness[i]) {
                    raw[i] = -1;
                    output[i] = -1;
                }
            }
            driver.Send(raw, readiness);
            expect(subscriber, receiver, {expected(output, readiness)});
        }

        // One-second real-process stream at 600 frames/s, servicing the subscriber
        // throughout so its own finite subscription queue does not hide delivery.
        const auto streamStart = Clock::now();
        for (int frame = 1; frame <= 600; ++frame) {
            std::this_thread::sleep_until(streamStart + std::chrono::microseconds(frame * 1000000LL / 600));
            driver.Send({110, 165, 385, 110});
            while (subscriber.handleTimeout(0) > 0) {}
        }
        expect(subscriber, receiver,
               std::vector<ExpectedMessage>(
                   600, expected(Values{1, 1, 1, 1})));
        // Bad CRC must neither publish nor extend the last valid frame's freshness.
        for (int i = 0; i < 16; ++i) {
            driver.Send({110, 165, 385, 110}, kAllReady, true);
            pump(subscriber, 10ms);
        }
        expect(subscriber, receiver, {expected(invalid, none_ready)});
        pump(subscriber, 100ms);
        expect(subscriber, receiver, {expected(invalid, none_ready)});
        driver.Send({110, 165, 385, 110});
        expect(subscriber, receiver, {expected({1, 1, 1, 1})});

        // A header and partial payload must not keep the stream ready.
        driver.Send({110, 165, 385, 110}, kAllReady, false, false, 5);
        pump(subscriber, 130ms);
        expect(subscriber, receiver, {expected(invalid, none_ready)});
        driver.Send({110, 165, 385, 110});
        expect(subscriber, receiver, {expected({1, 1, 1, 1})});
        driver.Disconnect();
        pump(subscriber, 30ms);
        check(receiver.messages.size() == 1, "I/O loss did not immediately publish fallback");
        expect(subscriber, receiver, {expected(invalid, none_ready)});
        driver.MakePty(); driver.WaitOpen();
        pump(subscriber, 5ms); receiver.messages.clear();
        driver.Send({110, 165, 385, 110});
        expect(subscriber, receiver, {expected({1, 1, 1, 1})});
        ::kill(driver.pid, SIGTERM); ::waitpid(driver.pid, nullptr, 0); driver.pid = -1;
        const auto log = driver.Log();
        check(log.find("R1 calibration complete") != std::string::npos, "missing calibration diagnostic");
        check(log.find("R1 data unavailable") != std::string::npos, "missing channel diagnostic");
        std::cout << "[PASS] PTY + real LCM: #02 per-channel readiness, 600 frames/s, calibration, CRC, partial frames, timeout, reconnect with retained zeros\n";
    } catch (const std::exception& error) {
        std::cerr << "[FAIL] " << error.what() << '\n'; return 1;
    }
}
