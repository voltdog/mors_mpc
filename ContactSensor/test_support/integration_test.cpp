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
    void Send(const contact_sensor::RawData& raw, bool corrupt = false, bool split = false,
              size_t bytes_to_send = contact_sensor::kFrameSize) {
        std::array<uint8_t, 15> frame{'#', '0', '1'};
        for (size_t i = 0; i < 4; ++i) {
            const uint16_t value = static_cast<uint16_t>(raw[i]);
            frame[3 + 2 * i] = static_cast<uint8_t>(value);
            frame[4 + 2 * i] = static_cast<uint8_t>(value >> 8);
        }
        const auto crc = contact_sensor::ComputeCrc16CcittFalse(std::span(frame).first(11));
        frame[11] = static_cast<uint8_t>(crc >> 8); frame[12] = static_cast<uint8_t>(crc);
        frame[13] = '\r'; frame[14] = '\n';
        if (corrupt) frame[5] ^= 0x10;
        if (split) {
            check(::write(master, frame.data(), 5) == 5, "split write");
            std::this_thread::sleep_for(1ms);
            check(::write(master, frame.data() + 5, 10) == 10, "split suffix");
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
static void expect(lcm::LCM& subscriber, Receiver& receiver,
                   const std::vector<Values>& values, bool available = true) {
    const auto end = Clock::now() + 80ms;
    while (receiver.messages.size() < values.size() && Clock::now() < end)
        check(subscriber.handleTimeout(5) >= 0, "LCM receive");
    while (subscriber.handleTimeout(0) > 0) {}
    check(receiver.messages.size() == values.size(), "frame/publication count mismatch");
    for (size_t frame = 0; frame < values.size(); ++frame) {
        for (size_t i = 0; i < 4; ++i) {
            check(std::abs(receiver.messages[frame].raw_data[i] - values[frame][i]) < 1e-5F,
                  "LCM value/marker mismatch");
            check(receiver.messages[frame].contact_states[i] == (values[frame][i] >= 1), "LCM contact mismatch");
            check(receiver.messages[frame].readiness[i] == available, "LCM readiness mismatch");
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
        pump(subscriber, 130ms);
        expect(subscriber, receiver, {invalid}, false);
        pump(subscriber, 100ms);
        expect(subscriber, receiver, {invalid}, false);

        // A break before calibration completes must discard the partial zero.
        driver.Send({110, 220, 330, 440});
        expect(subscriber, receiver, {invalid});
        pump(subscriber, 130ms);
        expect(subscriber, receiver, {invalid}, false);

        driver.Send({-1, 0, 55, 110}, false, true);
        driver.Send({-1, 0, 55, 110});
        driver.Send({-1, 0, 55, 110});
        expect(subscriber, receiver, {{-1, -1, -1, -1}, {-1, -1, -1, -1},
                                      {0, 0, 0, 0}});
        driver.Send({0, 0, 0, 0});
        expect(subscriber, receiver, {{1.0F / 55, 0, 1, 2}});

        // In v3, -1 in any combination of channels is an ordinary measurement.
        const Values minus_one_output{0, 1.0F / 55, 56.0F / 55, 111.0F / 55};
        for (unsigned mask = 0; mask < 16; ++mask) {
            contact_sensor::RawData raw{54, 55, 110, 165};
            Values output{1, 1, 1, 1};
            for (unsigned i = 0; i < 4; ++i) {
                if (!(mask & (1U << i))) {
                    raw[i] = -1;
                    output[i] = minus_one_output[i];
                }
            }
            driver.Send(raw); expect(subscriber, receiver, {output});
        }
        // One-second real-process stream at 600 frames/s, servicing the subscriber
        // throughout so its own finite subscription queue does not hide delivery.
        const auto streamStart = Clock::now();
        for (int frame = 1; frame <= 600; ++frame) {
            std::this_thread::sleep_until(streamStart + std::chrono::microseconds(frame * 1000000LL / 600));
            driver.Send({54, 55, 110, 165});
            while (subscriber.handleTimeout(0) > 0) {}
        }
        expect(subscriber, receiver, std::vector<Values>(600, Values{1, 1, 1, 1}));
        // Bad CRC must neither publish nor extend the last valid frame's freshness.
        for (int i = 0; i < 16; ++i) {
            driver.Send({110, 55, 165, 220}, true); pump(subscriber, 10ms);
        }
        expect(subscriber, receiver, {invalid}, false);
        pump(subscriber, 100ms);
        expect(subscriber, receiver, {invalid}, false);
        driver.Send({54, 55, 110, 165});
        expect(subscriber, receiver, {{1, 1, 1, 1}});

        // A header and partial payload must not keep the stream ready.
        driver.Send({54, 55, 110, 165}, false, false, 5);
        pump(subscriber, 130ms);
        expect(subscriber, receiver, {invalid}, false);
        driver.Send({54, 55, 110, 165});
        expect(subscriber, receiver, {{1, 1, 1, 1}});
        driver.Disconnect();
        pump(subscriber, 30ms);
        check(receiver.messages.size() == 1, "I/O loss did not immediately publish fallback");
        expect(subscriber, receiver, {invalid}, false);
        driver.MakePty(); driver.WaitOpen();
        pump(subscriber, 5ms); receiver.messages.clear();
        driver.Send({54, 55, 110, 165});
        expect(subscriber, receiver, {{1, 1, 1, 1}});
        ::kill(driver.pid, SIGTERM); ::waitpid(driver.pid, nullptr, 0); driver.pid = -1;
        const auto log = driver.Log();
        check(log.find("R1 calibration complete") != std::string::npos, "missing calibration diagnostic");
        check(log.find("R1 data unavailable") != std::string::npos, "missing channel diagnostic");
        std::cout << "[PASS] PTY + real LCM: v3 readiness, -1 measurements, 600 frames/s, calibration, CRC, partial frames, timeout, reconnect with retained zeros\n";
    } catch (const std::exception& error) {
        std::cerr << "[FAIL] " << error.what() << '\n'; return 1;
    }
}
