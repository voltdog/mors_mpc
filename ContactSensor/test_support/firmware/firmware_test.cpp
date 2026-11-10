#include "VBCoreG4_arduino_system.h"
#include "stm32g4xx_ll_exti.h"
#include "contact_sensor/core.hpp"
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string>

namespace fake {
struct Adc {
    bool present = true, conversions = true, stuckLow = false;
    bool ready = false, enabled = false, pending = false, exti = false;
    bool closeError = false, corruptConfig = false;
    int mode = INPUT, output = HIGH, sck = LOW;
    uint8_t config = 0, command = 0, writtenConfig = 0, status = 0;
    uint8_t injectedStatus = 0;
    int clocks = 0;
    int32_t raw = 110 * 256;
    uint64_t nextReady = 300000, highAt = 0;
    void (*callback)() = nullptr;
};
std::array<Adc, 4> adcs;
uint64_t now = 0;
uint32_t primask = 0;
constexpr std::array<uint32_t, 4> dout{PC9, PC6, PB14, PB10};
constexpr std::array<uint32_t, 4> sck{PA8, PC7, PB13, PB11};
constexpr std::array<uint32_t, 4> lines{LL_EXTI_LINE_9, LL_EXTI_LINE_6, LL_EXTI_LINE_14, LL_EXTI_LINE_10};
int index(uint32_t pin, const std::array<uint32_t, 4>& pins) {
    const auto it = std::find(pins.begin(), pins.end(), pin);
    if (it == pins.end()) throw std::runtime_error("unknown pin");
    return static_cast<int>(it - pins.begin());
}
uint64_t period(const Adc& adc) { return (adc.config & 0x30) == 0x20 ? 1563 : 100000; }
void dispatch() {
    if (primask) return;
    for (auto& adc : adcs) if (adc.pending && adc.enabled && adc.exti && adc.callback) {
        adc.pending = false;
        adc.callback();
    }
}
void advance(uint64_t us) {
    now += us;
    for (auto& adc : adcs) {
        if (adc.present && adc.conversions && !adc.ready && adc.sck == LOW &&
            adc.nextReady <= now) {
            adc.ready = true;
            adc.clocks = 0;
            adc.command = adc.writtenConfig = 0;
            adc.status = adc.injectedStatus;
            adc.pending = true;
        }
    }
    dispatch();
}
}
FakeSerial Serial;
uint32_t micros() { return static_cast<uint32_t>(fake::now); }
uint32_t millis() { return static_cast<uint32_t>(fake::now / 1000); }
uint32_t dwt_getCycles() { return static_cast<uint32_t>(fake::now * 170); }
void delayMicroseconds(uint32_t us) { fake::advance(us); }
void pinMode(uint32_t pin, int mode) {
    if (std::find(fake::dout.begin(), fake::dout.end(), pin) == fake::dout.end()) return;
    auto& adc = fake::adcs[fake::index(pin, fake::dout)];
    adc.mode = mode;
    adc.exti = false;
}
int digitalRead(uint32_t pin) {
    auto& adc = fake::adcs[fake::index(pin, fake::dout)];
    if (adc.mode == OUTPUT) return adc.output;
    if (adc.stuckLow) return LOW;
    if (!adc.present) return HIGH;
    if (adc.clocks >= 1 && adc.clocks <= 24 && adc.ready)
        return (static_cast<uint32_t>(adc.raw) >> (24 - adc.clocks)) & 1;
    if (adc.clocks == 25 || adc.clocks == 26)
        return (adc.status >> (26 - adc.clocks)) & 1;
    if (adc.clocks == 27 && adc.closeError) return LOW;
    if (adc.command == 0x56 && adc.clocks >= 38 && adc.clocks <= 45)
        return ((adc.config ^ (adc.corruptConfig ? 0x10 : 0)) >> (45 - adc.clocks)) & 1;
    return adc.ready && adc.clocks == 0 ? LOW : HIGH;
}
void digitalWrite(uint32_t pin, int level) {
    const auto d = std::find(fake::dout.begin(), fake::dout.end(), pin);
    if (d != fake::dout.end()) {
        fake::adcs[d - fake::dout.begin()].output = level;
        return;
    }
    auto& adc = fake::adcs[fake::index(pin, fake::sck)];
    if (adc.sck == level) return;
    adc.sck = level;
    if (level == HIGH) {
        adc.highAt = fake::now;
        if (adc.ready || adc.clocks >= 27) ++adc.clocks;
        if (adc.mode == OUTPUT && adc.clocks >= 30 && adc.clocks <= 36)
            adc.command = static_cast<uint8_t>((adc.command << 1) | adc.output);
        if (adc.mode == OUTPUT && adc.clocks >= 38 && adc.clocks <= 45)
            adc.writtenConfig = static_cast<uint8_t>((adc.writtenConfig << 1) | adc.output);
    } else if (fake::now - adc.highAt >= 100) {
        adc.ready = false;
        adc.clocks = 0;
        adc.nextReady = fake::now + ((adc.config & 0x30) == 0x20 ? 6250 : 300000);
    } else if (adc.clocks == 27 || adc.clocks == 46) {
        adc.ready = false;
        if (adc.clocks == 46 && adc.command == 0x65) adc.config = adc.writtenConfig;
        adc.nextReady = fake::now + fake::period(adc);
    }
}
void attachInterrupt(uint32_t pin, void (*callback)(), int) {
    auto& adc = fake::adcs[fake::index(pin, fake::dout)];
    adc.callback = callback; adc.exti = true; adc.enabled = true;
    if (adc.mode != INPUT) throw std::runtime_error("input mode lost");
}
uint32_t __get_PRIMASK() { return fake::primask; }
void __disable_irq() { fake::primask = 1; }
void __set_PRIMASK(uint32_t mask) { fake::primask = mask; fake::dispatch(); }
void LL_EXTI_DisableIT_0_31(uint32_t line) { fake::adcs[fake::index(line, fake::lines)].enabled = false; }
void LL_EXTI_EnableIT_0_31(uint32_t line) { fake::adcs[fake::index(line, fake::lines)].enabled = true; }
void LL_EXTI_ClearFlag_0_31(uint32_t line) { fake::adcs[fake::index(line, fake::lines)].pending = false; }
void FakeSerial::begin(int baud) { if (baud != 250000) throw std::runtime_error("baud"); }
size_t FakeSerial::write(const uint8_t* data, size_t size) {
    if (size != 16 || capacity < static_cast<int>(size)) throw std::runtime_error("partial/blocking write");
    bytes.insert(bytes.end(), data, data + size); times.push_back(fake::now); return size;
}

// Compile the actual sketch, with hardware replaced at its API boundary.
#include "../../../firmwares/contact_sensor_v4/contact_sensor_v4.ino"

static void check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
static void reset(unsigned present = 15, uint64_t at = 0) {
    fake::now = at; fake::primask = 0; fake::adcs = {};
    for (unsigned i = 0; i < 4; ++i) {
        fake::adcs[i].present = (present & (1U << i)) != 0;
        fake::adcs[i].nextReady = at + 300000;
        cs1237Channels[i] = {};
        cs1237ReadyStates[i] = {};
        cs1237MedianFilters[i] = {};
        cs1237Diagnostics[i] = {};
        cs1237FilteredValues[i] = 0;
        cs1237FilteredAtCycles[i] = 0;
    }
    Serial = {};
    contactFramesFormed = contactFramesSent = contactFramesSkipped = 0;
    recoveryCursor = 0; frameClockStarted = false;
    frameClockUs = 0; frameClockRemainder = 0;
    setup();
}
static void run(uint64_t us) {
    const auto end = fake::now + us;
    while (fake::now < end) { loop(); fake::advance(10); }
}
static const uint8_t* last_frame_bytes() {
    check(Serial.bytes.size() >= FRAME_SIZE, "missing frame");
    const uint8_t* frame = Serial.bytes.data() + Serial.bytes.size() - FRAME_SIZE;
    check(std::equal(std::begin(FRAME_HEADER), std::end(FRAME_HEADER), frame),
          "invalid frame header");
    check(frame[FRAME_TERMINATOR_OFFSET] == '\r' &&
              frame[FRAME_TERMINATOR_OFFSET + 1] == '\n',
          "invalid frame terminator");
    const uint16_t encodedCrc =
        (static_cast<uint16_t>(frame[FRAME_CRC_OFFSET]) << 8U) |
        static_cast<uint16_t>(frame[FRAME_CRC_OFFSET + 1]);
    check(encodedCrc == contact_sensor::ComputeCrc16CcittFalse(
                            std::span<const uint8_t>{frame, FRAME_CRC_OFFSET}),
          "invalid frame CRC");
    check((frame[FRAME_READINESS_OFFSET] & 0xF0U) == 0U,
          "reserved readiness bits set");
    return frame;
}
static contact_sensor::RawData last_frame() {
    const uint8_t* frame = last_frame_bytes();
    contact_sensor::RawData values{};
    for (size_t i = 0; i < values.size(); ++i) {
        const size_t offset = FRAME_PAYLOAD_OFFSET + i * sizeof(int16_t);
        const uint16_t encoded = static_cast<uint16_t>(frame[offset]) |
            (static_cast<uint16_t>(frame[offset + 1]) << 8U);
        values[i] = (encoded & 0x8000U) != 0U
            ? static_cast<int32_t>(encoded) - 0x10000L
            : static_cast<int32_t>(encoded);
    }
    return values;
}
static uint8_t last_readiness_mask() {
    return last_frame_bytes()[FRAME_READINESS_OFFSET];
}
static void timing(size_t from) {
    const auto count = Serial.times.size() - from;
    check(count >= 570 && count <= 630, "frame rate outside 600 +/- 5%: " + std::to_string(count));
    for (size_t i = from + 1; i < Serial.times.size(); ++i)
        check(Serial.times[i] - Serial.times[i - 1] <= 5000, "frame gap > 5 ms");
}
int main() {
    try {
        for (unsigned mask = 0; mask < 16; ++mask) {
            reset(mask);
            const auto started = fake::now;
            loop();
            check(Serial.times.front() - started <= 10000, "startup frame late");
            check(last_frame() == contact_sensor::RawData{-1, -1, -1, -1}, "startup validity");
            check(last_readiness_mask() == 0, "startup readiness");
            run(700000);
            const auto from = Serial.times.size();
            uint32_t outputs[4];
            for (int i = 0; i < 4; ++i) outputs[i] = cs1237Diagnostics[i].medianOutputs;
            run(1000000); timing(from);
            const auto values = last_frame();
            check(last_readiness_mask() == mask, "readiness mask " + std::to_string(mask));
            for (unsigned i = 0; i < 4; ++i) {
                const bool present = (mask & (1U << i)) != 0;
                check(values[i] == (present ? 110 : -1), "availability mask " + std::to_string(mask));
                if (present) check(cs1237Diagnostics[i].medianOutputs - outputs[i] >= 570, "new median rate");
                else check(cs1237Diagnostics[i].recoveryAttempts >= 2, "retry starvation");
            }
        }
        std::cout << "[PASS] all 16 startup combinations, frame and median rates\n";

        reset();
        fake::adcs[0].injectedStatus = 1;
        const uint64_t configDeadline = fake::now + 700000;
        while (cs1237Channels[0].phase != ChannelPhase::Settling &&
               fake::now < configDeadline) {
            loop();
            fake::advance(10);
        }
        check(cs1237Channels[0].phase == ChannelPhase::Settling,
              "reserved status blocked configuration");
        check(cs1237Diagnostics[0].reservedStatusErrors >= 2,
              "configuration reserved status not recorded");
        fake::adcs[0].injectedStatus = 0;
        run(700000);
        check(last_readiness_mask() == 15U,
              "channel did not recover after configuration status anomaly");
        std::cout << "[PASS] configuration tolerates reserved status anomaly\n";

        reset(); run(700000);
        for (int lost = 0; lost < 4; ++lost) {
            fake::adcs[lost].present = false;
            run(10000);
            check(last_frame()[lost] == -1, "stale measurement not invalidated at 7 ms");
            check(last_readiness_mask() == (15U & ~(1U << lost)),
                  "lost channel remained ready");
            run(30000);
            check(cs1237Channels[lost].phase == ChannelPhase::Retry, "runtime timeout");
            fake::adcs[lost].present = true;
            run(1900000);
            check(last_frame()[lost] == 110, "cable recovery > 2 s");
            check(last_readiness_mask() == 15U, "recovered channel not ready");
        }
        std::cout << "[PASS] individual cable loss, expiry and recovery\n";
        for (int i = 0; i < 3; ++i) fake::adcs[i].present = false;
        run(1300000);
        check(last_frame() == contact_sensor::RawData{-1, -1, -1, 110}, "three-channel failure");
        check(last_readiness_mask() == 8U, "three-channel readiness");
        for (int i = 0; i < 3; ++i) fake::adcs[i].present = true;
        run(1900000);
        check(last_frame() == contact_sensor::RawData{110, 110, 110, 110}, "multiple recovery");
        check(last_readiness_mask() == 15U, "multiple recovery readiness");
        Serial.capacity = 15;
        const auto sent = Serial.bytes.size(); run(20000);
        check(Serial.bytes.size() == sent && contactFramesSkipped > 0, "full buffer partial write");
        Serial.capacity = 64;
        const auto formed = contactFramesFormed;
        fake::advance(100000); loop();
        check(contactFramesFormed == formed + 1, "catch-up burst");
        std::cout << "[PASS] simultaneous recovery, full UART buffer, delayed schedule\n";
        for (uint64_t at : {uint64_t{0xFFFFFFFFU} - 5000,
                            (uint64_t{1} << 32) * 1000 - 5000,
                            (uint64_t{1} << 32) / 170 - 5000}) {
            reset(15, at); run(1800000);
            check(last_frame() == contact_sensor::RawData{110, 110, 110, 110}, "clock wrap recovery");
        }
        reset(); run(700000);
        fake::adcs[0].present = false;
        run(30000);
        fake::advance((uint64_t{1} << 32) / 170);
        loop(); check(!cs1237Channels[0].filteredValid, "DWT resurrected failed channel");
        run(10000);
        check(last_readiness_mask() == 0x0EU, "healthy channels after DWT wrap");
        fake::adcs[1].injectedStatus = 1;
        run(3000);
        check(last_frame()[1] == -1, "reserved status accepted");
        check(last_readiness_mask() == 0x0CU,
              "reserved status readiness: " + std::to_string(last_readiness_mask()));
        fake::adcs[1].injectedStatus = 0;
        run(1900000); check(last_frame()[1] == 110, "status recovery");
        fake::adcs[2].injectedStatus = 2;
        run(3000); check(last_frame()[2] == -1, "unexpected config update accepted");
        check(last_readiness_mask() == 0x0AU,
              "config update readiness: " + std::to_string(last_readiness_mask()));
        std::cout << "[PASS] micros/millis/DWT wrap and transaction status faults\n";
        reset(); run(700000);
        const auto attempts = cs1237Diagnostics[0].recoveryAttempts;
        // Consume an old but correctly closed transaction without restarting ADC.
        fake::adcs[0].ready = true; fake::adcs[0].clocks = 0;
        cs1237ReadyStates[0].phase = Cs1237ReadyPhase::Pending;
        cs1237ReadyStates[0].edgeCycles = dwt_getCycles() - 600 * 170;
        cs1237ReadyStates[0].edgeMs = millis();
        service_measurements();
        check(cs1237Diagnostics[0].staleReady > 0, "stale DRDY accepted");
        check(cs1237Channels[0].phase == ChannelPhase::Running &&
              cs1237Diagnostics[0].recoveryAttempts == attempts, "stale DRDY restarted ADC");
        fake::adcs[0].ready = false;
        cs1237ReadyStates[0].phase = Cs1237ReadyPhase::Pending;
        cs1237ReadyStates[0].edgeCycles = dwt_getCycles();
        cs1237ReadyStates[0].edgeMs = millis();
        service_measurements();
        check(cs1237Diagnostics[0].readyLost > 0, "lost DRDY not recorded");
        fake::adcs[0].closeError = true;
        run(3000); check(last_frame()[0] == -1, "unclosed transaction accepted");
        check(last_readiness_mask() == 0x0EU, "unclosed transaction readiness");
        check(cs1237Diagnostics[0].frameCloseErrors > 0, "missing close-error counter");
        reset(); fake::adcs[0].corruptConfig = true; run(700000);
        check(!cs1237Channels[0].filteredValid &&
              cs1237Diagnostics[0].lastError == static_cast<uint8_t>(ChannelError::ConfigMismatch),
              "unverified configuration accepted");
        check(last_frame()[1] == 110, "config failure blocked healthy channel");
        check(last_readiness_mask() == 0x0EU, "config failure readiness");
        std::cout << "[PASS] stale/lost DRDY, frame closure and config readback faults\n";

        reset(0);
        for (uint32_t pause : {100U, 200U, 400U, 800U, 1000U, 1000U}) {
            const auto now = millis();
            fail_channel(0, ChannelError::ReadyTimeout);
            check(cs1237Channels[0].retryAtMs - now == pause, "retry backoff");
        }
        // Exact rational deadlines: 1667, 3334, 5000 us, without accumulated drift.
        service_transmit();
        fake::advance(1666); service_transmit(); check(contactFramesSent == 1, "early deadline");
        fake::advance(1); service_transmit(); check(contactFramesSent == 2, "first deadline");
        fake::advance(1666); service_transmit(); check(contactFramesSent == 2, "early second deadline");
        fake::advance(1); service_transmit(); check(contactFramesSent == 3, "second deadline");
        fake::advance(1666); service_transmit(); check(contactFramesSent == 4, "fraction drift");
        // Median preserves full signed 24-bit values and the selected sample's time.
        median_filter_reset(0);
        int32_t value = 0; uint32_t atCycles = 0;
        check(!median_filter_push(0, -65537, 0, value, atCycles), "median warmup 1");
        check(!median_filter_push(0, 8388607, 1500 * 170, value, atCycles), "median warmup 2");
        check(median_filter_push(0, -8388608, 3000 * 170, value, atCycles), "median warmup 3");
        check(value == -65537 && atCycles == 0, "median value/timestamp");
        check(!median_filter_push(0, 200, 8000 * 170, value, atCycles), "expired median window");
        cs1237Channels[0].filteredValid = true;
        cs1237Channels[0].lastInputMs = millis();
        cs1237FilteredAtCycles[0] = dwt_getCycles() - 7001 * 170;
        send_contact_frame(); check(last_frame()[0] == -1, "7001 us measurement accepted");
        check(last_readiness_mask() == 0U, "expired measurement readiness");
        fake::advance((uint64_t{1} << 32) / 170);
        send_contact_frame(); check(last_frame()[0] == -1, "expired value resurrected");
        std::cout << "[PASS] retry intervals, fractional schedule, signed median/time and age latch\n";
        check(encode_measurement(-1, true) == 0xFFFE, "valid -1 code collision");
        check(encode_measurement(-256, true) == 0xFFFE, "valid -256 code collision");
        check(encode_measurement(-257, true) == 0xFFFE, "negative shift changed");
        check(encode_measurement(0, false) == 0xFFFF, "invalid serialization");
        reset(0);
        fake::adcs[0].stuckLow = true;
        run(1500000);
        for (const auto& d : cs1237Diagnostics) check(d.recoveryAttempts >= 2, "stuck LOW starvation");
        check(last_frame() == contact_sensor::RawData{-1, -1, -1, -1}, "stuck LOW validity");
        check(last_readiness_mask() == 0U, "stuck LOW readiness");
        std::cout << "[PASS] reserved wire code and stuck LOW fairness\n";
    } catch (const std::exception& error) {
        std::cerr << "[FAIL] " << error.what() << '\n'; return 1;
    }
}
