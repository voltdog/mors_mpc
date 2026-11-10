#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>
#define PC9 9
#define PC6 6
#define PB14 114
#define PB10 110
#define PA8 208
#define PC7 7
#define PB13 113
#define PB11 111
constexpr int INPUT = 0, OUTPUT = 1, INPUT_PULLUP = 2;
constexpr int LOW = 0, HIGH = 1, FALLING = 2;
constexpr int EXTI9_5_IRQn = 0, EXTI15_10_IRQn = 1;
using IRQn_Type = int;
inline constexpr uint32_t SystemCoreClock = 170000000;
uint32_t micros();
uint32_t millis();
uint32_t dwt_getCycles();
void delayMicroseconds(uint32_t us);
void pinMode(uint32_t pin, int mode);
void digitalWrite(uint32_t pin, int level);
int digitalRead(uint32_t pin);
inline uint32_t digitalPinToInterrupt(uint32_t pin) { return pin; }
void attachInterrupt(uint32_t pin, void (*callback)(), int mode);
uint32_t __get_PRIMASK();
void __disable_irq();
void __set_PRIMASK(uint32_t mask);
inline void __DMB() {}
inline void __DSB() {}
inline void HAL_NVIC_SetPriority(int, int, int) {}
struct FakeSerial {
    int capacity = 64;
    std::vector<uint8_t> bytes;
    std::vector<uint64_t> times;
    void begin(int baud);
    int availableForWrite() const { return capacity; }
    size_t write(const uint8_t* data, size_t size);
};
extern FakeSerial Serial;
