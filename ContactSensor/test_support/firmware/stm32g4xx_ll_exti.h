#pragma once
#include <cstdint>
constexpr uint32_t LL_EXTI_LINE_9 = 1U << 9;
constexpr uint32_t LL_EXTI_LINE_6 = 1U << 6;
constexpr uint32_t LL_EXTI_LINE_14 = 1U << 14;
constexpr uint32_t LL_EXTI_LINE_10 = 1U << 10;
void LL_EXTI_DisableIT_0_31(uint32_t line);
void LL_EXTI_EnableIT_0_31(uint32_t line);
void LL_EXTI_ClearFlag_0_31(uint32_t line);
