#include <VBCoreG4_arduino_system.h>
#include <stm32g4xx_ll_exti.h>

#define PIN_R1_DOUT PC9
#define PIN_R1_SCK PA8

#define PIN_L1_DOUT PC6
#define PIN_L1_SCK PC7

#define PIN_R2_DOUT PB14
#define PIN_R2_SCK PB13

#define PIN_L2_DOUT PB10
#define PIN_L2_SCK PB11

struct Cs1237Pins {
  uint32_t dout;
  uint32_t sck;
  uint32_t extiLine;
};

constexpr uint8_t CS1237_COUNT = 4;
constexpr Cs1237Pins CS1237_SENSORS[CS1237_COUNT] = {
    {PIN_R1_DOUT, PIN_R1_SCK, LL_EXTI_LINE_9},
    {PIN_L1_DOUT, PIN_L1_SCK, LL_EXTI_LINE_6},
    {PIN_R2_DOUT, PIN_R2_SCK, LL_EXTI_LINE_14},
    {PIN_L2_DOUT, PIN_L2_SCK, LL_EXTI_LINE_10},
};

constexpr uint32_t CS1237_READY_STABLE_US = 2;
constexpr uint32_t CS1237_READY_CHECK_SLACK_US = 1;
constexpr uint32_t CS1237_READY_TIMEOUT_MS = 200;
// После PowerDown конфигурация может оставаться заводской (10 Гц). При этой
// скорости CS1237 держит DRDY в HIGH около 300 мс во время установления, поэтому
// для первого DRDY нужен отдельный timeout. Рабочий timeout на 640 Гц не меняем.
constexpr uint32_t CS1237_POWER_UP_READY_TIMEOUT_MS = 400;
// Время отсчитывается от обработанного EXTI-фронта. Даже с задержкой ISR на
// одну транзакцию 500 мкс оставляет большой запас до следующего обновления
// данных при периоде 1.5625 мс (640 Гц).
constexpr uint32_t CS1237_MAX_READY_AGE_US = 500;
// Медиана может выбрать самый старый из трех отсчетов. 7 мс покрывают два
// периода фильтра, взаимную фазу четырех АЦП и время их последовательного
// чтения, но все еще не допускают попадания в кадр действительно старых данных.
constexpr uint32_t CS1237_MAX_FRAME_SKEW_US = 7000;
constexpr uint32_t CS1237_MAX_FILTERED_AGE_US = 7000;
constexpr uint32_t CS1237_POWER_DOWN_US = 120;
constexpr uint32_t CS1237_POWER_UP_LOW_US = 12;
constexpr uint32_t MEDIAN_MAX_WINDOW_SPAN_US = 4000;
constexpr uint32_t CS1237_RECOVERY_RETRY_MS = 10;
constexpr uint32_t CS1237_INIT_RETRY_MS = 100;
constexpr uint32_t CS1237_EXTI_IRQ_PRIORITY = 0;
constexpr uint8_t CS1237_ALL_SENSORS_MASK =
    static_cast<uint8_t>((1U << CS1237_COUNT) - 1U);

enum class Cs1237ReadResult : uint8_t {
  Ok,
  ReadyTimeout,
  StaleReady,
  ReadyLost,
  UnexpectedConfigUpdate,
  ReservedStatusSet,
  FrameNotClosed,
};

struct Cs1237ReadyInfo {
  uint32_t detectedCycles;
};

enum class Cs1237ReadyPhase : uint8_t {
  Disabled,
  Armed,
  Pending,
  Reading,
  Recovery,
  Config,
};

struct Cs1237ReadyState {
  volatile Cs1237ReadyPhase phase;
  volatile uint32_t edgeCycles;
  volatile uint32_t armedCycles;
};

struct MedianSample {
  int32_t value;
  uint32_t cycles;
};

struct MedianFilter3 {
  MedianSample samples[3];
  uint8_t count;
  uint8_t writeIndex;
};

// Эти счетчики не передаются в основном кадре, чтобы сохранить совместимость
// протокола #01. Их можно наблюдать через отладчик по символу
// cs1237Diagnostics.
struct Cs1237Diagnostics {
  volatile uint32_t successfulReads;
  volatile uint32_t drdyInterrupts;
  volatile uint32_t ignoredDrdyInterrupts;
  volatile uint32_t readyLowOnEntry;
  volatile uint32_t readyTimeouts;
  volatile uint32_t staleReady;
  volatile uint32_t unstableReady;
  volatile uint32_t readyLost;
  volatile uint32_t configUpdateFlags;
  volatile uint32_t unexpectedConfigUpdates;
  volatile uint32_t reservedStatusErrors;
  volatile uint32_t frameCloseErrors;
  volatile uint32_t rejectedSamples;
  volatile uint32_t resyncs;
  volatile uint32_t medianInputs;
  volatile uint32_t medianOutputs;
  volatile uint32_t medianResets;
  volatile uint32_t lastTransactionUs;
  volatile uint32_t maxTransactionUs;
  // В рабочих чтениях отсчет начинается в EXTI callback. Если в этот момент
  // IRQ были запрещены другой транзакцией, callback и timestamp запоздают на
  // ее длительность. При конфигурации остается polling-вариант.
  volatile uint32_t lastReadyDetectedToClockUs;
  volatile uint32_t maxReadyDetectedToClockUs;
};

Cs1237Diagnostics cs1237Diagnostics[CS1237_COUNT] = {};
Cs1237ReadyState cs1237ReadyStates[CS1237_COUNT] = {};
MedianFilter3 cs1237MedianFilters[CS1237_COUNT] = {};
bool cs1237Initialized[CS1237_COUNT] = {};
bool cs1237ReadyInterruptsActive = false;
uint8_t cs1237FreshFilteredMask = 0;
int32_t cs1237FilteredValues[CS1237_COUNT] = {};
uint32_t cs1237FilteredAtCycles[CS1237_COUNT] = {};
uint32_t cs1237RecoveryRetryAtMs[CS1237_COUNT] = {};
uint32_t cs1237InitRetryAtMs[CS1237_COUNT] = {};
volatile uint32_t cs1237FrameSkewRejects = 0;

constexpr uint8_t FRAME_HEADER[] = {'#', '0', '1'};
constexpr uint8_t FRAME_TERMINATOR[] = {'\r', '\n'};
constexpr size_t FRAME_PAYLOAD_SIZE = CS1237_COUNT * sizeof(int16_t);
constexpr size_t FRAME_CRC_SIZE = sizeof(uint16_t);
constexpr size_t FRAME_PAYLOAD_OFFSET = sizeof(FRAME_HEADER);
constexpr size_t FRAME_CRC_OFFSET = FRAME_PAYLOAD_OFFSET + FRAME_PAYLOAD_SIZE;
constexpr size_t FRAME_TERMINATOR_OFFSET = FRAME_CRC_OFFSET + FRAME_CRC_SIZE;
constexpr size_t FRAME_SIZE =
    FRAME_TERMINATOR_OFFSET + sizeof(FRAME_TERMINATOR);
static_assert(FRAME_SIZE == 15, "Unexpected contact sensor frame size");

constexpr uint8_t CMD_WRITE_CFG = 0x65;
constexpr uint8_t CMD_READ_CFG = 0x56;

constexpr uint8_t CFG_RESERVED = 1U << 7;
constexpr uint8_t CFG_SPEED_MASK = 0b11U << 4;
constexpr uint8_t CFG_SPEED_640_HZ = 0b10U << 4;
constexpr uint8_t CFG_PGA_MASK = 0b11U << 2;
constexpr uint8_t CFG_PGA_128 = 0b11U << 2;
constexpr uint8_t CFG_CHANNEL_MASK = 0b11U;
constexpr uint8_t CFG_CHANNEL_A = 0b00U;

bool cs1237_wait_ready(const Cs1237Pins &sensor, uint8_t sensorIndex,
                       Cs1237ReadyInfo *readyInfo = nullptr,
                       uint32_t timeoutMs = CS1237_READY_TIMEOUT_MS);
static inline void cs1237_sck_pulse(const Cs1237Pins &sensor);
uint32_t cs1237_read_bits(const Cs1237Pins &sensor, uint8_t bitCount);
void cs1237_write_bits(const Cs1237Pins &sensor, uint32_t value,
                       uint8_t bitCount);
bool cs1237_begin_config_command(const Cs1237Pins &sensor,
                                 uint8_t sensorIndex, uint8_t command,
                                 uint32_t timeoutMs = CS1237_READY_TIMEOUT_MS);
Cs1237ReadResult cs1237_read(const Cs1237Pins &sensor, uint8_t sensorIndex,
                             int32_t &value,
                             bool allowConfigUpdate = false);
Cs1237ReadResult cs1237_read_pending(uint8_t sensorIndex, int32_t &value,
                                     uint32_t &sampleCycles);
bool cs1237_read_config(const Cs1237Pins &sensor, uint8_t sensorIndex,
                        uint8_t &cfg,
                        uint32_t timeoutMs = CS1237_READY_TIMEOUT_MS);
bool cs1237_write_config(const Cs1237Pins &sensor, uint8_t sensorIndex,
                         uint8_t cfg);
bool cs1237_init_640_hz(const Cs1237Pins &sensor, uint8_t sensorIndex);
void cs1237_resync(const Cs1237Pins &sensor, uint8_t sensorIndex);
void cs1237_record_ready(uint8_t sensorIndex);
void cs1237_r1_ready_isr();
void cs1237_l1_ready_isr();
void cs1237_r2_ready_isr();
void cs1237_l2_ready_isr();
void cs1237_suspend_ready_interrupt(uint8_t sensorIndex,
                                    Cs1237ReadyPhase phase);
bool cs1237_attach_ready_interrupt(uint8_t sensorIndex);
void cs1237_rearm_ready_locked(uint8_t sensorIndex);
int8_t cs1237_find_oldest_pending();
int8_t cs1237_find_ready_timeout();
bool cs1237_filtered_frame_is_coherent();
bool cs1237_recover_and_arm(uint8_t sensorIndex);
void median_filter_reset(uint8_t sensorIndex);
bool median_filter_push(uint8_t sensorIndex, int32_t input,
                        uint32_t inputCycles, int32_t &output,
                        uint32_t &outputCycles);
void send_contact_frame(const int32_t *values);
uint32_t cycles_to_microseconds(uint32_t cycles);
void update_maximum(volatile uint32_t &maximum, uint32_t value);
uint16_t crc16_ccitt_false(const uint8_t *data, size_t size);

// ФВЧ сохранен для последующих экспериментов, но сейчас не используется.
constexpr float HPF_CUTOFF_HZ = 5.0f;

long highPassFilter(long input) {
  static bool initialized = false;
  static float previousInput = 0.0f;
  static float previousOutput = 0.0f;
  static uint32_t previousTimeUs = 0;

  const uint32_t currentTimeUs = micros();
  if (!initialized) {
    previousInput = (float)input;
    previousTimeUs = currentTimeUs;
    initialized = true;
    return 0;
  }

  const float dt = (float)(currentTimeUs - previousTimeUs) * 1.0e-6f;
  const float rc = 1.0f / (2.0f * PI * HPF_CUTOFF_HZ);
  const float alpha = rc / (rc + dt);
  const float output = alpha * (previousOutput + (float)input - previousInput);

  previousInput = (float)input;
  previousOutput = output;
  previousTimeUs = currentTimeUs;

  return (long)(output + (output >= 0.0f ? 0.5f : -0.5f));
}

uint32_t cycles_to_microseconds(uint32_t cycles) {
  const uint32_t cyclesPerMicrosecond = SystemCoreClock / 1000000U;
  return cyclesPerMicrosecond == 0U ? 0U : cycles / cyclesPerMicrosecond;
}

void update_maximum(volatile uint32_t &maximum, uint32_t value) {
  if (value > maximum) maximum = value;
}

static inline bool deadline_reached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

void cs1237_record_ready(uint8_t sensorIndex) {
  Cs1237ReadyState &state = cs1237ReadyStates[sensorIndex];
  Cs1237Diagnostics &diagnostics = cs1237Diagnostics[sensorIndex];
  const uint32_t edgeCycles = dwt_getCycles();
  ++diagnostics.drdyInterrupts;

  // EXTI текущего канала маскируется на время выдачи data/status. Guard также
  // отбрасывает редкий callback замаскированной линии из общего IRQ-вектора.
  if (state.phase != Cs1237ReadyPhase::Armed ||
      digitalRead(CS1237_SENSORS[sensorIndex].dout) != LOW) {
    ++diagnostics.ignoredDrdyInterrupts;
    return;
  }

  state.edgeCycles = edgeCycles;
  __DMB();
  state.phase = Cs1237ReadyPhase::Pending;
}

void cs1237_r1_ready_isr() { cs1237_record_ready(0); }
void cs1237_l1_ready_isr() { cs1237_record_ready(1); }
void cs1237_r2_ready_isr() { cs1237_record_ready(2); }
void cs1237_l2_ready_isr() { cs1237_record_ready(3); }

void cs1237_suspend_ready_interrupt(uint8_t sensorIndex,
                                    Cs1237ReadyPhase phase) {
  const uint32_t savedPrimask = __get_PRIMASK();
  __disable_irq();

  const uint32_t extiLine = CS1237_SENSORS[sensorIndex].extiLine;
  LL_EXTI_DisableIT_0_31(extiLine);
  LL_EXTI_ClearFlag_0_31(extiLine);
  cs1237ReadyStates[sensorIndex].phase = phase;
  __DMB();

  __set_PRIMASK(savedPrimask);
}

bool cs1237_attach_ready_interrupt(uint8_t sensorIndex) {
  const Cs1237Pins &sensor = CS1237_SENSORS[sensorIndex];
  Cs1237ReadyState &state = cs1237ReadyStates[sensorIndex];
  const uint32_t savedPrimask = __get_PRIMASK();
  __disable_irq();

  state.phase = Cs1237ReadyPhase::Disabled;
  LL_EXTI_DisableIT_0_31(sensor.extiLine);
  LL_EXTI_ClearFlag_0_31(sensor.extiLine);

  // При входе линия обязана быть HIGH после init/PowerDown. LOW здесь имеет
  // неизвестный возраст, поэтому его нельзя объявлять новым DRDY.
  if (digitalRead(sensor.dout) != HIGH) {
    state.phase = Cs1237ReadyPhase::Recovery;
    __set_PRIMASK(savedPrimask);
    return false;
  }

  switch (sensorIndex) {
    case 0:
      attachInterrupt(digitalPinToInterrupt(sensor.dout),
                      cs1237_r1_ready_isr, FALLING);
      break;
    case 1:
      attachInterrupt(digitalPinToInterrupt(sensor.dout),
                      cs1237_l1_ready_isr, FALLING);
      break;
    case 2:
      attachInterrupt(digitalPinToInterrupt(sensor.dout),
                      cs1237_r2_ready_isr, FALLING);
      break;
    default:
      attachInterrupt(digitalPinToInterrupt(sensor.dout),
                      cs1237_l2_ready_isr, FALLING);
      break;
  }

  // Callback лишь ставит флаг и снимает DWT timestamp. Высокий приоритет
  // уменьшает неизвестную задержку физический фронт -> время ISR; обычное
  // чтение все равно защищено PRIMASK.
  const IRQn_Type extiIrq = sensorIndex < 2 ? EXTI9_5_IRQn
                                            : EXTI15_10_IRQn;
  HAL_NVIC_SetPriority(extiIrq, CS1237_EXTI_IRQ_PRIORITY, 0);

  // attachInterrupt включает линию. Снова маскируем ее, чтобы атомарно
  // установить software-state и закрыть гонку между clear/enable/read.
  LL_EXTI_DisableIT_0_31(sensor.extiLine);
  LL_EXTI_ClearFlag_0_31(sensor.extiLine);
  state.armedCycles = dwt_getCycles();
  state.phase = Cs1237ReadyPhase::Armed;
  __DMB();
  LL_EXTI_EnableIT_0_31(sensor.extiLine);
  __DSB();

  // Фронт мог прийти во время attach/enable, пока глобальные IRQ запрещены.
  if (digitalRead(sensor.dout) == LOW) {
    state.edgeCycles = dwt_getCycles();
    __DMB();
    state.phase = Cs1237ReadyPhase::Pending;
    LL_EXTI_ClearFlag_0_31(sensor.extiLine);
  }

  __set_PRIMASK(savedPrimask);
  return true;
}

// Вызывается только с запрещенными IRQ после того, как 27-й такт вернул
// DOUT в HIGH. EXTI-mode GPIO при обычном чтении не менялся.
void cs1237_rearm_ready_locked(uint8_t sensorIndex) {
  const Cs1237Pins &sensor = CS1237_SENSORS[sensorIndex];
  Cs1237ReadyState &state = cs1237ReadyStates[sensorIndex];

  LL_EXTI_ClearFlag_0_31(sensor.extiLine);
  state.armedCycles = dwt_getCycles();
  state.phase = Cs1237ReadyPhase::Armed;
  __DMB();
  LL_EXTI_EnableIT_0_31(sensor.extiLine);
  __DSB();

  // Если новый DRDY попал в окно перевооружения, аппаратный pending
  // заменяем software-pending и не ждем второго фронта.
  if (digitalRead(sensor.dout) == LOW) {
    state.edgeCycles = dwt_getCycles();
    __DMB();
    state.phase = Cs1237ReadyPhase::Pending;
    LL_EXTI_ClearFlag_0_31(sensor.extiLine);
  }
}

int8_t cs1237_find_oldest_pending() {
  const uint32_t savedPrimask = __get_PRIMASK();
  __disable_irq();

  const uint32_t nowCycles = dwt_getCycles();
  uint32_t oldestAgeCycles = 0;
  int8_t oldestIndex = -1;
  for (uint8_t i = 0; i < CS1237_COUNT; ++i) {
    const Cs1237ReadyState &state = cs1237ReadyStates[i];
    if (state.phase != Cs1237ReadyPhase::Pending) continue;

    const uint32_t ageCycles = nowCycles - state.edgeCycles;
    if (oldestIndex < 0 || ageCycles > oldestAgeCycles) {
      oldestIndex = static_cast<int8_t>(i);
      oldestAgeCycles = ageCycles;
    }
  }

  __set_PRIMASK(savedPrimask);
  return oldestIndex;
}

int8_t cs1237_find_ready_timeout() {
  const uint32_t savedPrimask = __get_PRIMASK();
  __disable_irq();

  const uint32_t nowCycles = dwt_getCycles();
  int8_t timedOutIndex = -1;
  for (uint8_t i = 0; i < CS1237_COUNT; ++i) {
    const Cs1237ReadyState &state = cs1237ReadyStates[i];
    if (state.phase != Cs1237ReadyPhase::Armed) continue;

    const uint32_t armedUs =
        cycles_to_microseconds(nowCycles - state.armedCycles);
    if (armedUs >= CS1237_READY_TIMEOUT_MS * 1000U) {
      timedOutIndex = static_cast<int8_t>(i);
      break;
    }
  }

  __set_PRIMASK(savedPrimask);
  return timedOutIndex;
}

bool cs1237_filtered_frame_is_coherent() {
  const uint32_t nowCycles = dwt_getCycles();
  uint32_t youngestAgeCycles =
      nowCycles - cs1237FilteredAtCycles[0];
  uint32_t oldestAgeCycles = youngestAgeCycles;

  for (uint8_t i = 1; i < CS1237_COUNT; ++i) {
    const uint32_t ageCycles = nowCycles - cs1237FilteredAtCycles[i];
    if (ageCycles < youngestAgeCycles) youngestAgeCycles = ageCycles;
    if (ageCycles > oldestAgeCycles) oldestAgeCycles = ageCycles;
  }

  const uint32_t oldestAgeUs = cycles_to_microseconds(oldestAgeCycles);
  const uint32_t skewUs =
      cycles_to_microseconds(oldestAgeCycles - youngestAgeCycles);
  return oldestAgeUs <= CS1237_MAX_FILTERED_AGE_US &&
         skewUs <= CS1237_MAX_FRAME_SKEW_US;
}

bool cs1237_recover_and_arm(uint8_t sensorIndex) {
  cs1237FreshFilteredMask = 0;
  median_filter_reset(sensorIndex);
  cs1237_resync(CS1237_SENSORS[sensorIndex], sensorIndex);
  if (cs1237_attach_ready_interrupt(sensorIndex)) return true;

  // Не блокируем остальные каналы бесконечным retry при stuck-low DOUT.
  cs1237RecoveryRetryAtMs[sensorIndex] =
      millis() + CS1237_RECOVERY_RETRY_MS;
  return false;
}

void median_filter_reset(uint8_t sensorIndex) {
  MedianFilter3 &filter = cs1237MedianFilters[sensorIndex];
  filter.count = 0;
  filter.writeIndex = 0;
  ++cs1237Diagnostics[sensorIndex].medianResets;
}

static inline void swap_median_sample(MedianSample &left,
                                      MedianSample &right) {
  if (left.value <= right.value) return;
  const MedianSample temporary = left;
  left = right;
  right = temporary;
}

bool median_filter_push(uint8_t sensorIndex, int32_t input,
                        uint32_t inputCycles, int32_t &output,
                        uint32_t &outputCycles) {
  MedianFilter3 &filter = cs1237MedianFilters[sensorIndex];
  Cs1237Diagnostics &diagnostics = cs1237Diagnostics[sensorIndex];

  // Ограничиваем span всего будущего окна, а не только соседнюю пару.
  // При полном ring слот writeIndex сейчас является самым старым и будет
  // перезаписан, поэтому его возраст учитывать не нужно.
  bool windowExpired = false;
  for (uint8_t i = 0; i < filter.count; ++i) {
    if (filter.count == 3 && i == filter.writeIndex) continue;
    const uint32_t ageUs = cycles_to_microseconds(
        inputCycles - filter.samples[i].cycles);
    if (ageUs > MEDIAN_MAX_WINDOW_SPAN_US) {
      windowExpired = true;
      break;
    }
  }
  if (windowExpired) median_filter_reset(sensorIndex);

  filter.samples[filter.writeIndex] = {input, inputCycles};
  filter.writeIndex = static_cast<uint8_t>((filter.writeIndex + 1U) % 3U);
  if (filter.count < 3) ++filter.count;
  ++diagnostics.medianInputs;

  if (filter.count < 3) return false;

  MedianSample first = filter.samples[0];
  MedianSample second = filter.samples[1];
  MedianSample third = filter.samples[2];
  swap_median_sample(first, second);
  swap_median_sample(second, third);
  swap_median_sample(first, second);
  output = second.value;
  outputCycles = second.cycles;
  ++diagnostics.medianOutputs;
  return true;
}

bool cs1237_wait_ready(const Cs1237Pins &sensor, uint8_t sensorIndex,
                       Cs1237ReadyInfo *readyInfo, uint32_t timeoutMs) {
  Cs1237Diagnostics &diagnostics = cs1237Diagnostics[sensorIndex];
  digitalWrite(sensor.sck, LOW);

  const uint32_t startMs = millis();
  if (digitalRead(sensor.dout) == LOW) ++diagnostics.readyLowOnEntry;

  while (millis() - startMs < timeoutMs) {
    if (digitalRead(sensor.dout) == LOW) {
      const uint32_t lowDetectedCycles = dwt_getCycles();
      const uint32_t cyclesPerMicrosecond = SystemCoreClock / 1000000U;
      const uint32_t stableCycles =
          CS1237_READY_STABLE_US * cyclesPerMicrosecond;
      const uint32_t maximumCheckCycles =
          (CS1237_READY_STABLE_US + CS1237_READY_CHECK_SLACK_US) *
          cyclesPerMicrosecond;

      bool remainedLow = true;
      uint32_t elapsedCycles = 0;
      do {
        if (digitalRead(sensor.dout) != LOW) {
          remainedLow = false;
          break;
        }
        elapsedCycles = dwt_getCycles() - lowDetectedCycles;
      } while (elapsedCycles < stableCycles);

      // Если IRQ растянул проверку, начинаем ее заново: за пропущенное время
      // DOUT мог успеть перейти в HIGH и снова вернуться в LOW.
      if (remainedLow && elapsedCycles <= maximumCheckCycles) {
        if (readyInfo != nullptr) {
          readyInfo->detectedCycles = lowDetectedCycles;
        }
        return true;
      }
      ++diagnostics.unstableReady;
    }
    yield();
  }

  digitalWrite(sensor.sck, LOW);
  ++diagnostics.readyTimeouts;
  return false;
}

static inline void cs1237_sck_pulse(const Cs1237Pins &sensor) {
  digitalWrite(sensor.sck, HIGH);
  delayMicroseconds(1);
  digitalWrite(sensor.sck, LOW);
  delayMicroseconds(1);
}

uint32_t cs1237_read_bits(const Cs1237Pins &sensor, uint8_t bitCount) {
  uint32_t value = 0;
  for (uint8_t i = 0; i < bitCount; ++i) {
    digitalWrite(sensor.sck, HIGH);
    delayMicroseconds(1);  // t6(max)=455 нс; после 1 мкс бит уже стабилен
    value = (value << 1) | static_cast<uint32_t>(digitalRead(sensor.dout));
    digitalWrite(sensor.sck, LOW);
    delayMicroseconds(1);
  }
  return value;
}

void cs1237_write_bits(const Cs1237Pins &sensor, uint32_t value,
                       uint8_t bitCount) {
  while (bitCount > 0) {
    --bitCount;
    digitalWrite(sensor.dout, (value >> bitCount) & 0x01U);
    cs1237_sck_pulse(sensor);
  }
}

bool cs1237_begin_config_command(const Cs1237Pins &sensor,
                                 uint8_t sensorIndex, uint8_t command,
                                 uint32_t timeoutMs) {
  if (!cs1237_wait_ready(sensor, sensorIndex, nullptr, timeoutMs)) return false;

  // Первые 24 такта читают текущий результат, 25..27 завершают кадр.
  for (uint8_t i = 0; i < 27; ++i) cs1237_sck_pulse(sensor);

  pinMode(sensor.dout, OUTPUT);
  digitalWrite(sensor.dout, HIGH);
  cs1237_sck_pulse(sensor);  // 28
  cs1237_sck_pulse(sensor);  // 29
  cs1237_write_bits(sensor, command, 7);  // 30..36
  return true;
}

Cs1237ReadResult cs1237_read(const Cs1237Pins &sensor, uint8_t sensorIndex,
                             int32_t &value, bool allowConfigUpdate) {
  Cs1237Diagnostics &diagnostics = cs1237Diagnostics[sensorIndex];
  Cs1237ReadyInfo readyInfo{};
  if (!cs1237_wait_ready(sensor, sensorIndex, &readyInfo)) {
    return Cs1237ReadResult::ReadyTimeout;
  }

  uint32_t raw = 0;
  uint8_t status = 0;
  bool frameClosed = false;
  bool readyLost = false;
  uint32_t transactionCycles = 0;
  uint32_t readyDetectedToClockCycles = 0;

  // Ожидание DRDY выполняется с разрешенными прерываниями. Запрещаем их
  // только на время тактов 1..27 и восстанавливаем предыдущее значение
  // PRIMASK, не включая IRQ безусловно при выходе.
  const uint32_t savedPrimask = __get_PRIMASK();
  __disable_irq();

  if (digitalRead(sensor.dout) != LOW) {
    readyLost = true;
  } else {
    const uint32_t transactionStartCycles = dwt_getCycles();
    readyDetectedToClockCycles =
        transactionStartCycles - readyInfo.detectedCycles;

    raw = cs1237_read_bits(sensor, 24);
    status = static_cast<uint8_t>(cs1237_read_bits(sensor, 2));
    cs1237_sck_pulse(sensor);  // 27: CS1237 должен вернуть DOUT в HIGH.
    frameClosed = digitalRead(sensor.dout) == HIGH;

    digitalWrite(sensor.sck, LOW);
    transactionCycles = dwt_getCycles() - transactionStartCycles;
  }

  // SCLK должен оставаться LOW при любом результате транзакции.
  digitalWrite(sensor.sck, LOW);
  __set_PRIMASK(savedPrimask);

  if (readyLost) {
    ++diagnostics.readyLost;
    ++diagnostics.rejectedSamples;
    return Cs1237ReadResult::ReadyLost;
  }

  const uint32_t transactionUs =
      cycles_to_microseconds(transactionCycles);
  const uint32_t readyDetectedToClockUs =
      cycles_to_microseconds(readyDetectedToClockCycles);
  diagnostics.lastTransactionUs = transactionUs;
  diagnostics.lastReadyDetectedToClockUs = readyDetectedToClockUs;
  update_maximum(diagnostics.maxTransactionUs, transactionUs);
  update_maximum(diagnostics.maxReadyDetectedToClockUs,
                 readyDetectedToClockUs);

  const bool configUpdated = (status & 0b10U) != 0U;  // такт 25
  const bool reservedStatus = (status & 0b01U) != 0U;  // такт 26
  if (configUpdated) ++diagnostics.configUpdateFlags;

  if (!frameClosed) {
    ++diagnostics.frameCloseErrors;
    ++diagnostics.rejectedSamples;
    return Cs1237ReadResult::FrameNotClosed;
  }
  if (reservedStatus) {
    ++diagnostics.reservedStatusErrors;
    ++diagnostics.rejectedSamples;
    return Cs1237ReadResult::ReservedStatusSet;
  }
  if (configUpdated && !allowConfigUpdate) {
    ++diagnostics.unexpectedConfigUpdates;
    ++diagnostics.rejectedSamples;
    return Cs1237ReadResult::UnexpectedConfigUpdate;
  }

  // Явное преобразование 24-битного дополнительного кода в int32_t.
  int32_t decoded = static_cast<int32_t>(raw);
  if ((raw & 0x00800000UL) != 0U) decoded -= 0x01000000L;
  value = decoded;
  ++diagnostics.successfulReads;
  return Cs1237ReadResult::Ok;
}

Cs1237ReadResult cs1237_read_pending(uint8_t sensorIndex, int32_t &value,
                                     uint32_t &sampleCycles) {
  const Cs1237Pins &sensor = CS1237_SENSORS[sensorIndex];
  Cs1237ReadyState &readyState = cs1237ReadyStates[sensorIndex];
  Cs1237Diagnostics &diagnostics = cs1237Diagnostics[sensorIndex];

  uint32_t raw = 0;
  uint8_t status = 0;
  bool frameClosed = false;
  bool readyLost = false;
  uint32_t transactionCycles = 0;
  uint32_t readyDetectedToClockCycles = 0;

  // EXTI лишь фиксирует DRDY. Все такты выполняются вне ISR и атомарно.
  // Маска конкретной EXTI нужна дополнительно к PRIMASK: биты данных сами
  // создают переходы FALLING, которые иначе станут ложным следующим DRDY.
  const uint32_t savedPrimask = __get_PRIMASK();
  __disable_irq();

  if (readyState.phase != Cs1237ReadyPhase::Pending) {
    __set_PRIMASK(savedPrimask);
    return Cs1237ReadResult::ReadyLost;
  }

  const uint32_t readyEdgeCycles = readyState.edgeCycles;
  readyState.phase = Cs1237ReadyPhase::Reading;
  LL_EXTI_DisableIT_0_31(sensor.extiLine);
  LL_EXTI_ClearFlag_0_31(sensor.extiLine);

  if (digitalRead(sensor.dout) != LOW) {
    readyLost = true;
    digitalWrite(sensor.sck, LOW);
    cs1237_rearm_ready_locked(sensorIndex);
  } else {
    const uint32_t transactionStartCycles = dwt_getCycles();
    readyDetectedToClockCycles = transactionStartCycles - readyEdgeCycles;

    raw = cs1237_read_bits(sensor, 24);
    status = static_cast<uint8_t>(cs1237_read_bits(sensor, 2));
    cs1237_sck_pulse(sensor);  // 27: CS1237 должен вернуть DOUT в HIGH.
    frameClosed = digitalRead(sensor.dout) == HIGH;

    digitalWrite(sensor.sck, LOW);
    transactionCycles = dwt_getCycles() - transactionStartCycles;

    const bool configUpdated = (status & 0b10U) != 0U;
    const bool reservedStatus = (status & 0b01U) != 0U;
    if (frameClosed && !reservedStatus && !configUpdated) {
      cs1237_rearm_ready_locked(sensorIndex);
    } else {
      // Состояние интерфейса или конфигурации требует recovery. Линия EXTI
      // остается замаскирована до init/PowerDown и нового attach.
      readyState.phase = Cs1237ReadyPhase::Recovery;
      __DMB();
    }
  }

  digitalWrite(sensor.sck, LOW);
  __set_PRIMASK(savedPrimask);

  if (readyLost) {
    ++diagnostics.readyLost;
    ++diagnostics.rejectedSamples;
    return Cs1237ReadResult::ReadyLost;
  }

  const uint32_t transactionUs =
      cycles_to_microseconds(transactionCycles);
  const uint32_t readyDetectedToClockUs =
      cycles_to_microseconds(readyDetectedToClockCycles);
  diagnostics.lastTransactionUs = transactionUs;
  diagnostics.lastReadyDetectedToClockUs = readyDetectedToClockUs;
  update_maximum(diagnostics.maxTransactionUs, transactionUs);
  update_maximum(diagnostics.maxReadyDetectedToClockUs,
                 readyDetectedToClockUs);

  const bool configUpdated = (status & 0b10U) != 0U;  // такт 25
  const bool reservedStatus = (status & 0b01U) != 0U;  // такт 26
  if (configUpdated) ++diagnostics.configUpdateFlags;

  if (!frameClosed) {
    ++diagnostics.frameCloseErrors;
    ++diagnostics.rejectedSamples;
    return Cs1237ReadResult::FrameNotClosed;
  }
  if (reservedStatus) {
    ++diagnostics.reservedStatusErrors;
    ++diagnostics.rejectedSamples;
    return Cs1237ReadResult::ReservedStatusSet;
  }
  if (configUpdated) {
    ++diagnostics.unexpectedConfigUpdates;
    ++diagnostics.rejectedSamples;
    return Cs1237ReadResult::UnexpectedConfigUpdate;
  }
  if (readyDetectedToClockUs > CS1237_MAX_READY_AGE_US) {
    ++diagnostics.staleReady;
    ++diagnostics.rejectedSamples;
    return Cs1237ReadResult::StaleReady;
  }

  int32_t decoded = static_cast<int32_t>(raw);
  if ((raw & 0x00800000UL) != 0U) decoded -= 0x01000000L;
  value = decoded;
  sampleCycles = readyEdgeCycles;
  ++diagnostics.successfulReads;
  return Cs1237ReadResult::Ok;
}

bool cs1237_read_config(const Cs1237Pins &sensor, uint8_t sensorIndex,
                        uint8_t &cfg, uint32_t timeoutMs) {
  if (!cs1237_begin_config_command(sensor, sensorIndex, CMD_READ_CFG,
                                   timeoutMs)) {
    return false;
  }

  pinMode(sensor.dout, INPUT);
  cs1237_sck_pulse(sensor);  // 37: направление линии меняется на выход CS1237
  cfg = static_cast<uint8_t>(cs1237_read_bits(sensor, 8));  // 38..45

  cs1237_sck_pulse(sensor);  // 46
  return true;
}

bool cs1237_write_config(const Cs1237Pins &sensor, uint8_t sensorIndex,
                         uint8_t cfg) {
  if (!cs1237_begin_config_command(sensor, sensorIndex, CMD_WRITE_CFG)) {
    return false;
  }

  cs1237_sck_pulse(sensor);  // 37; при записи MCU продолжает управлять DOUT
  cs1237_write_bits(sensor, cfg, 8);  // 38..45

  // На 46-м такте линия снова становится выходом CS1237.
  pinMode(sensor.dout, INPUT);
  cs1237_sck_pulse(sensor);
  return true;
}

bool cs1237_init_640_hz(const Cs1237Pins &sensor, uint8_t sensorIndex) {
  uint8_t cfg;
  // Вызывается непосредственно после PowerDown. Первый DRDY при конфигурации
  // 10 Гц приходит позже обычного 200-мс timeout.
  if (!cs1237_read_config(sensor, sensorIndex, cfg,
                          CS1237_POWER_UP_READY_TIMEOUT_MS)) {
    return false;
  }

  cfg = static_cast<uint8_t>(cfg & ~CFG_RESERVED);
  cfg = static_cast<uint8_t>((cfg & ~CFG_SPEED_MASK) | CFG_SPEED_640_HZ);
  cfg = static_cast<uint8_t>((cfg & ~CFG_PGA_MASK) | CFG_PGA_128);
  cfg = static_cast<uint8_t>((cfg & ~CFG_CHANNEL_MASK) | CFG_CHANNEL_A);

  if (!cs1237_write_config(sensor, sensorIndex, cfg)) return false;

  // При 640 Гц после смены конфигурации нужны четыре периода установления.
  for (uint8_t i = 0; i < 4; ++i) {
    int32_t discarded;
    const Cs1237ReadResult result =
        cs1237_read(sensor, sensorIndex, discarded, true);
    if (result != Cs1237ReadResult::Ok) return false;
  }

  uint8_t appliedCfg;
  if (!cs1237_read_config(sensor, sensorIndex, appliedCfg)) return false;
  constexpr uint8_t verifyMask =
      CFG_SPEED_MASK | CFG_PGA_MASK | CFG_CHANNEL_MASK;
  return (appliedCfg & verifyMask) == (cfg & verifyMask);
}

void cs1237_resync(const Cs1237Pins &sensor, uint8_t sensorIndex) {
  if (cs1237ReadyInterruptsActive) {
    cs1237_suspend_ready_interrupt(sensorIndex,
                                   Cs1237ReadyPhase::Recovery);
  } else {
    cs1237ReadyStates[sensorIndex].phase = Cs1237ReadyPhase::Recovery;
  }

  pinMode(sensor.dout, INPUT);
  digitalWrite(sensor.sck, LOW);
  delayMicroseconds(CS1237_READY_STABLE_US);

  // Документированный PowerDown сбрасывает состояние последовательного
  // интерфейса, сохраняя конфигурацию. После возврата SCLK в LOW CS1237 сам
  // выдерживает установление перед следующим DRDY.
  digitalWrite(sensor.sck, HIGH);
  delayMicroseconds(CS1237_POWER_DOWN_US);
  digitalWrite(sensor.sck, LOW);
  delayMicroseconds(CS1237_POWER_UP_LOW_US);
  ++cs1237Diagnostics[sensorIndex].resyncs;
}

uint16_t crc16_ccitt_false(const uint8_t *data, size_t size) {
  uint16_t crc = 0xFFFFU;
  for (size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8U;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0U
                ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
                : static_cast<uint16_t>(crc << 1U);
    }
  }
  return crc;
}

void send_contact_frame(const int32_t *values) {
  // Формат кадра:
  // "#01" + R1,L1,R2,L2 (int16 LE) + CRC-16/CCITT-FALSE (BE) + "\r\n".
  uint8_t frame[FRAME_SIZE] = {};
  for (size_t i = 0; i < sizeof(FRAME_HEADER); ++i) {
    frame[i] = FRAME_HEADER[i];
  }

  for (uint8_t i = 0; i < CS1237_COUNT; ++i) {
    // Медиана вычисляется по полным signed 24-bit значениям. Только здесь
    // отбрасываются младшие 8 бит для совместимого протокола #01.
    const uint16_t encoded = static_cast<uint16_t>(
        static_cast<uint32_t>(values[i]) >> 8U);
    const size_t offset = FRAME_PAYLOAD_OFFSET + i * sizeof(int16_t);
    frame[offset] = static_cast<uint8_t>(encoded & 0x00FFU);
    frame[offset + 1] = static_cast<uint8_t>(encoded >> 8U);
  }

  const uint16_t crc = crc16_ccitt_false(frame, FRAME_CRC_OFFSET);
  frame[FRAME_CRC_OFFSET] = static_cast<uint8_t>(crc >> 8U);
  frame[FRAME_CRC_OFFSET + 1] = static_cast<uint8_t>(crc & 0x00FFU);
  frame[FRAME_TERMINATOR_OFFSET] = FRAME_TERMINATOR[0];
  frame[FRAME_TERMINATOR_OFFSET + 1] = FRAME_TERMINATOR[1];

  Serial.write(frame, sizeof(frame));
}

void setup() {
  Serial.begin(250000);

  for (const Cs1237Pins &sensor : CS1237_SENSORS) {
    pinMode(sensor.dout, INPUT);
    pinMode(sensor.sck, OUTPUT);
    digitalWrite(sensor.sck, LOW);
  }

  // Повторяем инициализацию, пока CS1237 не ответит и конфигурация не
  // прочитается обратно как 640 Гц, PGA=128, канал A.
  for (uint8_t i = 0; i < CS1237_COUNT; ++i) {
    while (true) {
      // PowerDown выполняется непосредственно перед попыткой: первое LOW,
      // увиденное polling-кодом конфигурации, принадлежит новому преобразованию,
      // а не отсчету, который пролежал непрочитанным во время retry-delay.
      cs1237_resync(CS1237_SENSORS[i], i);
      if (cs1237_init_640_hz(CS1237_SENSORS[i], i)) break;
      delay(CS1237_INIT_RETRY_MS);
    }
    cs1237Initialized[i] = true;
    median_filter_reset(i);
  }

  // Ранние каналы могли успеть выставить старый LOW, пока настраивались
  // поздние. Общий recovery возвращает все линии в HIGH до подключения EXTI,
  // поэтому первый принятый FALLING у каждого АЦП гарантированно новый.
  for (uint8_t i = 0; i < CS1237_COUNT; ++i) {
    cs1237_resync(CS1237_SENSORS[i], i);
  }

  cs1237ReadyInterruptsActive = true;
  for (uint8_t i = 0; i < CS1237_COUNT; ++i) {
    while (!cs1237_attach_ready_interrupt(i)) {
      delay(CS1237_INIT_RETRY_MS);
      cs1237_resync(CS1237_SENSORS[i], i);
    }
  }
}

void loop() {
  const uint32_t nowMs = millis();

  // Runtime reconfiguration меняет режим GPIO DOUT и делает накопленную
  // медиану этого канала недействительной.
  for (uint8_t i = 0; i < CS1237_COUNT; ++i) {
    if (cs1237Initialized[i]) continue;
    if (!deadline_reached(nowMs, cs1237InitRetryAtMs[i])) continue;

    cs1237FreshFilteredMask = 0;
    median_filter_reset(i);
    // Как и при startup, сбрасываем интерфейс после retry-delay, чтобы
    // конфигурационная транзакция началась от заведомо нового DRDY.
    cs1237_resync(CS1237_SENSORS[i], i);
    cs1237_suspend_ready_interrupt(i, Cs1237ReadyPhase::Config);
    if (!cs1237_init_640_hz(CS1237_SENSORS[i], i)) {
      cs1237InitRetryAtMs[i] = millis() + CS1237_INIT_RETRY_MS;
      return;
    }

    cs1237Initialized[i] = true;
    if (!cs1237_attach_ready_interrupt(i)) {
      cs1237_recover_and_arm(i);
    }
    return;
  }

  // Recovery одной неисправной линии выполняется одной попыткой и с паузой
  // между повторами, поэтому остальные EXTI и Serial не блокируются навсегда.
  for (uint8_t i = 0; i < CS1237_COUNT; ++i) {
    if (!cs1237Initialized[i] ||
        cs1237ReadyStates[i].phase != Cs1237ReadyPhase::Recovery ||
        !deadline_reached(nowMs, cs1237RecoveryRetryAtMs[i])) {
      continue;
    }
    cs1237_recover_and_arm(i);
    return;
  }

  // Отсутствие нового DRDY дольше старого 200-мс лимита считается timeout.
  // Проверяем это даже при активности остальных каналов.
  const int8_t timedOutIndex = cs1237_find_ready_timeout();
  if (timedOutIndex >= 0) {
    Cs1237Diagnostics &diagnostics =
        cs1237Diagnostics[static_cast<uint8_t>(timedOutIndex)];
    ++diagnostics.readyTimeouts;
    ++diagnostics.rejectedSamples;
    cs1237FreshFilteredMask = 0;
    cs1237_recover_and_arm(static_cast<uint8_t>(timedOutIndex));
    return;
  }

  const int8_t pendingIndex = cs1237_find_oldest_pending();
  if (pendingIndex < 0) {
    yield();
    return;
  }

  const uint8_t sensorIndex = static_cast<uint8_t>(pendingIndex);
  int32_t rawValue = 0;
  uint32_t sampleCycles = 0;
  const Cs1237ReadResult result =
      cs1237_read_pending(sensorIndex, rawValue, sampleCycles);

  if (result == Cs1237ReadResult::Ok) {
    int32_t filteredValue = 0;
    uint32_t filteredCycles = 0;
    if (!median_filter_push(sensorIndex, rawValue, sampleCycles,
                            filteredValue, filteredCycles)) {
      // Во время cold/stale warm-up продолжаем обслуживать все четыре АЦП,
      // но не выдаем старый или частично прогретый набор.
      cs1237FreshFilteredMask = 0;
      return;
    }

    cs1237FilteredValues[sensorIndex] = filteredValue;
    cs1237FilteredAtCycles[sensorIndex] = filteredCycles;
    cs1237FreshFilteredMask = static_cast<uint8_t>(
        cs1237FreshFilteredMask | (1U << sensorIndex));
    if (cs1237FreshFilteredMask == CS1237_ALL_SENSORS_MASK) {
      if (cs1237_filtered_frame_is_coherent()) {
        send_contact_frame(cs1237FilteredValues);
      } else {
        ++cs1237FrameSkewRejects;
      }
      cs1237FreshFilteredMask = 0;
    }
    return;
  }

  // Любая отвергнутая выборка разрывает текущий четырехканальный набор.
  cs1237FreshFilteredMask = 0;
  if (result == Cs1237ReadResult::UnexpectedConfigUpdate) {
    cs1237Initialized[sensorIndex] = false;
    median_filter_reset(sensorIndex);
    // read_pending уже оставил EXTI замаскированным. PowerDown будет сделан
    // непосредственно перед следующей попыткой init, после этой паузы.
    cs1237InitRetryAtMs[sensorIndex] =
        millis() + CS1237_RECOVERY_RETRY_MS;
    return;
  }

  if (result == Cs1237ReadResult::ReadyLost ||
      result == Cs1237ReadResult::StaleReady) {
    // read_pending уже безопасно перевооружил EXTI: при ReadyLost линия была
    // HIGH, а при StaleReady старый LOW был clock-out. Следующий FALLING снова
    // будет принадлежать новому преобразованию.
    return;
  }

  cs1237_recover_and_arm(sensorIndex);
}
