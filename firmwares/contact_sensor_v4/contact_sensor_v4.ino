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
constexpr uint32_t CS1237_READY_TIMEOUT_MS = 20;
// После PowerDown конфигурация может оставаться заводской (10 Гц). При этой
// скорости первый DRDY может потребовать около 300 мс установления.
constexpr uint32_t CS1237_POWER_UP_READY_TIMEOUT_MS = 400;
// Время отсчитывается от обработанного EXTI-фронта. Даже с задержкой ISR на
// одну транзакцию 500 мкс оставляет большой запас до следующего обновления
// данных при периоде 1.5625 мс (640 Гц).
constexpr uint32_t CS1237_MAX_READY_AGE_US = 500;
// Медиана может выбрать самый старый из трех отсчетов. 7 мс покрывают два
// периода фильтра, взаимную фазу четырех АЦП и время их последовательного
// чтения, но все еще не допускают попадания в кадр действительно старых данных.
constexpr uint32_t CS1237_MAX_FILTERED_AGE_US = 7000;
constexpr uint32_t CS1237_POWER_DOWN_US = 120;
constexpr uint32_t CS1237_POWER_UP_LOW_US = 12;
constexpr uint32_t MEDIAN_MAX_WINDOW_SPAN_US = 4000;
constexpr uint32_t CS1237_EXTI_IRQ_PRIORITY = 0;

enum class Cs1237ReadResult : uint8_t {
  Ok,
  ReadyTimeout,
  StaleReady,
  ReadyLost,
  UnexpectedConfigUpdate,
  ReservedStatusSet,
  FrameNotClosed,
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
  volatile uint32_t edgeMs;
  volatile uint32_t armedMs;
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
  // ее длительность. Конфигурационные транзакции также используют EXTI.
  volatile uint32_t lastReadyDetectedToClockUs;
  volatile uint32_t maxReadyDetectedToClockUs;
  volatile uint32_t recoveryAttempts;
  volatile uint32_t successfulRecoveries;
  volatile uint8_t lastError;
};

Cs1237Diagnostics cs1237Diagnostics[CS1237_COUNT] = {};
Cs1237ReadyState cs1237ReadyStates[CS1237_COUNT] = {};
MedianFilter3 cs1237MedianFilters[CS1237_COUNT] = {};
int32_t cs1237FilteredValues[CS1237_COUNT] = {};
uint32_t cs1237FilteredAtCycles[CS1237_COUNT] = {};

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

constexpr uint32_t CS1237_CONFIG_READY_TIMEOUT_MS = 200;
constexpr uint32_t CS1237_RETRY_DELAYS_MS[] = {100, 200, 400, 800, 1000};
constexpr uint32_t FRAME_RATE_HZ = 600;
constexpr int16_t INVALID_MEASUREMENT = -1;
constexpr uint8_t CONFIG_VERIFY_MASK =
    CFG_RESERVED | CFG_SPEED_MASK | CFG_PGA_MASK | CFG_CHANNEL_MASK;

enum class ChannelPhase : uint8_t {
  Reset, ResetStable, PowerDown, PowerUp, ReadConfig, WriteConfig,
  Settling, VerifyConfig, Warmup, Running, Retry,
};

enum class ChannelError : uint8_t {
  None, ReadyTimeout, StaleReady, ReadyLost, UnexpectedConfigUpdate,
  ReservedStatusSet, FrameNotClosed, ConfigMismatch, OldLow,
};

struct ChannelState {
  ChannelPhase phase;
  uint32_t deadlineUs;
  uint32_t startedMs;
  uint32_t retryAtMs;
  uint32_t lastInputMs;
  uint8_t config;
  uint8_t settlingSamples;
  uint8_t retryLevel;
  bool filteredValid;
};

// Debugger-visible lifecycle and counters; never sent in the serial stream.
ChannelState cs1237Channels[CS1237_COUNT] = {};
volatile uint32_t contactFramesFormed = 0;
volatile uint32_t contactFramesSent = 0;
volatile uint32_t contactFramesSkipped = 0;
uint8_t recoveryCursor = 0;
bool frameClockStarted = false;
uint32_t frameClockUs = 0;
uint64_t frameClockRemainder = 0;

// Explicit declarations keep Arduino's generated prototypes independent of types.
uint32_t cycles_to_microseconds(uint32_t cycles);
void update_maximum(volatile uint32_t &maximum, uint32_t value);
void cs1237_record_ready(uint8_t sensorIndex);
void cs1237_rearm_ready_locked(uint8_t sensorIndex);
void median_filter_reset(uint8_t sensorIndex);

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
  state.edgeMs = millis();
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
    ++cs1237Diagnostics[sensorIndex].readyLowOnEntry;
    __set_PRIMASK(savedPrimask);
    return false;
  }

  pinMode(sensor.dout, INPUT_PULLUP);
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
  state.armedMs = millis();
  state.phase = Cs1237ReadyPhase::Armed;
  __DMB();
  LL_EXTI_EnableIT_0_31(sensor.extiLine);
  __DSB();

  // Фронт мог прийти во время attach/enable, пока глобальные IRQ запрещены.
  if (digitalRead(sensor.dout) == LOW) {
    state.edgeCycles = dwt_getCycles();
    state.edgeMs = millis();
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
  state.armedMs = millis();
  state.phase = Cs1237ReadyPhase::Armed;
  __DMB();
  LL_EXTI_EnableIT_0_31(sensor.extiLine);
  __DSB();

  // Если новый DRDY попал в окно перевооружения, аппаратный pending
  // заменяем software-pending и не ждем второго фронта.
  if (digitalRead(sensor.dout) == LOW) {
    state.edgeCycles = dwt_getCycles();
    state.edgeMs = millis();
    __DMB();
    state.phase = Cs1237ReadyPhase::Pending;
    LL_EXTI_ClearFlag_0_31(sensor.extiLine);
  }
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

Cs1237ReadResult cs1237_read_pending(uint8_t sensorIndex, int32_t &value,
                                     uint32_t &sampleCycles, bool allowConfigUpdate) {
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
  const uint32_t readyEdgeMs = readyState.edgeMs;
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
    if (frameClosed && !reservedStatus && (!configUpdated || allowConfigUpdate)) {
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
  if (configUpdated && !allowConfigUpdate) {
    ++diagnostics.unexpectedConfigUpdates;
    ++diagnostics.rejectedSamples;
    return Cs1237ReadResult::UnexpectedConfigUpdate;
  }
  if (readyDetectedToClockUs > CS1237_MAX_READY_AGE_US ||
      millis() - readyEdgeMs > 1U) {
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


static bool measurement_phase(ChannelPhase phase) {
  return phase == ChannelPhase::Settling || phase == ChannelPhase::Warmup ||
         phase == ChannelPhase::Running;
}

static void fail_channel(uint8_t index, ChannelError error) {
  ChannelState &channel = cs1237Channels[index];
  cs1237_suspend_ready_interrupt(index, Cs1237ReadyPhase::Recovery);
  digitalWrite(CS1237_SENSORS[index].sck, LOW);
  pinMode(CS1237_SENSORS[index].dout, INPUT_PULLUP);
  channel.filteredValid = false;
  median_filter_reset(index);
  cs1237Diagnostics[index].lastError = static_cast<uint8_t>(error);
  channel.retryAtMs = millis() + CS1237_RETRY_DELAYS_MS[channel.retryLevel];
  if (channel.retryLevel < 4) ++channel.retryLevel;
  channel.phase = ChannelPhase::Retry;
}

// A single 46-clock config transaction; all DRDY waiting happens outside it.
static Cs1237ReadResult config_transaction(uint8_t index, uint8_t command,
                                          uint8_t &config) {
  const Cs1237Pins &sensor = CS1237_SENSORS[index];
  Cs1237ReadyState &ready = cs1237ReadyStates[index];
  Cs1237Diagnostics &diagnostics = cs1237Diagnostics[index];
  const uint32_t savedPrimask = __get_PRIMASK();
  __disable_irq();
  LL_EXTI_DisableIT_0_31(sensor.extiLine);
  LL_EXTI_ClearFlag_0_31(sensor.extiLine);
  const uint32_t started = dwt_getCycles();
  const uint32_t readyAgeUs = cycles_to_microseconds(started - ready.edgeCycles);
  const bool stale = readyAgeUs > CS1237_MAX_READY_AGE_US ||
                     millis() - ready.edgeMs > 1U;
  if (ready.phase != Cs1237ReadyPhase::Pending ||
      digitalRead(sensor.dout) != LOW) {
    cs1237_rearm_ready_locked(index);
    __set_PRIMASK(savedPrimask);
    ++diagnostics.readyLost;
    ++diagnostics.rejectedSamples;
    return Cs1237ReadResult::ReadyLost;
  }
  ready.phase = Cs1237ReadyPhase::Reading;
  cs1237_read_bits(sensor, 24);
  const uint8_t status = static_cast<uint8_t>(cs1237_read_bits(sensor, 2));
  cs1237_sck_pulse(sensor);
  bool closed = digitalRead(sensor.dout) == HIGH;
  if (closed && (status & 1U) == 0U && !stale) {
    digitalWrite(sensor.dout, HIGH);
    pinMode(sensor.dout, OUTPUT);
    cs1237_sck_pulse(sensor);  // 28
    cs1237_sck_pulse(sensor);  // 29
    cs1237_write_bits(sensor, command, 7);  // 30..36
    if (command == CMD_READ_CFG) {
      pinMode(sensor.dout, INPUT_PULLUP);
      cs1237_sck_pulse(sensor);  // 37
      config = static_cast<uint8_t>(cs1237_read_bits(sensor, 8));
    } else {
      cs1237_sck_pulse(sensor);  // 37
      cs1237_write_bits(sensor, config, 8);
      pinMode(sensor.dout, INPUT_PULLUP);
    }
    cs1237_sck_pulse(sensor);  // 46
    closed = digitalRead(sensor.dout) == HIGH;
  }
  digitalWrite(sensor.sck, LOW);
  // pinMode(INPUT_PULLUP) changes EXTI mode; attach restores it from a known HIGH.
  ready.phase = Cs1237ReadyPhase::Disabled;
  const uint32_t transactionUs = cycles_to_microseconds(dwt_getCycles() - started);
  __set_PRIMASK(savedPrimask);
  diagnostics.lastTransactionUs = transactionUs;
  diagnostics.lastReadyDetectedToClockUs = readyAgeUs;
  update_maximum(diagnostics.maxTransactionUs, transactionUs);
  update_maximum(diagnostics.maxReadyDetectedToClockUs, readyAgeUs);
  if ((status & 2U) != 0U) ++diagnostics.configUpdateFlags;
  Cs1237ReadResult result = Cs1237ReadResult::Ok;
  if (!closed) {
    ++diagnostics.frameCloseErrors;
    result = Cs1237ReadResult::FrameNotClosed;
  } else if ((status & 1U) != 0U) {
    ++diagnostics.reservedStatusErrors;
    result = Cs1237ReadResult::ReservedStatusSet;
  } else if (stale) {
    ++diagnostics.staleReady;
    result = Cs1237ReadResult::StaleReady;
  }
  if (result == Cs1237ReadResult::Ok) ++diagnostics.successfulReads;
  else ++diagnostics.rejectedSamples;
  return result;
}

static void expire_filtered(uint8_t index) {
  ChannelState &channel = cs1237Channels[index];
  // The millisecond guard also rejects data older than a full DWT revolution.
  // Invalidity is latched until a new complete median window is available.
  if (channel.filteredValid &&
      (millis() - channel.lastInputMs > 7U ||
       cycles_to_microseconds(dwt_getCycles() - cs1237FilteredAtCycles[index]) >
           CS1237_MAX_FILTERED_AGE_US)) {
    channel.filteredValid = false;
    median_filter_reset(index);
  }
}

static void service_measurements() {
  // At most one read per channel per pass. Choose oldest pending first.
  for (uint8_t readCount = 0; readCount < CS1237_COUNT; ++readCount) {
    int8_t selected = -1;
    uint32_t oldest = 0;
    const uint32_t nowCycles = dwt_getCycles();
    for (uint8_t i = 0; i < CS1237_COUNT; ++i) {
      if (!measurement_phase(cs1237Channels[i].phase) ||
          cs1237ReadyStates[i].phase != Cs1237ReadyPhase::Pending) continue;
      const uint32_t age = nowCycles - cs1237ReadyStates[i].edgeCycles;
      if (selected < 0 || age > oldest) { selected = i; oldest = age; }
    }
    if (selected < 0) break;
    const uint8_t index = static_cast<uint8_t>(selected);
    ChannelState &channel = cs1237Channels[index];
    int32_t raw = 0;
    uint32_t atCycles = 0;
    const Cs1237ReadResult result = cs1237_read_pending(
        index, raw, atCycles, channel.phase == ChannelPhase::Settling);
    if (result != Cs1237ReadResult::Ok) {
      cs1237Diagnostics[index].lastError = static_cast<uint8_t>(result);
    }
    if (result == Cs1237ReadResult::ReadyLost ||
        result == Cs1237ReadResult::StaleReady) continue;
    if (result != Cs1237ReadResult::Ok) {
      fail_channel(index, static_cast<ChannelError>(result));
      continue;
    }
    const uint32_t nowMs = millis();
    if (nowMs - channel.lastInputMs > 4U) median_filter_reset(index);
    channel.lastInputMs = nowMs;
    if (channel.phase == ChannelPhase::Settling) {
      if (++channel.settlingSamples == 4) {
        channel.phase = ChannelPhase::VerifyConfig;
        channel.startedMs = nowMs;
      }
      continue;
    }
    channel.filteredValid = median_filter_push(
        index, raw, atCycles, cs1237FilteredValues[index],
        cs1237FilteredAtCycles[index]);
    if (channel.filteredValid && channel.phase == ChannelPhase::Warmup) {
      channel.phase = ChannelPhase::Running;
      channel.retryLevel = 0;
      ++cs1237Diagnostics[index].successfulRecoveries;
    }
  }
  for (uint8_t i = 0; i < CS1237_COUNT; ++i) {
    expire_filtered(i);
    if (measurement_phase(cs1237Channels[i].phase) &&
        millis() - cs1237Channels[i].lastInputMs >= CS1237_READY_TIMEOUT_MS) {
      ++cs1237Diagnostics[i].readyTimeouts;
      ++cs1237Diagnostics[i].rejectedSamples;
      fail_channel(i, ChannelError::ReadyTimeout);
    }
  }
}

static bool service_recovery_step(uint8_t index) {
  const Cs1237Pins &sensor = CS1237_SENSORS[index];
  ChannelState &channel = cs1237Channels[index];
  const uint32_t nowMs = millis();
  const uint32_t nowUs = micros();
  switch (channel.phase) {
    case ChannelPhase::Retry:
      if (!deadline_reached(nowMs, channel.retryAtMs)) return false;
      channel.phase = ChannelPhase::Reset;
      return true;
    case ChannelPhase::Reset:
      cs1237_suspend_ready_interrupt(index, Cs1237ReadyPhase::Recovery);
      channel.filteredValid = false;
      median_filter_reset(index);
      pinMode(sensor.dout, INPUT_PULLUP);
      digitalWrite(sensor.sck, LOW);
      channel.deadlineUs = nowUs + CS1237_READY_STABLE_US;
      channel.phase = ChannelPhase::ResetStable;
      ++cs1237Diagnostics[index].recoveryAttempts;
      return true;
    case ChannelPhase::ResetStable:
      if (!deadline_reached(nowUs, channel.deadlineUs)) return false;
      digitalWrite(sensor.sck, HIGH);
      channel.deadlineUs = nowUs + CS1237_POWER_DOWN_US;
      channel.phase = ChannelPhase::PowerDown;
      return true;
    case ChannelPhase::PowerDown:
      if (!deadline_reached(nowUs, channel.deadlineUs)) return false;
      digitalWrite(sensor.sck, LOW);
      channel.deadlineUs = nowUs + CS1237_POWER_UP_LOW_US;
      channel.phase = ChannelPhase::PowerUp;
      ++cs1237Diagnostics[index].resyncs;
      return true;
    case ChannelPhase::PowerUp:
      if (!deadline_reached(nowUs, channel.deadlineUs)) return false;
      if (!cs1237_attach_ready_interrupt(index)) {
        fail_channel(index, ChannelError::OldLow);
      } else {
        channel.phase = ChannelPhase::ReadConfig;
        channel.startedMs = nowMs;
      }
      return true;
    case ChannelPhase::ReadConfig:
    case ChannelPhase::WriteConfig:
    case ChannelPhase::VerifyConfig: {
      const uint32_t timeout = channel.phase == ChannelPhase::ReadConfig
          ? CS1237_POWER_UP_READY_TIMEOUT_MS : CS1237_CONFIG_READY_TIMEOUT_MS;
      if (cs1237ReadyStates[index].phase != Cs1237ReadyPhase::Pending) {
        if (nowMs - channel.startedMs < timeout) return false;
        ++cs1237Diagnostics[index].readyTimeouts;
        ++cs1237Diagnostics[index].rejectedSamples;
        fail_channel(index, ChannelError::ReadyTimeout);
        return true;
      }
      if (cycles_to_microseconds(dwt_getCycles() -
              cs1237ReadyStates[index].edgeCycles) < CS1237_READY_STABLE_US) {
        return false;
      }
      const bool writing = channel.phase == ChannelPhase::WriteConfig;
      uint8_t config = channel.config;
      const auto result = config_transaction(
          index, writing ? CMD_WRITE_CFG : CMD_READ_CFG, config);
      if (result != Cs1237ReadResult::Ok) {
        fail_channel(index, static_cast<ChannelError>(result));
        return true;
      }
      if (channel.phase == ChannelPhase::ReadConfig) {
        channel.config = static_cast<uint8_t>(
            (config & ~CONFIG_VERIFY_MASK) | CFG_SPEED_640_HZ | CFG_PGA_128 |
            CFG_CHANNEL_A);
        channel.phase = ChannelPhase::WriteConfig;
      } else if (writing) {
        channel.settlingSamples = 0;
        channel.phase = ChannelPhase::Settling;
      } else {
        if ((config & CONFIG_VERIFY_MASK) !=
            (channel.config & CONFIG_VERIFY_MASK)) {
          fail_channel(index, ChannelError::ConfigMismatch);
          return true;
        }
        channel.phase = ChannelPhase::Warmup;
        median_filter_reset(index);
      }
      channel.startedMs = millis();
      channel.lastInputMs = millis();
      if (!cs1237_attach_ready_interrupt(index)) {
        fail_channel(index, ChannelError::OldLow);
      }
      return true;
    }
    default: return false;
  }
}

static void service_recovery() {
  for (uint8_t count = 0; count < CS1237_COUNT; ++count) {
    const uint8_t index = recoveryCursor;
    recoveryCursor = static_cast<uint8_t>((recoveryCursor + 1U) % CS1237_COUNT);
    if (service_recovery_step(index)) return;
  }
}

static uint16_t encode_measurement(int32_t value, bool valid) {
  if (!valid) return static_cast<uint16_t>(INVALID_MEASUREMENT);
  // Same signed 24-bit -> int16 conversion as v3; reserve just one wire code.
  const uint16_t encoded = static_cast<uint16_t>(static_cast<uint32_t>(value) >> 8U);
  return encoded == 0xFFFFU ? 0xFFFEU : encoded;
}

static void send_contact_frame() {
  uint8_t frame[FRAME_SIZE] = {};
  for (size_t i = 0; i < sizeof(FRAME_HEADER); ++i) frame[i] = FRAME_HEADER[i];
  for (uint8_t i = 0; i < CS1237_COUNT; ++i) {
    expire_filtered(i);
    const uint16_t encoded = encode_measurement(
        cs1237FilteredValues[i], cs1237Channels[i].filteredValid);
    const size_t offset = FRAME_PAYLOAD_OFFSET + i * sizeof(int16_t);
    frame[offset] = static_cast<uint8_t>(encoded);
    frame[offset + 1] = static_cast<uint8_t>(encoded >> 8U);
  }
  const uint16_t crc = crc16_ccitt_false(frame, FRAME_CRC_OFFSET);
  frame[FRAME_CRC_OFFSET] = static_cast<uint8_t>(crc >> 8U);
  frame[FRAME_CRC_OFFSET + 1] = static_cast<uint8_t>(crc);
  frame[FRAME_TERMINATOR_OFFSET] = '\r';
  frame[FRAME_TERMINATOR_OFFSET + 1] = '\n';
  ++contactFramesFormed;
  // Serial has one writer. UART IRQ only frees space after this check.
  if (Serial.availableForWrite() < static_cast<int>(sizeof(frame))) {
    ++contactFramesSkipped;
    return;
  }
  Serial.write(frame, sizeof(frame));
  ++contactFramesSent;
}

static void service_transmit() {
  const uint32_t nowUs = micros();
  if (!frameClockStarted) {
    frameClockStarted = true;
    frameClockUs = nowUs;
    send_contact_frame();
    return;
  }
  frameClockRemainder += static_cast<uint64_t>(nowUs - frameClockUs) * FRAME_RATE_HZ;
  frameClockUs = nowUs;
  const uint32_t due = static_cast<uint32_t>(frameClockRemainder / 1000000U);
  frameClockRemainder %= 1000000U;
  if (due == 0) return;
  contactFramesSkipped += due - 1U;
  send_contact_frame();
}

void setup() {
  Serial.begin(250000);
  for (const Cs1237Pins &sensor : CS1237_SENSORS) {
    pinMode(sensor.dout, INPUT_PULLUP);
    pinMode(sensor.sck, OUTPUT);
    digitalWrite(sensor.sck, LOW);
  }
}

void loop() {
  service_measurements();
  service_transmit();
  service_recovery();
}
