#include <VBCoreG4_arduino_system.h>

#define PIN_L1_DOUT PB_7_ALT1
#define PIN_L1_SCK PC6

constexpr uint8_t CMD_WRITE_CFG = 0x65;
constexpr uint8_t CMD_READ_CFG = 0x56;

constexpr uint8_t CFG_RESERVED = 1U << 7;
constexpr uint8_t CFG_SPEED_MASK = 0b11U << 4;
constexpr uint8_t CFG_SPEED_640_HZ = 0b10U << 4;
constexpr uint8_t CFG_PGA_MASK = 0b11U << 2;
constexpr uint8_t CFG_PGA_128 = 0b11U << 2;
constexpr uint8_t CFG_CHANNEL_MASK = 0b11U;
constexpr uint8_t CFG_CHANNEL_A = 0b00U;

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

bool cs1237_wait_ready(uint32_t timeoutMs = 200) {
  const uint32_t startMs = millis();
  while (digitalRead(PIN_L1_DOUT) == HIGH) {
    if (millis() - startMs >= timeoutMs) return false;
    yield();
  }
  return true;
}

static inline void cs1237_sck_pulse() {
  digitalWrite(PIN_L1_SCK, HIGH);
  delayMicroseconds(1);
  digitalWrite(PIN_L1_SCK, LOW);
  delayMicroseconds(1);
}

bool cs1237_read(int32_t &value) {
  if (!cs1237_wait_ready()) return false;

  uint32_t raw = 0;
  for (uint8_t i = 0; i < 24; ++i) {
    digitalWrite(PIN_L1_SCK, HIGH);
    delayMicroseconds(1);  // t6(max)=455 нс; после 1 мкс бит уже стабилен
    raw = (raw << 1) | (uint32_t)digitalRead(PIN_L1_DOUT);
    digitalWrite(PIN_L1_SCK, LOW);
    delayMicroseconds(1);
  }

  // Такты 25 и 26 выдают статус, такт 27 завершает транзакцию.
  for (uint8_t i = 0; i < 3; ++i) cs1237_sck_pulse();

  // Явное преобразование 24-битного дополнительного кода в int32_t.
  value = (int32_t)raw;
  if (raw & 0x00800000UL) value -= 0x01000000L;
  return true;
}

bool cs1237_read_config(uint8_t &cfg) {
  if (!cs1237_wait_ready()) return false;

  // Первые 24 такта читают текущий результат, 25..27 завершают кадр.
  for (uint8_t i = 0; i < 27; ++i) cs1237_sck_pulse();

  pinMode(PIN_L1_DOUT, OUTPUT);
  digitalWrite(PIN_L1_DOUT, HIGH);
  cs1237_sck_pulse();  // 28
  cs1237_sck_pulse();  // 29

  for (int8_t bit = 6; bit >= 0; --bit) {
    digitalWrite(PIN_L1_DOUT, (CMD_READ_CFG >> bit) & 0x01);
    cs1237_sck_pulse();  // 30..36
  }

  pinMode(PIN_L1_DOUT, INPUT);
  cs1237_sck_pulse();  // 37: направление линии меняется на выход CS1237

  cfg = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    digitalWrite(PIN_L1_SCK, HIGH);
    delayMicroseconds(1);
    cfg = (cfg << 1) | (uint8_t)digitalRead(PIN_L1_DOUT);
    digitalWrite(PIN_L1_SCK, LOW);
    delayMicroseconds(1);
  }

  cs1237_sck_pulse();  // 46
  return true;
}

bool cs1237_write_config(uint8_t cfg) {
  if (!cs1237_wait_ready()) return false;

  for (uint8_t i = 0; i < 27; ++i) cs1237_sck_pulse();

  pinMode(PIN_L1_DOUT, OUTPUT);
  digitalWrite(PIN_L1_DOUT, HIGH);
  cs1237_sck_pulse();  // 28
  cs1237_sck_pulse();  // 29

  for (int8_t bit = 6; bit >= 0; --bit) {
    digitalWrite(PIN_L1_DOUT, (CMD_WRITE_CFG >> bit) & 0x01);
    cs1237_sck_pulse();  // 30..36
  }

  cs1237_sck_pulse();  // 37; при записи MCU продолжает управлять DOUT
  for (int8_t bit = 7; bit >= 0; --bit) {
    digitalWrite(PIN_L1_DOUT, (cfg >> bit) & 0x01);
    cs1237_sck_pulse();  // 38..45
  }

  // На 46-м такте линия снова становится выходом CS1237.
  pinMode(PIN_L1_DOUT, INPUT);
  cs1237_sck_pulse();
  return true;
}

bool cs1237_init_640_hz() {
  uint8_t cfg;
  if (!cs1237_read_config(cfg)) return false;

  cfg &= ~CFG_RESERVED;
  cfg = (cfg & ~CFG_SPEED_MASK) | CFG_SPEED_640_HZ;
  cfg = (cfg & ~CFG_PGA_MASK) | CFG_PGA_128;
  cfg = (cfg & ~CFG_CHANNEL_MASK) | CFG_CHANNEL_A;

  if (!cs1237_write_config(cfg)) return false;

  // При 640 Гц после смены конфигурации нужны четыре периода установления.
  for (uint8_t i = 0; i < 4; ++i) {
    int32_t discarded;
    if (!cs1237_read(discarded)) return false;
  }

  uint8_t appliedCfg;
  if (!cs1237_read_config(appliedCfg)) return false;
  constexpr uint8_t verifyMask =
      CFG_SPEED_MASK | CFG_PGA_MASK | CFG_CHANNEL_MASK;
  return (appliedCfg & verifyMask) == (cfg & verifyMask);
}

void setup() {
  Serial.begin(250000);

  pinMode(PIN_L1_DOUT, INPUT);
  pinMode(PIN_L1_SCK, OUTPUT);
  digitalWrite(PIN_L1_SCK, LOW);

  // Повторяем инициализацию, пока CS1237 не ответит и конфигурация не
  // прочитается обратно как 640 Гц, PGA=128, канал A.
  while (!cs1237_init_640_hz()) {
    delay(100);
  }
}

void loop() {
  int32_t rawValueR1;
  if (cs1237_read(rawValueR1)) {
    // Отбрасываем младшие 8 бит 24-битного АЦП, сохраняя знак результата.
    const int16_t valueR1 = static_cast<int16_t>(rawValueR1 >> 8);
    const int16_t values[4] = {valueR1, 0, 0, 0};

    // Передаем кадр фиксированного размера: четыре int16_t, всего 8 байт.
    Serial.write(reinterpret_cast<const uint8_t *>(values), sizeof(values));
  }
}
