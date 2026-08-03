#include <Arduino.h>
/*
 * AstroNav ICM-45686 Data Verification Test (v10)
 * Target: Custom RP2350A
 * Bus: SPI0
 */

#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <math.h>

// ---------------------------------------------------------
// 1. PIN DEFINITIONS
// ---------------------------------------------------------
#define PIN_MISO       4
#define PIN_SCK        6
#define PIN_MOSI       7
#define PIN_CS_IMU     9
#define PIN_CS_BMP     12
#define PIN_PYRO       1
#define PIN_SERVO_1    2
#define PIN_SERVO_2    3
#define PIN_VIN_SENSE  27
#define WS2812_PIN     0

#define ICM45686_EXPECTED_ID   0xE9

// ICM-45686 UI Registers from the public driver path
#define ICM_REG_ACCEL_DATA_X1_UI 0x00
#define ICM_REG_GYRO_DATA_X1_UI   0x06
#define ICM_REG_TEMP_DATA1_UI     0x0C
#define ICM_REG_PWR_MGMT0         0x10
#define ICM_REG_INT1_STATUS0      0x19
#define ICM_REG_INT1_STATUS1      0x1A
#define ICM_REG_ACCEL_CONFIG0     0x1B
#define ICM_REG_GYRO_CONFIG0      0x1C
#define ICM_REG_REG_MISC2         0x7F

// The current register stream maps the stationary 1 g vector to about half scale.
static constexpr float ACCEL_LSB_PER_G = 1024.0f;
static constexpr float GYRO_LSB_PER_DPS = 16.4f;
static constexpr float ADC_VREF = 3.3f;
static constexpr float ADC_COUNTS = 4095.0f;
static constexpr float VIN_DIVIDER_TOP_OHMS = 20000.0f;
static constexpr float VIN_DIVIDER_BOTTOM_OHMS = 10000.0f;
static constexpr float VIN_DIVIDER_RATIO =
  (VIN_DIVIDER_TOP_OHMS + VIN_DIVIDER_BOTTOM_OHMS) / VIN_DIVIDER_BOTTOM_OHMS;

struct ImuSample {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
};

Adafruit_NeoPixel led(1, WS2812_PIN, NEO_GRB + NEO_KHZ800);
BMP581 baro;
SPISettings spiSettings(500000, MSBFIRST, SPI_MODE3);
bool imuInitialized = false;

void setStatusLED(uint8_t r, uint8_t g, uint8_t b);
uint8_t readRegister(uint8_t csPin, uint8_t regAddr, SPISettings settings);
uint8_t readRegister(uint8_t csPin, uint8_t regAddr);
void writeRegister(uint8_t csPin, uint8_t regAddr, uint8_t value, SPISettings settings);
void writeRegister(uint8_t csPin, uint8_t regAddr, uint8_t value);
ImuSample decodeSample(const uint8_t *rawData);

enum class LedState {
  Busy,
  Healthy,
  Warning,
  Critical
};

struct BoardHealth {
  bool coreTickOk = false;
  bool flashOk = false;
  bool heapOk = false;
  bool spiOk = false;
  bool imuWhoAmIOk = false;
  bool imuConfigOk = false;
  bool imuStreamOk = false;
  bool baroOk = false;
  bool warning = false;
  bool critical = false;
};

BoardHealth boardHealth;
uint32_t lastSummaryMs = 0;
uint32_t bootWhiteUntilMs = 0;
float currentAx = 0.0f;
float currentAy = 0.0f;
float currentAz = 0.0f;
float currentGx = 0.0f;
float currentGy = 0.0f;
float currentGz = 0.0f;
float currentTemperature = 0.0f;
float currentPressure = 0.0f;
float currentVinVoltage = 0.0f;
const char *currentPowerSource = "Unknown";

const char kFlashSignature[] = "ASTRONAV_PCB_TEST";

void setLedState(LedState state) {
  switch (state) {
    case LedState::Busy:
      setStatusLED(255, 255, 255);
      break;
    case LedState::Healthy:
      setStatusLED(0, 255, 0);
      break;
    case LedState::Warning:
      setStatusLED(255, 140, 0);
      break;
    case LedState::Critical:
    default:
      setStatusLED(255, 0, 0);
      break;
  }
}

const char *boolText(bool value) {
  return value ? "OK" : "FAIL";
}

bool testCoreTick() {
  uint32_t start = millis();
  delay(2);
  return millis() > start;
}

bool testFlashRead() {
  uint8_t checksum = 0;
  for (size_t i = 0; i < sizeof(kFlashSignature) - 1; i++) {
    checksum ^= static_cast<uint8_t>(kFlashSignature[i]);
  }
  return checksum != 0;
}

bool testHeap() {
  const size_t testSize = 256;
  uint8_t *buffer = static_cast<uint8_t *>(malloc(testSize));
  if (buffer == nullptr) {
    return false;
  }

  for (size_t i = 0; i < testSize; i++) {
    buffer[i] = static_cast<uint8_t>(i ^ 0x5A);
  }

  bool ok = buffer[0] == 0x5A && buffer[1] == 0x5B && buffer[255] == static_cast<uint8_t>(255 ^ 0x5A);
  free(buffer);
  return ok;
}

bool readInputVoltage(float &voltage) {
  const int samples = 8;
  uint32_t total = 0;

  for (int i = 0; i < samples; i++) {
    total += static_cast<uint32_t>(analogRead(PIN_VIN_SENSE));
    delay(1);
  }

  float averageCounts = static_cast<float>(total) / static_cast<float>(samples);
  float senseVoltage = averageCounts * ADC_VREF / ADC_COUNTS;
  voltage = senseVoltage * VIN_DIVIDER_RATIO;
  return voltage > 0.1f;
}

const char *classifyPowerSource(float voltage) {
  if (voltage >= 4.5f && voltage <= 5.5f) {
    return "USB power";
  }
  if (voltage >= 3.0f && voltage <= 4.35f) {
    return "1S LiPo";
  }
  if (voltage >= 6.0f && voltage <= 8.4f) {
    return "2S LiPo";
  }
  return "Unknown";
}

bool readBaroSample(float &temperature, float &pressureHpa) {
  bmp5_sensor_data data = {0};
  if (baro.getSensorData(&data) != BMP5_OK) {
    return false;
  }

  temperature = data.temperature;
  pressureHpa = data.pressure / 100.0f;
  return pressureHpa > 300.0f && pressureHpa < 1200.0f;
}

bool readImuFrame(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  uint8_t rawData[12] = {0};

  for (int attempt = 0; attempt < 4; attempt++) {
    digitalWrite(PIN_CS_IMU, LOW);
    SPI.beginTransaction(spiSettings);
    SPI.transfer(ICM_REG_ACCEL_DATA_X1_UI | 0x80);
    for (int i = 0; i < 12; i++) {
      rawData[i] = SPI.transfer(0x00);
    }
    SPI.endTransaction();
    digitalWrite(PIN_CS_IMU, HIGH);

    bool allZero = true;
    bool allFF = true;
    for (int i = 0; i < 12; i++) {
      if (rawData[i] != 0x00) {
        allZero = false;
      }
      if (rawData[i] != 0xFF) {
        allFF = false;
      }
    }

    if (!allZero && !allFF) {
      ImuSample sample = decodeSample(rawData);
      ax = (float)sample.ax / ACCEL_LSB_PER_G;
      ay = (float)sample.ay / ACCEL_LSB_PER_G;
      az = (float)sample.az / ACCEL_LSB_PER_G;
      gx = (float)sample.gx / GYRO_LSB_PER_DPS;
      gy = (float)sample.gy / GYRO_LSB_PER_DPS;
      gz = (float)sample.gz / GYRO_LSB_PER_DPS;
      return true;
    }

    delay(2);
  }

  return false;
}

bool verifyImuStream() {
  const int samples = 8;
  int validSamples = 0;
  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

  for (int i = 0; i < samples; i++) {
    if (readImuFrame(ax, ay, az, gx, gy, gz)) {
      if (isfinite(ax) && isfinite(ay) && isfinite(az) && isfinite(gx) && isfinite(gy) && isfinite(gz)) {
        validSamples++;
      }
    }
    delay(2);
  }

  return validSamples >= 6;
}

bool configureImu() {
  bool transportOk = false;
  uint8_t whoAmI = 0;

  for (size_t i = 0; i < 2; i++) {
    uint8_t value = readRegister(PIN_CS_IMU, 0x72, (i == 0)
      ? SPISettings(500000, MSBFIRST, SPI_MODE3)
      : SPISettings(500000, MSBFIRST, SPI_MODE0));
    if (value != 0x00 && value != 0xFF) {
      spiSettings = (i == 0)
        ? SPISettings(500000, MSBFIRST, SPI_MODE3)
        : SPISettings(500000, MSBFIRST, SPI_MODE0);
      whoAmI = value;
      transportOk = true;
      break;
    }
  }

  boardHealth.spiOk = transportOk;
  boardHealth.imuWhoAmIOk = transportOk && (whoAmI == ICM45686_EXPECTED_ID);

  if (!boardHealth.imuWhoAmIOk) {
    return false;
  }

  writeRegister(PIN_CS_IMU, ICM_REG_REG_MISC2, 0x02, spiSettings);
  delay(5);
  writeRegister(PIN_CS_IMU, ICM_REG_PWR_MGMT0, 0x0F, spiSettings);
  writeRegister(PIN_CS_IMU, ICM_REG_ACCEL_CONFIG0, 0x06, spiSettings);
  writeRegister(PIN_CS_IMU, ICM_REG_GYRO_CONFIG0, 0x06, spiSettings);

  boardHealth.imuConfigOk =
    readRegister(PIN_CS_IMU, ICM_REG_PWR_MGMT0, spiSettings) == 0x0F &&
    readRegister(PIN_CS_IMU, ICM_REG_ACCEL_CONFIG0, spiSettings) == 0x06 &&
    readRegister(PIN_CS_IMU, ICM_REG_GYRO_CONFIG0, spiSettings) == 0x06;

  return boardHealth.imuConfigOk;
}

void updateHealthLed() {
  if (millis() < bootWhiteUntilMs) {
    setLedState(LedState::Busy);
    return;
  }

  if (boardHealth.critical) {
    setLedState(LedState::Critical);
  } else if (boardHealth.warning) {
    setLedState(LedState::Warning);
  } else {
    setLedState(LedState::Healthy);
  }
}

void printBootResult() {
  const bool healthy = boardHealth.coreTickOk && boardHealth.flashOk && boardHealth.heapOk &&
                       boardHealth.spiOk && boardHealth.imuWhoAmIOk && boardHealth.imuConfigOk &&
                       boardHealth.imuStreamOk;
  const bool hasWarning = !healthy && !boardHealth.critical;

  Serial.printf("[TEST] core tick   : %s\n", boolText(boardHealth.coreTickOk));
  Serial.printf("[TEST] flash read  : %s\n", boolText(boardHealth.flashOk));
  Serial.printf("[TEST] heap alloc  : %s\n", boolText(boardHealth.heapOk));
  Serial.printf("[TEST] IMU transport: %s\n", boolText(boardHealth.spiOk));
  Serial.printf("[TEST] IMU WHO_AM_I : %s\n", boolText(boardHealth.imuWhoAmIOk));
  Serial.printf("[TEST] IMU config   : %s\n", boolText(boardHealth.imuConfigOk));
  Serial.printf("[TEST] IMU stream   : %s\n", boolText(boardHealth.imuStreamOk));
  Serial.printf("[TEST] barometer    : %s\n", boolText(boardHealth.baroOk));
  Serial.printf("[TEST] VIN sense    : %.3f V (%s)\n", currentVinVoltage, currentPowerSource);
  Serial.printf("[TEST] result       : %s\n", healthy ? "PASS" : (boardHealth.critical ? "FAIL" : "WARN"));

  if (millis() < bootWhiteUntilMs) {
    setLedState(LedState::Busy);
  } else if (healthy) {
    setLedState(LedState::Healthy);
  } else if (hasWarning) {
    setLedState(LedState::Warning);
  } else {
    setLedState(LedState::Critical);
  }
}

void printRuntimeSummary() {
  const bool healthy = boardHealth.coreTickOk && boardHealth.flashOk && boardHealth.heapOk &&
                       boardHealth.spiOk && boardHealth.imuWhoAmIOk && boardHealth.imuConfigOk &&
                       boardHealth.imuStreamOk && boardHealth.baroOk && !boardHealth.critical;

  Serial.printf("[PCB] %s | core:%s flash:%s heap:%s imu:%s baro:%s vin:%.3fV(%s) | accel=%.3f,%.3f,%.3f g gyro=%.3f,%.3f,%.3f dps temp=%.1f C pressure=%.2f hPa\n",
                healthy ? "OK" : (boardHealth.critical ? "FAIL" : "WARN"),
                boolText(boardHealth.coreTickOk),
                boolText(boardHealth.flashOk),
                boolText(boardHealth.heapOk),
                boolText(boardHealth.imuWhoAmIOk && boardHealth.imuConfigOk && boardHealth.imuStreamOk),
                boolText(boardHealth.baroOk),
                currentVinVoltage,
                currentPowerSource,
                currentAx, currentAy, currentAz,
                currentGx, currentGy, currentGz,
                currentTemperature, currentPressure);
}

void setStatusLED(uint8_t r, uint8_t g, uint8_t b) {
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}

uint8_t readRegister(uint8_t csPin, uint8_t regAddr, SPISettings settings) {
  uint8_t value = 0;
  digitalWrite(csPin, LOW);
  SPI.beginTransaction(settings);
  SPI.transfer(regAddr | 0x80);
  value = SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(csPin, HIGH);
  return value;
}

uint8_t readRegister(uint8_t csPin, uint8_t regAddr) {
  return readRegister(csPin, regAddr, spiSettings);
}

void writeRegister(uint8_t csPin, uint8_t regAddr, uint8_t value, SPISettings settings) {
  digitalWrite(csPin, LOW);
  SPI.beginTransaction(settings);
  SPI.transfer(regAddr & 0x7F);
  SPI.transfer(value);
  SPI.endTransaction();
  digitalWrite(csPin, HIGH);
}

void writeRegister(uint8_t csPin, uint8_t regAddr, uint8_t value) {
  writeRegister(csPin, regAddr, value, spiSettings);
}

ImuSample decodeSample(const uint8_t *rawData) {
  ImuSample sample;
  sample.ax = (int16_t)((rawData[1] << 8) | rawData[0]);
  sample.ay = (int16_t)((rawData[3] << 8) | rawData[2]);
  sample.az = (int16_t)((rawData[5] << 8) | rawData[4]);
  sample.gx = (int16_t)((rawData[7] << 8) | rawData[6]);
  sample.gy = (int16_t)((rawData[9] << 8) | rawData[8]);
  sample.gz = (int16_t)((rawData[11] << 8) | rawData[10]);
  return sample;
}

// ---------------------------------------------------------
// 2. SETUP
// ---------------------------------------------------------
void setup() {
  Serial.begin(115200);
  led.begin();
  led.setBrightness(40);
  setLedState(LedState::Busy);
  bootWhiteUntilMs = millis() + 2000;
  delay(400);
  analogReadResolution(12);

  uint32_t startWait = millis();
  while (!Serial && (millis() - startWait < 3000)) { delay(10); }

  Serial.println("\n=============================================");
  Serial.println("  AstroNav PCB Health Test v11              ");
  Serial.println("=============================================");

  pinMode(PIN_PYRO, OUTPUT);
  pinMode(PIN_SERVO_1, OUTPUT);
  pinMode(PIN_SERVO_2, OUTPUT);
  digitalWrite(PIN_PYRO, LOW);
  digitalWrite(PIN_SERVO_1, LOW);
  digitalWrite(PIN_SERVO_2, LOW);

  pinMode(PIN_CS_IMU, OUTPUT);
  pinMode(PIN_CS_BMP, OUTPUT);
  pinMode(PIN_VIN_SENSE, INPUT);
  digitalWrite(PIN_CS_IMU, HIGH);
  digitalWrite(PIN_CS_BMP, HIGH);

  // Force BMP581 out of I2C into SPI mode
  digitalWrite(PIN_CS_BMP, LOW);  delay(15);
  digitalWrite(PIN_CS_BMP, HIGH); delay(15);

  SPI.setRX(PIN_MISO);
  SPI.setSCK(PIN_SCK);
  SPI.setTX(PIN_MOSI);
  SPI.begin();

  boardHealth.coreTickOk = testCoreTick();
  boardHealth.flashOk = testFlashRead();
  boardHealth.heapOk = testHeap();

  if (readInputVoltage(currentVinVoltage)) {
    currentPowerSource = classifyPowerSource(currentVinVoltage);
    if (strcmp(currentPowerSource, "Unknown") == 0) {
      boardHealth.warning = true;
    }
  }

  baro.beginSPI(PIN_CS_BMP, 250000);
  delay(30);
  boardHealth.baroOk = readBaroSample(currentTemperature, currentPressure);

  bool imuBootOk = configureImu();
  imuInitialized = boardHealth.spiOk && boardHealth.imuWhoAmIOk && boardHealth.imuConfigOk;
  if (imuBootOk) {
    boardHealth.imuStreamOk = verifyImuStream();
    readImuFrame(currentAx, currentAy, currentAz, currentGx, currentGy, currentGz);
  }

  boardHealth.warning = !boardHealth.baroOk || !boardHealth.imuStreamOk;
  if (strcmp(currentPowerSource, "Unknown") == 0) {
    boardHealth.warning = true;
  }
  boardHealth.critical = !boardHealth.coreTickOk || !boardHealth.flashOk || !boardHealth.heapOk ||
                         !boardHealth.spiOk || !boardHealth.imuWhoAmIOk || !boardHealth.imuConfigOk ||
                         !boardHealth.imuStreamOk;

  printBootResult();
  delay(500);
  updateHealthLed();
  lastSummaryMs = millis();
  Serial.println("=============================================\n");
}

// ---------------------------------------------------------
// 3. MAIN LOOP
// ---------------------------------------------------------
void loop() {
  if (readInputVoltage(currentVinVoltage)) {
    currentPowerSource = classifyPowerSource(currentVinVoltage);
    boardHealth.warning = strcmp(currentPowerSource, "Unknown") == 0;
  }

  if (imuInitialized) {
    float ax = 0.0f, ay = 0.0f, az = 0.0f, gx = 0.0f, gy = 0.0f, gz = 0.0f;
    if (readImuFrame(ax, ay, az, gx, gy, gz)) {
      currentAx = ax;
      currentAy = ay;
      currentAz = az;
      currentGx = gx;
      currentGy = gy;
      currentGz = gz;
      boardHealth.imuStreamOk = true;
      boardHealth.critical = !boardHealth.coreTickOk || !boardHealth.flashOk || !boardHealth.heapOk ||
                             !boardHealth.spiOk || !boardHealth.imuWhoAmIOk || !boardHealth.imuConfigOk;
    } else {
      boardHealth.imuStreamOk = false;
      boardHealth.warning = true;
    }
  }

  if (readBaroSample(currentTemperature, currentPressure)) {
    boardHealth.baroOk = true;
  } else {
    boardHealth.baroOk = false;
    boardHealth.warning = true;
  }

  boardHealth.warning = !boardHealth.critical && (!boardHealth.baroOk || !boardHealth.imuStreamOk);
  boardHealth.critical = !boardHealth.coreTickOk || !boardHealth.flashOk || !boardHealth.heapOk ||
                         !boardHealth.spiOk || !boardHealth.imuWhoAmIOk || !boardHealth.imuConfigOk;

  updateHealthLed();

  if (millis() - lastSummaryMs >= 1000) {
    printRuntimeSummary();
    lastSummaryMs = millis();
  }

  delay(10);
}