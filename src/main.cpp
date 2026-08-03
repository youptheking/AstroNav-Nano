#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <FatFS.h>
#include <FatFSUSB.h>
#include <ctype.h>
#include <math.h>
#include <string.h>

/*
 * AstroNav flight firmware for RP2350
 * - Startup health check with LED status
 * - Pad orientation calibration
 * - Launch and apogee detection
 * - Safe pyro pulse and lockout
 * - RAM mission log flushed to flash after landing
 * - USB mass-storage mode when powered from USB
 */

// ---------------------------------------------------------
// 1. PIN DEFINITIONS
// ---------------------------------------------------------
#define PIN_MISO       4
#define PIN_SCK        6
#define PIN_MOSI       7
#define PIN_CS_IMU     9
#define PIN_CS_BMP    12
#define PIN_PYRO       1
#define PIN_SERVO_1    2
#define PIN_SERVO_2    3
#define PIN_VIN_SENSE 27
#define WS2812_PIN     0

#define ICM45686_EXPECTED_ID   0xE9

// ICM-45686 UI registers
#define ICM_REG_ACCEL_DATA_X1_UI 0x00
#define ICM_REG_GYRO_DATA_X1_UI  0x06
#define ICM_REG_TEMP_DATA1_UI    0x0C
#define ICM_REG_PWR_MGMT0        0x10
#define ICM_REG_ACCEL_CONFIG0    0x1B
#define ICM_REG_GYRO_CONFIG0     0x1C
#define ICM_REG_REG_MISC2        0x7F

static constexpr float ACCEL_LSB_PER_G = 1024.0f;
static constexpr float GYRO_LSB_PER_DPS = 16.4f;
static constexpr float ADC_VREF = 3.3f;
static constexpr float ADC_COUNTS = 4095.0f;
static constexpr float VIN_DIVIDER_TOP_OHMS = 20000.0f;
static constexpr float VIN_DIVIDER_BOTTOM_OHMS = 10000.0f;
static constexpr float VIN_DIVIDER_RATIO =
  (VIN_DIVIDER_TOP_OHMS + VIN_DIVIDER_BOTTOM_OHMS) / VIN_DIVIDER_BOTTOM_OHMS;

static constexpr uint32_t SAMPLE_PERIOD_MS = 100;
static constexpr uint32_t LED_PERIOD_MS = 25;
static constexpr uint32_t BOOT_PULSE_MS = 2000;
static constexpr uint32_t CALIBRATION_TIMEOUT_MS = 6000;
static constexpr uint8_t CALIBRATION_GOOD_SAMPLES = 20;
static constexpr float STATIONARY_ACCEL_TOLERANCE_G = 0.08f;
static constexpr float STATIONARY_GYRO_TOLERANCE_DPS = 20.0f;
static constexpr float LAUNCH_THRESHOLD_G = 1.35f;
static constexpr uint8_t LAUNCH_CONFIRM_SAMPLES = 3;
static constexpr uint32_t LAUNCH_MIN_TIME_MS = 400;
static constexpr float APOGEE_DROP_M = 0.25f;
static constexpr float APOGEE_VELOCITY_THRESHOLD_MPS = -0.25f;
static constexpr float APOGEE_ACCEL_MAX_G = 0.90f;
static constexpr uint8_t APOGEE_CONFIRM_SAMPLES = 3;
static constexpr float LANDING_ALTITUDE_TOLERANCE_M = 2.0f;
static constexpr float LANDING_VELOCITY_TOLERANCE_MPS = 0.25f;
static constexpr uint8_t LANDING_CONFIRM_SAMPLES = 15;
static constexpr uint32_t PYRO_PULSE_MS = 350;
static constexpr uint32_t MAX_FLIGHT_TIME_MS = 45000;
static constexpr uint16_t FLIGHT_LOG_CAPACITY = 2048;
static constexpr uint32_t PROFILE_MAGIC = 0x50455246;   // "FRFP"
static constexpr uint32_t PROFILE_VERSION = 1;

struct ImuSample {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
};

enum class PowerMode {
  Unknown,
  USB,
  LiPo1S,
  LiPo2S
};

enum class FlightState {
  Booting,
  Calibrating,
  Idle,
  UsbMode,
  Armed,
  Boost,
  Coast,
  PyroFired,
  Landed,
  Fault
};

enum class LedProfile {
  Boot,
  Idle,
  IdleUsb,
  Idle1S,
  Idle2S,
  Coast,
  Warning,
  Landed,
  Fault
};

struct BoardHealth {
  bool coreTickOk = false;
  bool heapOk = false;
  bool spiOk = false;
  bool imuWhoAmIOk = false;
  bool imuConfigOk = false;
  bool imuStreamOk = false;
  bool baroOk = false;
  bool flashFsOk = false;
  bool usbStorageOk = false;
  bool vinOk = false;
  bool warning = false;
  bool critical = false;
};

struct FlightSample {
  uint32_t counter = 0;
  uint32_t ms = 0;
  int16_t axMg = 0;
  int16_t ayMg = 0;
  int16_t azMg = 0;
  int16_t gxDps10 = 0;
  int16_t gyDps10 = 0;
  int16_t gzDps10 = 0;
  int16_t tempCd10 = 0;
  uint16_t pressureHd10 = 0;
  int16_t altitudeCm = 0;
  int16_t velocityCms = 0;
  int16_t rollD10 = 0;
  int16_t pitchD10 = 0;
  uint8_t healthBits = 0;
  uint8_t state = 0;
};

struct FlightSettings {
  float estimatedHeightM = 120.0f;
  float estimatedSpeedMps = 35.0f;
  float heightMarginM = 20.0f;
  float speedMarginMps = 8.0f;
  float launchThresholdG = 1.35f;
  bool usbExit = false;
};

struct DeviceProfile {
  uint32_t magic = PROFILE_MAGIC;
  uint32_t version = PROFILE_VERSION;
  char serialNumber[20] = "AN-0001";
  char flashStamp[24] = "";
  uint32_t flightCount = 0;
  uint32_t logCount = 0;
  float maxFlightAltitudeM = 0.0f;
  float maxFlightSpeedMps = 0.0f;
  float lastFlightAltitudeM = 0.0f;
  float lastFlightSpeedMps = 0.0f;
  uint32_t checksum = 0;
};

Adafruit_NeoPixel led(1, WS2812_PIN, NEO_GRB + NEO_KHZ800);
BMP581 baro;
SPISettings spiSettings(500000, MSBFIRST, SPI_MODE3);

BoardHealth boardHealth;
PowerMode powerMode = PowerMode::Unknown;
FlightState flightState = FlightState::Booting;

bool imuInitialized = false;
bool storageReady = false;
bool usbDriveReady = false;
bool usbFlightOverride = false;
bool missionLogFlushed = false;
bool pyroLatched = false;
bool missionStarted = false;

uint32_t bootMs = 0;
uint32_t bootPulseUntilMs = 0;
uint32_t nextSampleMs = 0;
uint32_t nextLedMs = 0;
uint32_t launchMs = 0;
uint32_t pyroPulseUntilMs = 0;
uint32_t lastLogMs = 0;
char serialCommandBuffer[48] = {0};
uint8_t serialCommandLength = 0;

float currentVinVoltage = 0.0f;
float currentAx = 0.0f;
float currentAy = 0.0f;
float currentAz = 0.0f;
float currentGx = 0.0f;
float currentGy = 0.0f;
float currentGz = 0.0f;
float currentTemperature = 0.0f;
float currentPressure = 0.0f;
float currentAltitude = 0.0f;
float currentVerticalVelocity = 0.0f;
float currentRollDeg = 0.0f;
float currentPitchDeg = 0.0f;
float baselinePressureHpa = 0.0f;
float filteredTemperature = 0.0f;
float filteredPressure = 0.0f;
float filteredAltitude = 0.0f;
float peakAltitude = 0.0f;
float gravityAxisX = 0.0f;
float gravityAxisY = 0.0f;
float gravityAxisZ = 1.0f;
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;
float lastAltitudeForVelocity = 0.0f;

uint8_t launchConfirmCount = 0;
uint8_t apogeeConfirmCount = 0;
uint8_t landingConfirmCount = 0;
uint32_t logCounter = 0;
float peakFlightSpeedMps = 0.0f;

float flightLaunchThresholdG = 1.35f;
float flightApogeeDropM = 0.25f;
float flightApogeeVelocityThresholdMPS = -0.25f;
float flightLandingAltitudeToleranceM = 2.0f;
float flightLandingVelocityToleranceMPS = 0.25f;

FlightSettings flightSettings;
DeviceProfile deviceProfile;

FlightSample missionLog[FLIGHT_LOG_CAPACITY];
uint16_t missionLogCount = 0;

char missionLogPath[48] = {0};

void setStatusLED(uint8_t r, uint8_t g, uint8_t b);
void setLedProfile(LedProfile profile);
void updateStatusLed();
float clampFloat(float value, float minimum, float maximum);
void appendMissionSample();
void flushMissionLogToFlash();
void firePyro();
void updatePyroOutput();
void updateFlightStateFromSamples();
void recordFaultAndSafeStop();
void startMissionLog();
void ensureLogsDirectory();
void handleSerialCommand(const char *command);
void enterFlightModeFromUsb();
void ensureUsbDemoFile();
bool shouldExitUsbModeFromDemoFile();
bool loadFlightSettings();
bool loadOrCreateDeviceProfile();
void persistDeviceProfile();
void writePersonalFile();
bool systemHealthy();

uint8_t readRegister(uint8_t csPin, uint8_t regAddr, SPISettings settings);
uint8_t readRegister(uint8_t csPin, uint8_t regAddr);
void writeRegister(uint8_t csPin, uint8_t regAddr, uint8_t value, SPISettings settings);
void writeRegister(uint8_t csPin, uint8_t regAddr, uint8_t value);
ImuSample decodeSample(const uint8_t *rawData);

const char *boolText(bool value) {
  return value ? "OK" : "FAIL";
}

const char *stateText(FlightState state) {
  switch (state) {
    case FlightState::Booting: return "boot";
    case FlightState::Calibrating: return "calibrating";
    case FlightState::Idle: return "idle";
    case FlightState::UsbMode: return "usb mode";
    case FlightState::Armed: return "armed";
    case FlightState::Boost: return "boost";
    case FlightState::Coast: return "coast";
    case FlightState::PyroFired: return "pyro";
    case FlightState::Landed: return "landed";
    case FlightState::Fault:
    default: return "fault";
  }
}

const char *powerModeText(PowerMode mode) {
  switch (mode) {
    case PowerMode::USB: return "USB";
    case PowerMode::LiPo1S: return "1S LiPo";
    case PowerMode::LiPo2S: return "2S LiPo";
    case PowerMode::Unknown:
    default: return "Unknown";
  }
}

const char *runtimeModeText() {
  if (powerMode == PowerMode::USB && !usbFlightOverride) {
    return "usb";
  }

  return "flight";
}

void buildFlashStamp(char *buffer, size_t bufferSize) {
  static const char *months[] = {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
  };

  int day = 1;
  int year = 2026;
  int monthIndex = 7;
  char monthText[4] = {0};

  if (sscanf(__DATE__, "%3s %d %d", monthText, &day, &year) == 3) {
    for (int i = 0; i < 12; i++) {
      if (strcmp(monthText, months[i]) == 0) {
        monthIndex = i;
        break;
      }
    }
  }

  snprintf(buffer, bufferSize, "%02d-%02d-%04d %s",
           day,
           monthIndex + 1,
           year,
           __TIME__);
}

uint32_t checksumBytes(const uint8_t *data, size_t length) {
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < length; i++) {
    hash ^= data[i];
    hash *= 16777619u;
  }
  return hash;
}

void applyFlightSettings() {
  float heightMargin = clampFloat(flightSettings.heightMarginM, 0.0f, 1000.0f);
  float speedMargin = clampFloat(flightSettings.speedMarginMps, 0.0f, 1000.0f);

  flightLaunchThresholdG = clampFloat(flightSettings.launchThresholdG, 1.15f, 4.0f);
  flightApogeeDropM = clampFloat((flightSettings.estimatedHeightM * 0.08f) + (heightMargin * 0.25f), 0.25f, 50.0f);
  flightApogeeVelocityThresholdMPS = -clampFloat((flightSettings.estimatedSpeedMps * 0.05f) + (speedMargin * 0.15f), 0.25f, 12.0f);
  flightLandingAltitudeToleranceM = clampFloat(2.0f + (heightMargin * 0.05f), 2.0f, 25.0f);
  flightLandingVelocityToleranceMPS = clampFloat(0.25f + (speedMargin * 0.05f), 0.25f, 5.0f);
}

void makeDefaultFlightSettings() {
  flightSettings.estimatedHeightM = 120.0f;
  flightSettings.estimatedSpeedMps = 35.0f;
  flightSettings.heightMarginM = 20.0f;
  flightSettings.speedMarginMps = 8.0f;
  flightSettings.launchThresholdG = 1.35f;
  flightSettings.usbExit = false;
  applyFlightSettings();
}

void saveFlightSettings();

bool parseBoolValue(const char *text) {
  if (!text) {
    return false;
  }
  while (*text == ' ' || *text == '\t') {
    text++;
  }
  return (strcasecmp(text, "TRUE") == 0) || (strcasecmp(text, "1") == 0) ||
         (strcasecmp(text, "YES") == 0) || (strcasecmp(text, "ON") == 0);
}

bool readSettingLine(const char *line, const char *key, char *valueBuffer, size_t valueBufferSize) {
  size_t keyLength = strlen(key);
  if (strncmp(line, key, keyLength) != 0 || line[keyLength] != '=') {
    return false;
  }

  const char *valueStart = line + keyLength + 1;
  while (*valueStart == ' ' || *valueStart == '\t') {
    valueStart++;
  }

  strncpy(valueBuffer, valueStart, valueBufferSize - 1);
  valueBuffer[valueBufferSize - 1] = '\0';

  size_t valueLength = strlen(valueBuffer);
  while (valueLength > 0 && (valueBuffer[valueLength - 1] == '\r' || valueBuffer[valueLength - 1] == '\n' || valueBuffer[valueLength - 1] == ' ' || valueBuffer[valueLength - 1] == '\t')) {
    valueBuffer[valueLength - 1] = '\0';
    valueLength--;
  }

  return true;
}

bool loadFlightSettings() {
  if (!storageReady || !FatFS.exists("/SETTINGS.TXT")) {
    makeDefaultFlightSettings();
    return false;
  }

  File settingsFile = FatFS.open("/SETTINGS.TXT", "r");
  if (!settingsFile) {
    makeDefaultFlightSettings();
    return false;
  }

  makeDefaultFlightSettings();
  bool anyValue = false;
  char line[128];

  while (settingsFile.available()) {
    size_t length = settingsFile.readBytesUntil('\n', line, sizeof(line) - 1);
    line[length] = '\0';

    if (line[0] == '#' || line[0] == ';' || line[0] == '\0') {
      continue;
    }

    char value[64];
    if (readSettingLine(line, "ESTIMATED_HEIGHT_M", value, sizeof(value))) {
      flightSettings.estimatedHeightM = static_cast<float>(atof(value));
      anyValue = true;
    } else if (readSettingLine(line, "ESTIMATED_SPEED_MPS", value, sizeof(value))) {
      flightSettings.estimatedSpeedMps = static_cast<float>(atof(value));
      anyValue = true;
    } else if (readSettingLine(line, "HEIGHT_MARGIN_M", value, sizeof(value))) {
      flightSettings.heightMarginM = static_cast<float>(atof(value));
      anyValue = true;
    } else if (readSettingLine(line, "SPEED_MARGIN_MPS", value, sizeof(value))) {
      flightSettings.speedMarginMps = static_cast<float>(atof(value));
      anyValue = true;
    } else if (readSettingLine(line, "LAUNCH_THRESHOLD_G", value, sizeof(value))) {
      flightSettings.launchThresholdG = static_cast<float>(atof(value));
      anyValue = true;
    } else if (readSettingLine(line, "USB_EXIT", value, sizeof(value))) {
      flightSettings.usbExit = parseBoolValue(value);
      anyValue = true;
    }
  }

  settingsFile.close();
  applyFlightSettings();
  if (!anyValue) {
    saveFlightSettings();
  }
  return true;
}

void saveFlightSettings() {
  if (!storageReady) {
    return;
  }

  File settingsFile = FatFS.open("/SETTINGS.TXT", "w");
  if (!settingsFile) {
    boardHealth.warning = true;
    return;
  }

  settingsFile.println("# AstroNav Nano flight settings");
  settingsFile.println("# Edit the values below, then save the file.");
  settingsFile.printf("ESTIMATED_HEIGHT_M=%.1f\n", flightSettings.estimatedHeightM);
  settingsFile.printf("HEIGHT_MARGIN_M=%.1f\n", flightSettings.heightMarginM);
  settingsFile.printf("ESTIMATED_SPEED_MPS=%.1f\n", flightSettings.estimatedSpeedMps);
  settingsFile.printf("SPEED_MARGIN_MPS=%.1f\n", flightSettings.speedMarginMps);
  settingsFile.printf("LAUNCH_THRESHOLD_G=%.2f\n", flightSettings.launchThresholdG);
  settingsFile.printf("USB_EXIT=%s\n", flightSettings.usbExit ? "TRUE" : "FALSE");
  settingsFile.flush();
  settingsFile.close();
}

void makeDefaultDeviceProfile() {
  memset(&deviceProfile, 0, sizeof(deviceProfile));
  deviceProfile.magic = PROFILE_MAGIC;
  deviceProfile.version = PROFILE_VERSION;
  snprintf(deviceProfile.serialNumber, sizeof(deviceProfile.serialNumber), "AN-0001");
  buildFlashStamp(deviceProfile.flashStamp, sizeof(deviceProfile.flashStamp));
}

void updateDeviceProfileChecksum() {
  deviceProfile.checksum = 0;
  deviceProfile.checksum = checksumBytes(reinterpret_cast<const uint8_t *>(&deviceProfile), sizeof(deviceProfile) - sizeof(deviceProfile.checksum));
}

bool loadOrCreateDeviceProfile() {
  if (!storageReady || !FatFS.exists("/PROFILE.BIN")) {
    makeDefaultDeviceProfile();
    updateDeviceProfileChecksum();
    return false;
  }

  File profileFile = FatFS.open("/PROFILE.BIN", "r");
  if (!profileFile) {
    makeDefaultDeviceProfile();
    updateDeviceProfileChecksum();
    return false;
  }

  if (profileFile.read(reinterpret_cast<uint8_t *>(&deviceProfile), sizeof(deviceProfile)) != sizeof(deviceProfile)) {
    profileFile.close();
    makeDefaultDeviceProfile();
    updateDeviceProfileChecksum();
    return false;
  }
  profileFile.close();

  uint32_t expectedChecksum = deviceProfile.checksum;
  uint32_t savedChecksum = deviceProfile.checksum;
  deviceProfile.checksum = 0;
  if (deviceProfile.magic != PROFILE_MAGIC || deviceProfile.version != PROFILE_VERSION) {
    makeDefaultDeviceProfile();
    updateDeviceProfileChecksum();
    return false;
  }

  uint32_t computedChecksum = checksumBytes(reinterpret_cast<const uint8_t *>(&deviceProfile), sizeof(deviceProfile) - sizeof(deviceProfile.checksum));
  if (computedChecksum != expectedChecksum || savedChecksum == 0) {
    makeDefaultDeviceProfile();
    updateDeviceProfileChecksum();
    return false;
  }

  deviceProfile.checksum = expectedChecksum;
  return true;
}

void persistDeviceProfile() {
  if (!storageReady) {
    return;
  }

  updateDeviceProfileChecksum();
  File profileFile = FatFS.open("/PROFILE.BIN", "w");
  if (!profileFile) {
    boardHealth.warning = true;
    return;
  }

  profileFile.write(reinterpret_cast<const uint8_t *>(&deviceProfile), sizeof(deviceProfile));
  profileFile.flush();
  profileFile.close();
}

void writePersonalFile() {
  if (!storageReady) {
    return;
  }

  File personalFile = FatFS.open("/PERSONAL.TXT", "w");
  if (!personalFile) {
    boardHealth.warning = true;
    return;
  }

  personalFile.println("AstroNav Nano Personal Data");
  personalFile.println("This is a text copy of the flash-backed profile data.");
  personalFile.println();
  personalFile.printf("FlashStamp=%s\n", deviceProfile.flashStamp);
  personalFile.printf("SerialNumber=%s\n", deviceProfile.serialNumber);
  personalFile.printf("FlightCount=%lu\n", static_cast<unsigned long>(deviceProfile.flightCount));
  personalFile.printf("LogCount=%lu\n", static_cast<unsigned long>(deviceProfile.logCount));
  personalFile.printf("MaxFlightAltitudeM=%.1f\n", deviceProfile.maxFlightAltitudeM);
  personalFile.printf("MaxFlightSpeedMps=%.1f\n", deviceProfile.maxFlightSpeedMps);
  personalFile.printf("LastFlightAltitudeM=%.1f\n", deviceProfile.lastFlightAltitudeM);
  personalFile.printf("LastFlightSpeedMps=%.1f\n", deviceProfile.lastFlightSpeedMps);
  personalFile.printf("ProfileChecksum=%08lX\n", static_cast<unsigned long>(deviceProfile.checksum));
  personalFile.flush();
  personalFile.close();
}

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

PowerMode classifyPowerSource(float voltage) {
  if (voltage >= 4.5f && voltage <= 5.5f) {
    return PowerMode::USB;
  }
  if (voltage >= 3.0f && voltage <= 4.35f) {
    return PowerMode::LiPo1S;
  }
  if (voltage >= 6.0f && voltage <= 8.4f) {
    return PowerMode::LiPo2S;
  }
  return PowerMode::Unknown;
}

bool systemHealthy() {
  return boardHealth.coreTickOk && boardHealth.heapOk && boardHealth.spiOk &&
         boardHealth.imuWhoAmIOk && boardHealth.imuConfigOk && boardHealth.imuStreamOk &&
         boardHealth.baroOk && !boardHealth.critical;
}

bool testCoreTick() {
  uint32_t start = millis();
  delay(2);
  return millis() > start;
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
      ax = static_cast<float>(sample.ax) / ACCEL_LSB_PER_G;
      ay = static_cast<float>(sample.ay) / ACCEL_LSB_PER_G;
      az = static_cast<float>(sample.az) / ACCEL_LSB_PER_G;
      gx = static_cast<float>(sample.gx) / GYRO_LSB_PER_DPS;
      gy = static_cast<float>(sample.gy) / GYRO_LSB_PER_DPS;
      gz = static_cast<float>(sample.gz) / GYRO_LSB_PER_DPS;
      return true;
    }

    delay(2);
  }

  return false;
}

bool verifyImuStream() {
  const int samples = 8;
  int validSamples = 0;
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;

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

float magnitude3(float x, float y, float z) {
  return sqrtf((x * x) + (y * y) + (z * z));
}

bool calibratePadOrientation() {
  uint32_t deadline = millis() + CALIBRATION_TIMEOUT_MS;
  uint8_t stableCount = 0;
  float accelSumX = 0.0f;
  float accelSumY = 0.0f;
  float accelSumZ = 0.0f;
  float gyroSumX = 0.0f;
  float gyroSumY = 0.0f;
  float gyroSumZ = 0.0f;
  float temperatureSum = 0.0f;
  float pressureSum = 0.0f;

  while (millis() < deadline) {
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;
    float temp = 0.0f;
    float pressure = 0.0f;

    bool imuOk = readImuFrame(ax, ay, az, gx, gy, gz);
    bool baroOk = readBaroSample(temp, pressure);
    if (!imuOk || !baroOk) {
      stableCount = 0;
      accelSumX = accelSumY = accelSumZ = 0.0f;
      gyroSumX = gyroSumY = gyroSumZ = 0.0f;
      temperatureSum = 0.0f;
      pressureSum = 0.0f;
      delay(40);
      continue;
    }

    float accelMag = magnitude3(ax, ay, az);
    float gyroMag = magnitude3(gx, gy, gz);
    if (fabsf(accelMag - 1.0f) <= STATIONARY_ACCEL_TOLERANCE_G &&
        gyroMag <= STATIONARY_GYRO_TOLERANCE_DPS) {
      accelSumX += ax;
      accelSumY += ay;
      accelSumZ += az;
      gyroSumX += gx;
      gyroSumY += gy;
      gyroSumZ += gz;
      temperatureSum += temp;
      pressureSum += pressure;
      stableCount++;

      if (stableCount >= CALIBRATION_GOOD_SAMPLES) {
        float invCount = 1.0f / static_cast<float>(stableCount);
        gravityAxisX = accelSumX * invCount;
        gravityAxisY = accelSumY * invCount;
        gravityAxisZ = accelSumZ * invCount;
        float gravityMag = magnitude3(gravityAxisX, gravityAxisY, gravityAxisZ);
        if (gravityMag > 0.01f) {
          gravityAxisX /= gravityMag;
          gravityAxisY /= gravityMag;
          gravityAxisZ /= gravityMag;
        } else {
          gravityAxisX = 0.0f;
          gravityAxisY = 0.0f;
          gravityAxisZ = 1.0f;
        }

        gyroBiasX = gyroSumX * invCount;
        gyroBiasY = gyroSumY * invCount;
        gyroBiasZ = gyroSumZ * invCount;
        filteredTemperature = temperatureSum * invCount;
        filteredPressure = pressureSum * invCount;
        baselinePressureHpa = filteredPressure;
        filteredAltitude = 0.0f;
        peakAltitude = 0.0f;
        lastAltitudeForVelocity = 0.0f;
        currentVerticalVelocity = 0.0f;
        return true;
      }
    } else {
      stableCount = 0;
      accelSumX = accelSumY = accelSumZ = 0.0f;
      gyroSumX = gyroSumY = gyroSumZ = 0.0f;
      temperatureSum = 0.0f;
      pressureSum = 0.0f;
    }

    delay(40);
  }

  return false;
}

float altitudeFromPressure(float pressureHpa) {
  if (baselinePressureHpa <= 0.0f || pressureHpa <= 0.0f) {
    return 0.0f;
  }
  return 44330.0f * (1.0f - powf(pressureHpa / baselinePressureHpa, 0.190294957f));
}

uint8_t healthBits() {
  uint8_t bits = 0;
  bits |= boardHealth.coreTickOk ? (1 << 0) : 0;
  bits |= boardHealth.heapOk ? (1 << 1) : 0;
  bits |= boardHealth.spiOk ? (1 << 2) : 0;
  bits |= boardHealth.imuWhoAmIOk ? (1 << 3) : 0;
  bits |= boardHealth.imuConfigOk ? (1 << 4) : 0;
  bits |= boardHealth.imuStreamOk ? (1 << 5) : 0;
  bits |= boardHealth.baroOk ? (1 << 6) : 0;
  bits |= boardHealth.flashFsOk ? (1 << 7) : 0;
  return bits;
}

void startMissionLog() {
  snprintf(missionLogPath, sizeof(missionLogPath), "/logs/flight_%08lu.csv", static_cast<unsigned long>(bootMs));
  missionLogCount = 0;
  missionStarted = true;
  missionLogFlushed = false;
  logCounter = 0;
}

void appendMissionSample() {
  if (!missionStarted) {
    startMissionLog();
  }

  if (missionLogCount >= FLIGHT_LOG_CAPACITY) {
    boardHealth.warning = true;
    return;
  }

  FlightSample &sample = missionLog[missionLogCount++];
  sample.counter = ++logCounter;
  sample.ms = millis() - bootMs;
  sample.axMg = static_cast<int16_t>(lroundf(currentAx * 1000.0f));
  sample.ayMg = static_cast<int16_t>(lroundf(currentAy * 1000.0f));
  sample.azMg = static_cast<int16_t>(lroundf(currentAz * 1000.0f));
  sample.gxDps10 = static_cast<int16_t>(lroundf((currentGx - gyroBiasX) * 10.0f));
  sample.gyDps10 = static_cast<int16_t>(lroundf((currentGy - gyroBiasY) * 10.0f));
  sample.gzDps10 = static_cast<int16_t>(lroundf((currentGz - gyroBiasZ) * 10.0f));
  sample.tempCd10 = static_cast<int16_t>(lroundf(currentTemperature * 10.0f));
  sample.pressureHd10 = static_cast<uint16_t>(lroundf(clampFloat(currentPressure * 10.0f, 0.0f, 65535.0f)));
  sample.altitudeCm = static_cast<int16_t>(lroundf(clampFloat(currentAltitude * 100.0f, -32768.0f, 32767.0f)));
  sample.velocityCms = static_cast<int16_t>(lroundf(clampFloat(currentVerticalVelocity * 100.0f, -32768.0f, 32767.0f)));
  sample.rollD10 = static_cast<int16_t>(lroundf(clampFloat(currentRollDeg * 10.0f, -32768.0f, 32767.0f)));
  sample.pitchD10 = static_cast<int16_t>(lroundf(clampFloat(currentPitchDeg * 10.0f, -32768.0f, 32767.0f)));
  sample.healthBits = healthBits();
  sample.state = static_cast<uint8_t>(flightState);
}

void ensureLogsDirectory() {
  if (!storageReady) {
    return;
  }
  FatFS.mkdir("/logs");
}

void ensureUsbDemoFile() {
  if (!storageReady) {
    return;
  }

  if (!FatFS.exists("/HOWTO.TXT")) {
    File howToFile = FatFS.open("/HOWTO.TXT", "w");
    if (!howToFile) {
      boardHealth.warning = true;
      return;
    }

    howToFile.println("AstroNav Nano USB Guide");
    howToFile.println("Author: YoupSpace");
    howToFile.println();
    howToFile.println("Open SETTINGS.TXT to tune the estimated flight height and speed.");
    howToFile.println("Change USB_EXIT=FALSE to USB_EXIT=TRUE and save SETTINGS.TXT to leave USB mode.");
    howToFile.println("PERSONAL.TXT is copied from the flash-backed profile and is regenerated automatically.");
    howToFile.println();
    howToFile.println("Files:");
    howToFile.println("- HOWTO.TXT: this guide");
    howToFile.println("- SETTINGS.TXT: editable flight estimates and USB exit switch");
    howToFile.println("- PERSONAL.TXT: flash-backed profile copy");
    howToFile.flush();
    howToFile.close();
  }

  if (!FatFS.exists("/SETTINGS.TXT")) {
    saveFlightSettings();
  }

  writePersonalFile();
}

bool shouldExitUsbModeFromDemoFile() {
  if (!storageReady) {
    return false;
  }

  loadFlightSettings();
  return flightSettings.usbExit;
}

void enterFlightModeFromUsb() {
  usbFlightOverride = true;
  if (usbDriveReady) {
    FatFSUSB.end();
    usbDriveReady = false;
    boardHealth.usbStorageOk = false;
  }

  flightState = FlightState::Calibrating;
  if (calibratePadOrientation()) {
    flightState = FlightState::Idle;
    startMissionLog();
    appendMissionSample();
  } else {
    boardHealth.critical = true;
    recordFaultAndSafeStop();
    flightState = FlightState::Fault;
  }
}

void handleSerialCommand(const char *command) {
  if (!command || !command[0]) {
    return;
  }

  char upperCommand[48] = {0};
  size_t len = strlen(command);
  if (len >= sizeof(upperCommand)) {
    len = sizeof(upperCommand) - 1;
  }

  for (size_t i = 0; i < len; i++) {
    upperCommand[i] = static_cast<char>(toupper(static_cast<unsigned char>(command[i])));
  }
  upperCommand[len] = '\0';

  if ((strcmp(upperCommand, "FLIGHT") == 0 || strcmp(upperCommand, "EXITUSB") == 0 || strcmp(upperCommand, "ARM") == 0) &&
      powerMode == PowerMode::USB && !usbFlightOverride) {
    enterFlightModeFromUsb();
  }
}

void flushMissionLogToFlash() {
  if (!storageReady || missionLogFlushed || missionLogCount == 0) {
    return;
  }

  ensureLogsDirectory();
  File logFile = FatFS.open(missionLogPath, "w");
  if (!logFile) {
    boardHealth.warning = true;
    return;
  }

  logFile.println("counter,ms,state,power,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_c,pressure_hpa,altitude_m,vertical_velocity_mps,roll_deg,pitch_deg,health_bits");
  char line[192];
  for (uint16_t i = 0; i < missionLogCount; i++) {
    const FlightSample &sample = missionLog[i];
    snprintf(line, sizeof(line),
             "%lu,%lu,%s,%s,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%.2f,%.2f,%.2f,%.1f,%.1f,%u",
             static_cast<unsigned long>(sample.counter),
             static_cast<unsigned long>(sample.ms),
             stateText(static_cast<FlightState>(sample.state)),
             powerModeText(powerMode),
             sample.axMg / 1000.0f,
             sample.ayMg / 1000.0f,
             sample.azMg / 1000.0f,
             sample.gxDps10 / 10.0f,
             sample.gyDps10 / 10.0f,
             sample.gzDps10 / 10.0f,
             sample.tempCd10 / 10.0f,
             sample.pressureHd10 / 10.0f,
             sample.altitudeCm / 100.0f,
             sample.velocityCms / 100.0f,
             sample.rollD10 / 10.0f,
             sample.pitchD10 / 10.0f,
             sample.healthBits);
    logFile.println(line);
  }

  logFile.flush();
  logFile.close();
  missionLogFlushed = true;
}

void setStatusLED(uint8_t r, uint8_t g, uint8_t b) {
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}

void setLedProfile(LedProfile profile) {
  float pulse = 1.0f;
  if (profile != LedProfile::Fault) {
    uint32_t now = millis();
    float phase = static_cast<float>((now % 2000UL)) / 2000.0f;
    pulse = 0.10f + 0.90f * (0.5f + 0.5f * sinf(phase * 6.2831853f));
  }

  auto scaleAndSet = [&](uint8_t r, uint8_t g, uint8_t b) {
    setStatusLED(static_cast<uint8_t>(r * pulse), static_cast<uint8_t>(g * pulse), static_cast<uint8_t>(b * pulse));
  };

  switch (profile) {
    case LedProfile::Boot:
      scaleAndSet(255, 255, 255);
      break;
    case LedProfile::Idle:
      scaleAndSet(0, 255, 160);
      break;
    case LedProfile::IdleUsb:
      scaleAndSet(0, 128, 255);
      break;
    case LedProfile::Idle1S:
      scaleAndSet(0, 255, 96);
      break;
    case LedProfile::Idle2S:
      scaleAndSet(255, 144, 0);
      break;
    case LedProfile::Coast:
      scaleAndSet(180, 0, 255);
      break;
    case LedProfile::Warning:
      scaleAndSet(255, 0, 160);
      break;
    case LedProfile::Landed:
      scaleAndSet(0, 220, 220);
      break;
    case LedProfile::Fault:
    default:
      setStatusLED(255, 0, 0);
      break;
  }
}

void updateStatusLed() {
  if (boardHealth.critical) {
    setLedProfile(LedProfile::Fault);
    return;
  }

  if (millis() < bootPulseUntilMs) {
    setLedProfile(LedProfile::Boot);
    return;
  }

  switch (flightState) {
    case FlightState::Booting:
      setLedProfile(LedProfile::Boot);
      break;
    case FlightState::Calibrating:
      setLedProfile(LedProfile::Boot);
      break;
    case FlightState::Idle:
      setLedProfile(LedProfile::Idle);
      break;
    case FlightState::UsbMode:
      setLedProfile(LedProfile::IdleUsb);
      break;
    case FlightState::Armed:
      setLedProfile(LedProfile::Idle1S);
      break;
    case FlightState::Boost:
      setLedProfile(LedProfile::Idle2S);
      break;
    case FlightState::Coast:
      setLedProfile(LedProfile::Coast);
      break;
    case FlightState::PyroFired:
      setLedProfile(LedProfile::Warning);
      break;
    case FlightState::Landed:
      setLedProfile(LedProfile::Landed);
      break;
    case FlightState::Fault:
    default:
      setLedProfile(LedProfile::Fault);
      break;
  }
}

void printStartupSummary() {
  Serial.printf("[BOOT] mode=%s state=%s healthy=%s\n",
                runtimeModeText(),
                stateText(flightState),
                boolText(systemHealthy()));
}

void printRuntimeSummary() {
  Serial.printf("[RUNTIME] mode=%s state=%s healthy=%s\n",
                runtimeModeText(),
                stateText(flightState),
                boolText(systemHealthy()));
}

void firePyro() {
  if (pyroLatched) {
    return;
  }

  pyroLatched = true;
  pyroPulseUntilMs = millis() + PYRO_PULSE_MS;
  digitalWrite(PIN_PYRO, HIGH);
  flightState = FlightState::PyroFired;
}

void updatePyroOutput() {
  if (pyroLatched && digitalRead(PIN_PYRO) == HIGH && millis() >= pyroPulseUntilMs) {
    digitalWrite(PIN_PYRO, LOW);
  }
}

bool missionAtSafeGroundState() {
  return flightState == FlightState::Landed || flightState == FlightState::Fault;
}

void updateFlightStateFromSamples() {
  float accelMag = magnitude3(currentAx, currentAy, currentAz);
  float gravityProjection = (currentAx * gravityAxisX) + (currentAy * gravityAxisY) + (currentAz * gravityAxisZ);
  currentRollDeg = atan2f(currentAy, currentAz) * 57.2957795f;
  currentPitchDeg = atan2f(-currentAx, sqrtf((currentAy * currentAy) + (currentAz * currentAz))) * 57.2957795f;

  if (currentPressure > 0.0f) {
    filteredPressure = (filteredPressure * 0.80f) + (currentPressure * 0.20f);
  }
  if (currentTemperature > -100.0f) {
    filteredTemperature = (filteredTemperature * 0.80f) + (currentTemperature * 0.20f);
  }

  currentAltitude = altitudeFromPressure(filteredPressure);
  filteredAltitude = (filteredAltitude * 0.80f) + (currentAltitude * 0.20f);
  currentAltitude = filteredAltitude;
  if (currentAltitude > peakAltitude) {
    peakAltitude = currentAltitude;
  }

  float rawVelocity = (currentAltitude - lastAltitudeForVelocity) / (SAMPLE_PERIOD_MS / 1000.0f);
  currentVerticalVelocity = (currentVerticalVelocity * 0.75f) + (rawVelocity * 0.25f);
  lastAltitudeForVelocity = currentAltitude;

  if (flightState == FlightState::Idle) {
    if (accelMag > flightLaunchThresholdG) {
      if (launchConfirmCount < 255) {
        launchConfirmCount++;
      }
    } else {
      launchConfirmCount = 0;
    }

    if (launchConfirmCount >= LAUNCH_CONFIRM_SAMPLES) {
      launchMs = millis();
      launchConfirmCount = 0;
      apogeeConfirmCount = 0;
      landingConfirmCount = 0;
      flightState = FlightState::Boost;
    }
  } else if (flightState == FlightState::Boost || flightState == FlightState::Coast) {
    if (flightState == FlightState::Boost && accelMag < 1.15f) {
      flightState = FlightState::Coast;
    }

    bool apogeeCandidate = (peakAltitude - currentAltitude) >= flightApogeeDropM &&
                           currentVerticalVelocity <= flightApogeeVelocityThresholdMPS &&
                           accelMag <= APOGEE_ACCEL_MAX_G &&
                           (millis() - launchMs) >= LAUNCH_MIN_TIME_MS;

    if (apogeeCandidate) {
      if (apogeeConfirmCount < 255) {
        apogeeConfirmCount++;
      }
    } else {
      apogeeConfirmCount = 0;
    }

    if (apogeeConfirmCount >= APOGEE_CONFIRM_SAMPLES) {
      firePyro();
      apogeeConfirmCount = 0;
    }

    if (!pyroLatched && (millis() - launchMs) > MAX_FLIGHT_TIME_MS) {
      firePyro();
    }
  }

  if (pyroLatched && missionStarted && !missionLogFlushed) {
    if (currentAltitude <= flightLandingAltitudeToleranceM && fabsf(currentVerticalVelocity) <= flightLandingVelocityToleranceMPS && accelMag <= 1.15f && gravityProjection >= 0.8f) {
      if (landingConfirmCount < 255) {
        landingConfirmCount++;
      }
    } else {
      landingConfirmCount = 0;
    }

    if (landingConfirmCount >= LANDING_CONFIRM_SAMPLES) {
      flightState = FlightState::Landed;
    }
  }
}

void recordFaultAndSafeStop() {
  flightState = FlightState::Fault;
  digitalWrite(PIN_PYRO, LOW);
}

void setup() {
  Serial.begin(115200);
  led.begin();
  led.setBrightness(40);
  setStatusLED(0, 0, 0);
  analogReadResolution(12);

  bootMs = millis();
  bootPulseUntilMs = bootMs + BOOT_PULSE_MS;
  nextSampleMs = bootMs;
  nextLedMs = bootMs;

  uint32_t startWait = millis();
  while (!Serial && (millis() - startWait < 1500)) {
    delay(10);
  }

  Serial.println();
  Serial.println("=============================================");
  Serial.println("  AstroNav flight firmware");
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

  digitalWrite(PIN_CS_BMP, LOW);
  delay(15);
  digitalWrite(PIN_CS_BMP, HIGH);
  delay(15);

  SPI.setRX(PIN_MISO);
  SPI.setSCK(PIN_SCK);
  SPI.setTX(PIN_MOSI);
  SPI.begin();

  boardHealth.coreTickOk = testCoreTick();
  boardHealth.heapOk = testHeap();
  boardHealth.vinOk = readInputVoltage(currentVinVoltage);
  powerMode = classifyPowerSource(currentVinVoltage);
  if (!boardHealth.vinOk || powerMode == PowerMode::Unknown) {
    boardHealth.warning = true;
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

  storageReady = FatFS.begin();
  boardHealth.flashFsOk = storageReady;
  if (storageReady) {
    ensureLogsDirectory();
    loadFlightSettings();
    loadOrCreateDeviceProfile();
  }

  if (powerMode == PowerMode::USB && storageReady && !usbFlightOverride) {
    usbDriveReady = FatFSUSB.begin();
    boardHealth.usbStorageOk = usbDriveReady;
    if (!usbDriveReady) {
      boardHealth.warning = true;
    }
    ensureUsbDemoFile();
  }

  if (!boardHealth.coreTickOk || !boardHealth.heapOk || !boardHealth.spiOk ||
      !boardHealth.imuWhoAmIOk || !boardHealth.imuConfigOk || !boardHealth.imuStreamOk ||
      !boardHealth.baroOk) {
    boardHealth.critical = true;
    recordFaultAndSafeStop();
  }

  if (powerMode == PowerMode::Unknown) {
    boardHealth.critical = true;
    recordFaultAndSafeStop();
  }

  printStartupSummary();

  if (boardHealth.critical) {
    flightState = FlightState::Fault;
  } else if (powerMode == PowerMode::USB && !usbFlightOverride) {
    flightState = FlightState::UsbMode;
  } else {
    flightState = FlightState::Calibrating;
    if (!calibratePadOrientation()) {
      boardHealth.critical = true;
      recordFaultAndSafeStop();
      flightState = FlightState::Fault;
    } else {
      flightState = FlightState::Idle;
    }
  }

  if (storageReady) {
    if (powerMode == PowerMode::USB && !usbFlightOverride) {
      writePersonalFile();
    } else {
      persistDeviceProfile();
    }
  }

  updateStatusLed();
  Serial.println("=============================================");
}

void loop() {
  uint32_t now = millis();

  while (Serial.available() > 0) {
    char incoming = static_cast<char>(Serial.read());
    if (incoming == '\r' || incoming == '\n') {
      if (serialCommandLength > 0) {
        serialCommandBuffer[serialCommandLength] = '\0';
        handleSerialCommand(serialCommandBuffer);
        serialCommandLength = 0;
      }
    } else if (serialCommandLength < sizeof(serialCommandBuffer) - 1) {
      serialCommandBuffer[serialCommandLength++] = incoming;
    }
  }

  if (powerMode == PowerMode::USB && !usbFlightOverride && shouldExitUsbModeFromDemoFile()) {
    enterFlightModeFromUsb();
  }

  if (now >= nextLedMs) {
    updateStatusLed();
    nextLedMs = now + LED_PERIOD_MS;
  }

  updatePyroOutput();

  if (boardHealth.critical) {
    flightState = FlightState::Fault;
  }

  if (now >= nextSampleMs) {
    nextSampleMs += SAMPLE_PERIOD_MS;

    if (readInputVoltage(currentVinVoltage)) {
      powerMode = classifyPowerSource(currentVinVoltage);
    } else {
      boardHealth.warning = true;
    }

    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;

    if (imuInitialized && readImuFrame(ax, ay, az, gx, gy, gz)) {
      currentAx = ax;
      currentAy = ay;
      currentAz = az;
      currentGx = gx;
      currentGy = gy;
      currentGz = gz;
      boardHealth.imuStreamOk = true;
    } else if (imuInitialized) {
      boardHealth.imuStreamOk = false;
      boardHealth.warning = true;
    }

    float temp = 0.0f;
    float pressure = 0.0f;
    if (readBaroSample(temp, pressure)) {
      currentTemperature = temp;
      currentPressure = pressure;
      boardHealth.baroOk = true;
    } else {
      boardHealth.baroOk = false;
      boardHealth.warning = true;
    }

    if (boardHealth.imuStreamOk && boardHealth.baroOk && !boardHealth.critical) {
      updateFlightStateFromSamples();
      appendMissionSample();
    }

    if (missionAtSafeGroundState() && !missionLogFlushed) {
      flushMissionLogToFlash();
    }

    if (now - lastLogMs >= 1000) {
      lastLogMs = now;
      printRuntimeSummary();
    }
  }

  delay(1);
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
  sample.ax = static_cast<int16_t>((rawData[1] << 8) | rawData[0]);
  sample.ay = static_cast<int16_t>((rawData[3] << 8) | rawData[2]);
  sample.az = static_cast<int16_t>((rawData[5] << 8) | rawData[4]);
  sample.gx = static_cast<int16_t>((rawData[7] << 8) | rawData[6]);
  sample.gy = static_cast<int16_t>((rawData[9] << 8) | rawData[8]);
  sample.gz = static_cast<int16_t>((rawData[11] << 8) | rawData[10]);
  return sample;
}
