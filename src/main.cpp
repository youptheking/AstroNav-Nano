#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <FatFS.h>
#include <FatFSUSB.h>
#include <ctype.h>
#include <math.h>
#include <string.h>

#include "hardware_control.h"
#include "otp_memory.h"
#include "usb_info_files.h"

#ifndef AUTO_VERSION
#define AUTO_VERSION "dev"
#endif

class Assets {
 public:
  const char *Firmware_Version = AUTO_VERSION;
  uint16_t TotalFlights = 0;
  String Device_Key = "0";
};

Assets assets;

namespace OldCodeTemporary {
String generateRandomKey(uint8_t len) {
  uint32_t seed = micros() ^ (millis() << 16) ^ static_cast<uint32_t>(analogRead(PIN_VIN_SENSE));
  randomSeed(seed);

  const char chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
  String key = "";
  for (uint8_t i = 0; i < len; i++) {
    key += chars[random(0, sizeof(chars) - 1)];
  }
  return key;
}
}

bool isPlaceholderSerial(const char *serialNumber) {
  return !serialNumber || serialNumber[0] == '\0' || strcmp(serialNumber, "AN-0001") == 0;
}

extern "C" bool tud_disconnect(void);

/*
 * AstroNav flight firmware for RP2350
 * - Startup health check with LED status
 * - Pad orientation calibration
 * - Launch and apogee detection
 * - Safe pyro pulse and lockout
 * - RAM mission log flushed to flash after landing
 * - USB mass-storage mode when powered from USB
 */

static constexpr float ACCEL_LSB_PER_G = 1024.0f;
static constexpr float GYRO_LSB_PER_DPS = 16.4f;
static constexpr float ADC_VREF = 3.3f;
static constexpr float ADC_COUNTS = 4095.0f;
static constexpr float VIN_DIVIDER_TOP_OHMS = 20000.0f;
static constexpr float VIN_DIVIDER_BOTTOM_OHMS = 10000.0f;
static constexpr float VIN_DIVIDER_RATIO =
  (VIN_DIVIDER_TOP_OHMS + VIN_DIVIDER_BOTTOM_OHMS) / VIN_DIVIDER_BOTTOM_OHMS;

static constexpr uint32_t SAMPLE_PERIOD_MS = 50;
static constexpr uint32_t LED_PERIOD_MS = 25;
static constexpr uint32_t BOOT_PULSE_MS = 2000;
static constexpr uint32_t CALIBRATION_TIMEOUT_MS = 6000;
static constexpr uint8_t CALIBRATION_GOOD_SAMPLES = 20;
static constexpr float STATIONARY_ACCEL_TOLERANCE_G = 0.08f;
static constexpr float STATIONARY_GYRO_TOLERANCE_DPS = 20.0f;
static constexpr float LAUNCH_THRESHOLD_G = 1.35f;
static constexpr uint8_t LAUNCH_CONFIRM_SAMPLES = 3;
static constexpr uint8_t BOOST_DECAY_CONFIRM_SAMPLES = 3;
static constexpr uint32_t LAUNCH_MIN_TIME_MS = 400;
static constexpr float APOGEE_DROP_M = 0.25f;
static constexpr float APOGEE_VELOCITY_THRESHOLD_MPS = -0.25f;
static constexpr float APOGEE_ACCEL_MAX_G = 0.90f;
static constexpr uint8_t APOGEE_CONFIRM_SAMPLES = 3;
static constexpr float LANDING_ALTITUDE_TOLERANCE_M = 2.0f;
static constexpr float LANDING_VELOCITY_TOLERANCE_MPS = 0.25f;
static constexpr uint8_t LANDING_CONFIRM_SAMPLES = 15;
static constexpr uint32_t PYRO_PULSE_MS = 1000;
static constexpr uint32_t PYRO_TEST_CONFIRM_WINDOW_MS = 5000;
static constexpr uint32_t APOGEE_FIRE_MAX_MS = 500;
static constexpr uint8_t APOGEE_HOLD_SAMPLES = 8;
static constexpr uint32_t MAX_FLIGHT_TIME_MS = 45000;
static constexpr uint16_t FLIGHT_LOG_CAPACITY = 2048;
static constexpr uint32_t PROFILE_MAGIC = 0x50455246;   // "FRFP"
static constexpr uint32_t PROFILE_VERSION = 1;
static constexpr const char *DEVICE_INFO_FILE = "/Settings.ini";
static constexpr const char *HOWTO_FILE = "/Howto.txt";
static constexpr const char *WEBSITE_FILE = "/Website.url";
static constexpr const char *AUTORUN_FILE = "/autorun.inf";
static constexpr const char *USB_VOLUME_LABEL = "AstroNav Nano";
static constexpr const char *LEGACY_SETTINGS_FILE = "/SETTINGS.INI";
static constexpr const char *LEGACY_PROFILE_FILE = "/PROFILE.INI";
static constexpr const char *LEGACY_DEBUG_FILE = "/DEBUG.INI";
static constexpr const char *LEGACY_HOWTO_FILE = "/HOWTO.INI";
static constexpr const char *LEGACY_HOWTO_TEXT_FILE = "/HOWTO.TXT";
static constexpr const char *LEGACY_WEBSITE_FILE = "/WEBSITE.URL";
static constexpr const char *LEGACY_MOREHELP_FILE = "/MoreHelp.url";
static constexpr const char *LEGACY_AUTORUN_FILE = "/AUTORUN.INF";
static constexpr const char *SECTION_FILES = "files";
static constexpr const char *SECTION_SETTINGS = "settings";
static constexpr const char *SECTION_PROFILE = "profile";
static constexpr const char *SECTION_DEBUG = "debug";

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
  Calibrating,
  Idle,
  IdleUsb,
  Idle1S,
  Idle2S,
  Coast,
  Warning,
  Landed,
  Fault
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
  uint8_t pyro = 0;
};

struct FlightSettings {
  float estimatedHeightM = 120.0f;
  float estimatedSpeedMps = 35.0f;
  float heightMarginM = 20.0f;
  float speedMarginMps = 8.0f;
  float launchThresholdG = 1.35f;
  bool firePyroAtApogee = true;
};

struct DeviceProfile {
  uint32_t magic = PROFILE_MAGIC;
  uint32_t version = PROFILE_VERSION;
  char serialNumber[20] = "";
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
bool usbExitRequested = false;
bool settingsNeedPersist = false;
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
uint32_t lastUsbSettingsPollMs = 0;
char serialCommandBuffer[48] = {0};
uint8_t serialCommandLength = 0;
bool pyroTestPendingConfirm = false;
uint32_t pyroTestConfirmUntilMs = 0;

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
uint8_t boostDecayConfirmCount = 0;
uint8_t apogeeConfirmCount = 0;
uint8_t landingConfirmCount = 0;
uint8_t apogeePeakHoldCount = 0;
uint32_t apogeePeakHoldStartMs = 0;
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
uint32_t currentFlightNumber = 0;

char missionLogPath[48] = {0};
char lastFaultReason[96] = "none";

void setStatusLED(uint8_t r, uint8_t g, uint8_t b);
void setLedProfile(LedProfile profile);
void updateStatusLed();
bool setFlightState(FlightState nextState);
float clampFloat(float value, float minimum, float maximum);
bool testCoreTick();
bool testHeap();
bool readInputVoltage(float &voltage);
bool readBaroSample(float &temperature, float &pressureHpa);
bool readImuFrame(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
bool verifyImuStream();
bool configureImu();
bool calibratePadOrientation();
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
void onUsbStorageUnplug(uint32_t cbData);
void ensureUsbInfoFiles();
bool configureUsbVolumeLabel();
bool shouldExitUsbModeFromUsbFiles();
bool loadFlightSettings();
bool loadOrCreateDeviceProfile();
void persistDeviceProfile();
void syncAssetsFromDeviceProfile();
void buildFaultReason(char *buffer, size_t bufferSize);
uint8_t healthBits();
bool systemHealthy();
uint32_t countAndCleanFlightLogs();

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
    case FlightState::UsbMode: return "storage";
    case FlightState::Armed: return "armed";
    case FlightState::Boost: return "boost";
    case FlightState::Coast: return "coast";
    case FlightState::PyroFired: return "pyro";
    case FlightState::Landed: return "landed";
    case FlightState::Fault:
    default: return "fault";
  }
}

bool setFlightState(FlightState nextState) {
  if (flightState == nextState) {
    return true;
  }

  bool allowed = false;
  switch (flightState) {
    case FlightState::Booting:
      allowed = (nextState == FlightState::Calibrating || nextState == FlightState::UsbMode || nextState == FlightState::Fault);
      break;
    case FlightState::Calibrating:
      allowed = (nextState == FlightState::Idle || nextState == FlightState::Fault);
      break;
    case FlightState::Idle:
      allowed = (nextState == FlightState::Boost || nextState == FlightState::UsbMode || nextState == FlightState::Fault);
      break;
    case FlightState::UsbMode:
      allowed = (nextState == FlightState::Calibrating || nextState == FlightState::Fault);
      break;
    case FlightState::Armed:
      allowed = (nextState == FlightState::Boost || nextState == FlightState::Fault);
      break;
    case FlightState::Boost:
      allowed = (nextState == FlightState::Coast || nextState == FlightState::PyroFired ||
                 nextState == FlightState::Landed || nextState == FlightState::Fault);
      break;
    case FlightState::Coast:
      allowed = (nextState == FlightState::PyroFired || nextState == FlightState::Landed || nextState == FlightState::Fault);
      break;
    case FlightState::PyroFired:
      allowed = (nextState == FlightState::Landed || nextState == FlightState::Fault);
      break;
    case FlightState::Landed:
      allowed = (nextState == FlightState::Fault);
      break;
    case FlightState::Fault:
    default:
      allowed = (nextState == FlightState::Fault);
      break;
  }

  if (!allowed) {
    boardHealth.warning = true;
    strncpy(lastFaultReason, "stateTransition", sizeof(lastFaultReason) - 1);
    lastFaultReason[sizeof(lastFaultReason) - 1] = '\0';
    return false;
  }

  flightState = nextState;
  if (missionStarted) {
    appendMissionSample();
  }
  return true;
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
    return "usb-storage";
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
  flightSettings.firePyroAtApogee = true;
  applyFlightSettings();
}

void buildFaultReason(char *buffer, size_t bufferSize) {
  buffer[0] = '\0';

  auto appendReason = [&](const char *label) {
    if (!label || !label[0]) {
      return;
    }
    size_t currentLength = strlen(buffer);
    size_t labelLength = strlen(label);
    if (currentLength + labelLength + 2 >= bufferSize) {
      return;
    }
    if (currentLength > 0) {
      strncat(buffer, ",", bufferSize - currentLength - 1);
    }
    strncat(buffer, label, bufferSize - strlen(buffer) - 1);
  };

  if (!boardHealth.coreTickOk) appendReason("coreTick");
  if (!boardHealth.heapOk) appendReason("heap");
  if (!boardHealth.spiOk) appendReason("spi");
  if (!boardHealth.imuWhoAmIOk) appendReason("imuWhoAmI");
  if (!boardHealth.imuConfigOk) appendReason("imuConfig");
  if (!boardHealth.imuStreamOk) appendReason("imuStream");
  if (!boardHealth.baroOk) appendReason("baro");
  if (!boardHealth.flashFsOk) appendReason("flashFs");
  if (!boardHealth.usbStorageOk && powerMode == PowerMode::USB) appendReason("usbStorage");
  if (!boardHealth.vinOk) appendReason("vin");
  if (powerMode == PowerMode::Unknown) appendReason("powerMode");

  if (buffer[0] == '\0') {
    strncpy(buffer, "none", bufferSize - 1);
    buffer[bufferSize - 1] = '\0';
  }
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

bool readProfileLine(const char *line, const char *key, char *valueBuffer, size_t valueBufferSize) {
  return readSettingLine(line, key, valueBuffer, valueBufferSize);
}

void writeProfileFieldLine(File &file, const char *key, const char *value) {
  file.printf("%s=%s\n", key, value ? value : "");
}

bool parseIniSectionHeader(const char *line, char *sectionBuffer, size_t sectionBufferSize) {
  if (!line || line[0] != '[') {
    return false;
  }

  const char *sectionStart = line + 1;
  const char *sectionEnd = strchr(sectionStart, ']');
  if (!sectionEnd || sectionEnd <= sectionStart) {
    return false;
  }

  size_t length = static_cast<size_t>(sectionEnd - sectionStart);
  if (length >= sectionBufferSize) {
    length = sectionBufferSize - 1;
  }

  for (size_t i = 0; i < length; i++) {
    sectionBuffer[i] = static_cast<char>(tolower(static_cast<unsigned char>(sectionStart[i])));
  }
  sectionBuffer[length] = '\0';
  return true;
}

bool loadFlightSettings() {
  const char *sourcePath = nullptr;
  bool legacyFormat = false;

  if (storageReady && FatFS.exists(DEVICE_INFO_FILE)) {
    sourcePath = DEVICE_INFO_FILE;
  } else if (storageReady && FatFS.exists(LEGACY_SETTINGS_FILE)) {
    sourcePath = LEGACY_SETTINGS_FILE;
    legacyFormat = true;
  }

  if (!sourcePath) {
    makeDefaultFlightSettings();
    return false;
  }

  File settingsFile = FatFS.open(sourcePath, "r");
  if (!settingsFile) {
    makeDefaultFlightSettings();
    return false;
  }

  makeDefaultFlightSettings();
  bool anyValue = false;
  bool settingsDirty = false;
  bool estimatedHeightLoaded = false;
  bool estimatedSpeedLoaded = false;
  bool heightMarginLoaded = false;
  bool speedMarginLoaded = false;
  bool launchThresholdLoaded = false;
  bool firePyroAtApogeeLoaded = false;
  char currentSection[32] = {0};
  char line[128];

  while (settingsFile.available()) {
    size_t length = settingsFile.readBytesUntil('\n', line, sizeof(line) - 1);
    line[length] = '\0';

    if (line[0] == '#' || line[0] == ';' || line[0] == '[' || line[0] == '\0') {
      if (line[0] == '[') {
        parseIniSectionHeader(line, currentSection, sizeof(currentSection));
      }
      continue;
    }

    if (!legacyFormat && strcmp(currentSection, SECTION_SETTINGS) != 0) {
      continue;
    }

    char value[64];
    if (readSettingLine(line, "ESTIMATED_HEIGHT_M", value, sizeof(value))) {
      estimatedHeightLoaded = true;
      if (value[0] != '\0') {
        flightSettings.estimatedHeightM = static_cast<float>(strtof(value, nullptr));
      } else {
        settingsDirty = true;
      }
      anyValue = true;
    } else if (readSettingLine(line, "ESTIMATED_SPEED_MPS", value, sizeof(value))) {
      estimatedSpeedLoaded = true;
      if (value[0] != '\0') {
        flightSettings.estimatedSpeedMps = static_cast<float>(strtof(value, nullptr));
      } else {
        settingsDirty = true;
      }
      anyValue = true;
    } else if (readSettingLine(line, "HEIGHT_MARGIN_M", value, sizeof(value))) {
      heightMarginLoaded = true;
      if (value[0] != '\0') {
        flightSettings.heightMarginM = static_cast<float>(strtof(value, nullptr));
      } else {
        settingsDirty = true;
      }
      anyValue = true;
    } else if (readSettingLine(line, "SPEED_MARGIN_MPS", value, sizeof(value))) {
      speedMarginLoaded = true;
      if (value[0] != '\0') {
        flightSettings.speedMarginMps = static_cast<float>(strtof(value, nullptr));
      } else {
        settingsDirty = true;
      }
      anyValue = true;
    } else if (readSettingLine(line, "LAUNCH_THRESHOLD_G", value, sizeof(value))) {
      launchThresholdLoaded = true;
      if (value[0] != '\0') {
        flightSettings.launchThresholdG = static_cast<float>(strtof(value, nullptr));
      } else {
        settingsDirty = true;
      }
      anyValue = true;
    } else if (readSettingLine(line, "FIRE_PYRO_APOGEE", value, sizeof(value))) {
      firePyroAtApogeeLoaded = true;
      if (value[0] != '\0') {
        flightSettings.firePyroAtApogee = parseBoolValue(value);
      } else {
        settingsDirty = true;
      }
      anyValue = true;
    }
  }

  settingsFile.close();
  applyFlightSettings();
  if (!estimatedHeightLoaded || !estimatedSpeedLoaded || !heightMarginLoaded || !speedMarginLoaded || !launchThresholdLoaded || !firePyroAtApogeeLoaded) {
    settingsDirty = true;
  }

  settingsNeedPersist = settingsDirty;
  return true;
}

void saveFlightSettings() {
  if (!storageReady) {
    return;
  }

  persistDeviceProfile();
}

void makeDefaultDeviceProfile() {
  memset(&deviceProfile, 0, sizeof(deviceProfile));
  deviceProfile.magic = PROFILE_MAGIC;
  deviceProfile.version = PROFILE_VERSION;
  deviceProfile.serialNumber[0] = '\0';
  assets.Device_Key = "0";
  buildFlashStamp(deviceProfile.flashStamp, sizeof(deviceProfile.flashStamp));
  assets.TotalFlights = 0;
}

void ensureDeviceSerial() {
  uint32_t otpSerialNumber = 0;
  if (astroNavOtpGetSerialNumber(otpSerialNumber) && otpSerialNumber != 0) {
    formatAstroNavDisplaySerial(otpSerialNumber, deviceProfile.serialNumber, sizeof(deviceProfile.serialNumber));
    assets.Device_Key = deviceProfile.serialNumber;
    return;
  }

  if (!isPlaceholderSerial(deviceProfile.serialNumber)) {
    assets.Device_Key = deviceProfile.serialNumber;
    return;
  }

  assets.Device_Key = OldCodeTemporary::generateRandomKey(10);
  snprintf(deviceProfile.serialNumber, sizeof(deviceProfile.serialNumber), "%s", assets.Device_Key.c_str());
}

void updateDeviceProfileChecksum() {
  deviceProfile.checksum = 0;
  deviceProfile.checksum = checksumBytes(reinterpret_cast<const uint8_t *>(&deviceProfile), sizeof(deviceProfile) - sizeof(deviceProfile.checksum));
}

void syncAssetsFromDeviceProfile() {
  assets.TotalFlights = static_cast<uint16_t>(deviceProfile.flightCount > UINT16_MAX ? UINT16_MAX : deviceProfile.flightCount);
  assets.Device_Key = deviceProfile.serialNumber;
}

bool loadOrCreateDeviceProfile() {
  bool legacyFormat = false;
  const char *sourcePath = nullptr;

  if (storageReady && FatFS.exists(DEVICE_INFO_FILE)) {
    sourcePath = DEVICE_INFO_FILE;
  } else if (storageReady && FatFS.exists(LEGACY_PROFILE_FILE)) {
    sourcePath = LEGACY_PROFILE_FILE;
    legacyFormat = true;
  }

  if (!sourcePath) {
    makeDefaultDeviceProfile();
    updateDeviceProfileChecksum();
    return false;
  }

  File profileFile = FatFS.open(sourcePath, "r");
  if (!profileFile) {
    makeDefaultDeviceProfile();
    updateDeviceProfileChecksum();
    return false;
  }

  makeDefaultDeviceProfile();
  bool anyValue = false;
  bool checksumPresent = false;
  uint32_t expectedChecksum = 0;
  char currentSection[32] = {0};
  char line[160];

  while (profileFile.available()) {
    size_t length = profileFile.readBytesUntil('\n', line, sizeof(line) - 1);
    line[length] = '\0';

    if (line[0] == '#' || line[0] == ';' || line[0] == '[' || line[0] == '\0') {
      if (line[0] == '[') {
        parseIniSectionHeader(line, currentSection, sizeof(currentSection));
      }
      continue;
    }

    if (!legacyFormat && strcmp(currentSection, SECTION_PROFILE) != 0) {
      continue;
    }

    char value[96];
    if (readProfileLine(line, "MAGIC", value, sizeof(value))) {
      deviceProfile.magic = static_cast<uint32_t>(strtoul(value, nullptr, 16));
      anyValue = true;
    } else if (readProfileLine(line, "SERIAL_NUMBER", value, sizeof(value))) {
      strncpy(deviceProfile.serialNumber, value, sizeof(deviceProfile.serialNumber) - 1);
      deviceProfile.serialNumber[sizeof(deviceProfile.serialNumber) - 1] = '\0';
      anyValue = true;
      if (!isPlaceholderSerial(deviceProfile.serialNumber)) {
        assets.Device_Key = deviceProfile.serialNumber;
      }
    } else if (readProfileLine(line, "FLASH_STAMP", value, sizeof(value))) {
      strncpy(deviceProfile.flashStamp, value, sizeof(deviceProfile.flashStamp) - 1);
      deviceProfile.flashStamp[sizeof(deviceProfile.flashStamp) - 1] = '\0';
      anyValue = true;
    } else if (readProfileLine(line, "FLIGHT_COUNT", value, sizeof(value))) {
      deviceProfile.flightCount = static_cast<uint32_t>(strtoul(value, nullptr, 10));
      anyValue = true;
      assets.TotalFlights = static_cast<uint16_t>(deviceProfile.flightCount > UINT16_MAX ? UINT16_MAX : deviceProfile.flightCount);
    } else if (readProfileLine(line, "LOG_COUNT", value, sizeof(value))) {
      deviceProfile.logCount = static_cast<uint32_t>(strtoul(value, nullptr, 10));
      anyValue = true;
    } else if (readProfileLine(line, "MAX_FLIGHT_ALTITUDE_M", value, sizeof(value))) {
      deviceProfile.maxFlightAltitudeM = static_cast<float>(atof(value));
      anyValue = true;
    } else if (readProfileLine(line, "MAX_FLIGHT_SPEED_MPS", value, sizeof(value))) {
      deviceProfile.maxFlightSpeedMps = static_cast<float>(atof(value));
      anyValue = true;
    } else if (readProfileLine(line, "LAST_FLIGHT_ALTITUDE_M", value, sizeof(value))) {
      deviceProfile.lastFlightAltitudeM = static_cast<float>(atof(value));
      anyValue = true;
    } else if (readProfileLine(line, "LAST_FLIGHT_SPEED_MPS", value, sizeof(value))) {
      deviceProfile.lastFlightSpeedMps = static_cast<float>(atof(value));
      anyValue = true;
    } else if (readProfileLine(line, "CHECKSUM", value, sizeof(value))) {
      expectedChecksum = static_cast<uint32_t>(strtoul(value, nullptr, 16));
      checksumPresent = true;
    }
  }

  profileFile.close();

  deviceProfile.checksum = 0;
  if (deviceProfile.magic != PROFILE_MAGIC || deviceProfile.version != PROFILE_VERSION) {
    makeDefaultDeviceProfile();
    updateDeviceProfileChecksum();
    return false;
  }

  uint32_t computedChecksum = checksumBytes(reinterpret_cast<const uint8_t *>(&deviceProfile), sizeof(deviceProfile) - sizeof(deviceProfile.checksum));
  if (!anyValue || !checksumPresent || computedChecksum != expectedChecksum) {
    makeDefaultDeviceProfile();
    updateDeviceProfileChecksum();
    return false;
  }

  deviceProfile.checksum = expectedChecksum;
  ensureDeviceSerial();
  syncAssetsFromDeviceProfile();
  return true;
}

void persistDeviceProfile() {
  if (!storageReady) {
    return;
  }

  ensureDeviceSerial();
  AstroNav_OTP_Data otpData = {};
  bool otpReadable = readAstroNavOtpData(otpData);
  const AstroNavOtpStatus &otpStatus = getAstroNavOtpStatus();

  updateDeviceProfileChecksum();
  File infoFile = FatFS.open(DEVICE_INFO_FILE, "w");
  if (!infoFile) {
    boardHealth.warning = true;
    return;
  }

  infoFile.println("; AstroNav Nano combined device settings");
  infoFile.println("[files]");
  infoFile.println("SETTINGS_FILE=Settings.ini");
  infoFile.println("HOWTO_FILE=Howto.txt");
  infoFile.println("WEBSITE_FILE=Website.url");
  infoFile.println("LOG_DIRECTORY=/logs");
  infoFile.println("LOG_PATTERN=Flight_*.csv");
  infoFile.println();
  UsbInfoFiles::writeSettingsSection(infoFile,
                                    flightSettings.estimatedHeightM,
                                    flightSettings.estimatedSpeedMps,
                                    flightSettings.heightMarginM,
                                    flightSettings.speedMarginMps,
                                    flightSettings.launchThresholdG,
                                    flightSettings.firePyroAtApogee);
  infoFile.println();
  infoFile.println("[profile]");
  writeProfileFieldLine(infoFile, "MAGIC", "50455246");
  infoFile.printf("VERSION=%s\n", assets.Firmware_Version);
  writeProfileFieldLine(infoFile, "SERIAL_NUMBER", deviceProfile.serialNumber);
  writeProfileFieldLine(infoFile, "FLASH_STAMP", deviceProfile.flashStamp);
  infoFile.printf("FLIGHT_COUNT=%lu\n", static_cast<unsigned long>(deviceProfile.flightCount));
  infoFile.printf("LOG_COUNT=%lu\n", static_cast<unsigned long>(deviceProfile.logCount));
  infoFile.print("MAX_FLIGHT_ALTITUDE_M=");
  infoFile.println(deviceProfile.maxFlightAltitudeM, 1);
  infoFile.print("MAX_FLIGHT_SPEED_MPS=");
  infoFile.println(deviceProfile.maxFlightSpeedMps, 1);
  infoFile.print("LAST_FLIGHT_ALTITUDE_M=");
  infoFile.println(deviceProfile.lastFlightAltitudeM, 1);
  infoFile.print("LAST_FLIGHT_SPEED_MPS=");
  infoFile.println(deviceProfile.lastFlightSpeedMps, 1);
  infoFile.printf("CHECKSUM=%08lX\n", static_cast<unsigned long>(deviceProfile.checksum));
  infoFile.println();
  infoFile.println("[otp]");
  infoFile.printf("OTP_READ_OK=%s\n", otpReadable ? "TRUE" : "FALSE");
  infoFile.printf("OTP_MAGIC_VALID=%s\n", otpStatus.has_magic_header ? "TRUE" : "FALSE");
  infoFile.printf("OTP_PROGRAMMED_THIS_BOOT=%s\n", astroNavOtpWasProgrammedThisBoot() ? "TRUE" : "FALSE");
  infoFile.printf("OTP_PROFILE_MATCHES_CURRENT_BUILD=%s\n", otpStatus.signature_matches_build ? "TRUE" : "FALSE");
  infoFile.printf("OTP_OFFICIAL_SIGNATURE=%s\n", otpStatus.official_signature ? "TRUE" : "FALSE");
  if (otpReadable) {
    char otpDisplaySerial[16] = {0};
    formatAstroNavDisplaySerial(otpData.serial_number, otpDisplaySerial, sizeof(otpDisplaySerial));
    infoFile.printf("OTP_MAGIC_HEADER=%08lX\n", static_cast<unsigned long>(otpData.magic_header));
    infoFile.printf("OTP_MANUFACTURER_ID=%s\n", otpData.manufacturer_id);
    infoFile.printf("OTP_PRODUCT_ID=%s\n", otpData.product_id);
    infoFile.printf("OTP_HARDWARE_MAJOR=%u\n", static_cast<unsigned int>(otpData.hardware_major));
    infoFile.printf("OTP_HARDWARE_MINOR=%u\n", static_cast<unsigned int>(otpData.hardware_minor));
    infoFile.printf("OTP_PRODUCTION_DATE=%s\n", otpData.production_date);
    infoFile.printf("OTP_PRODUCTION_SECOND=%u\n", static_cast<unsigned int>(otpData.reserved[0]));
    infoFile.printf("OTP_PRODUCTION_TIMESTAMP=%s-%02u\n", otpData.production_date, static_cast<unsigned int>(otpData.reserved[0]));
    infoFile.printf("OTP_SERIAL_NUMBER=%s\n", otpDisplaySerial);
    infoFile.printf("OTP_INITIAL_FIRMWARE=%u.%u.%u\n",
                    static_cast<unsigned int>(otpData.initial_firmware[0]),
                    static_cast<unsigned int>(otpData.initial_firmware[1]),
                    static_cast<unsigned int>(otpData.initial_firmware[2]));
    infoFile.printf("OTP_WARRANTY_SIGNATURE=%08lX\n", static_cast<unsigned long>(otpData.warranty_signature));
  }
  infoFile.println();
  UsbInfoFiles::writeDebugSection(infoFile,
                                  lastFaultReason,
                                  boardHealth.coreTickOk,
                                  boardHealth.heapOk,
                                  boardHealth.spiOk,
                                  boardHealth.imuWhoAmIOk,
                                  boardHealth.imuConfigOk,
                                  boardHealth.imuStreamOk,
                                  boardHealth.baroOk,
                                  boardHealth.flashFsOk,
                                  boardHealth.usbStorageOk,
                                  boardHealth.vinOk,
                                  boardHealth.warning,
                                  boardHealth.critical,
                                  currentVinVoltage,
                                  healthBits(),
                                  runtimeModeText(),
                                  stateText(flightState),
                                  powerModeText(powerMode));
  infoFile.flush();
  infoFile.close();
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

float magnitude3(float x, float y, float z) {
  return sqrtf((x * x) + (y * y) + (z * z));
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
  uint32_t countedLogs = countAndCleanFlightLogs();
  uint32_t persistedFlightCount = deviceProfile.flightCount;
  currentFlightNumber = (countedLogs > persistedFlightCount ? countedLogs : persistedFlightCount) + 1u;
  deviceProfile.flightCount = currentFlightNumber;
  assets.TotalFlights = static_cast<uint16_t>(deviceProfile.flightCount > UINT16_MAX ? UINT16_MAX : deviceProfile.flightCount);
  snprintf(missionLogPath, sizeof(missionLogPath), "/logs/Flight_%08lu.csv", static_cast<unsigned long>(currentFlightNumber));
  missionLogCount = 0;
  peakAltitude = 0.0f;
  lastAltitudeForVelocity = 0.0f;
  launchConfirmCount = 0;
  boostDecayConfirmCount = 0;
  apogeeConfirmCount = 0;
  landingConfirmCount = 0;
  apogeePeakHoldCount = 0;
  apogeePeakHoldStartMs = 0;
  missionStarted = true;
  missionLogFlushed = false;
  logCounter = 0;
  persistDeviceProfile();
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
  sample.pyro = pyroLatched ? 1u : 0u;
}

void ensureLogsDirectory() {
  if (!storageReady) {
    return;
  }
  FatFS.mkdir("/logs");
}

uint32_t countAndCleanFlightLogs() {
  if (!storageReady) {
    return 0;
  }

  File logsDirectory = FatFS.open("/logs", "r");
  if (!logsDirectory || !logsDirectory.isDirectory()) {
    return 0;
  }

  uint32_t flightLogCount = 0;
  File entry = logsDirectory.openNextFile();
  while (entry) {
    String entryName = entry.name();
    bool validFlightLog = entryName.length() == 19 &&
                          entryName.startsWith("Flight_") &&
                          entryName.endsWith(".csv");
    if (validFlightLog) {
      for (size_t i = 7; i < 15; i++) {
        if (!isDigit(entryName[i])) {
          validFlightLog = false;
          break;
        }
      }
    }

    entry.close();
    if (validFlightLog) {
      flightLogCount++;
    } else {
      FatFS.remove(entryName.c_str());
    }
    entry = logsDirectory.openNextFile();
  }

  logsDirectory.close();
  return flightLogCount;
}

void cleanupLegacyUsbFiles() {
  if (!storageReady) {
    return;
  }

  FatFS.remove("/SETTINGS.BIN");
  FatFS.remove("/settings.bin");
  FatFS.remove("/USB_MODE_DEMO");
  FatFS.remove("/USB_MODE_DEMO.TXT");
  FatFS.remove("/USB_MODE_DEMO.INI");
  FatFS.remove(LEGACY_HOWTO_FILE);
  FatFS.remove(LEGACY_HOWTO_TEXT_FILE);
  FatFS.remove("/PERSONAL.TXT");
  FatFS.remove("/Profile.bin");
  FatFS.remove("/PROFILE.BIN");
  FatFS.remove(LEGACY_SETTINGS_FILE);
  FatFS.remove(LEGACY_PROFILE_FILE);
  FatFS.remove(LEGACY_DEBUG_FILE);
  FatFS.remove(LEGACY_WEBSITE_FILE);
  FatFS.remove(LEGACY_MOREHELP_FILE);
  FatFS.remove(LEGACY_AUTORUN_FILE);
  FatFS.remove("/SETTINGS.TXT");
  FatFS.remove("/WEBSITE.TXT");
}

bool configureUsbVolumeLabel() {
  if (!storageReady) {
    return false;
  }

  fatfs::f_setlabel("ASTRONAV");

  return true;
}

void ensureUsbInfoFiles() {
  if (!storageReady) {
    return;
  }

  cleanupLegacyUsbFiles();

  File howToFile = FatFS.open(HOWTO_FILE, "w");
  if (!howToFile) {
    boardHealth.warning = true;
    return;
  }

  UsbInfoFiles::writeHowtoText(howToFile);
  howToFile.flush();
  howToFile.close();

  // Avoid autorun metadata to reduce Windows trust/scanning warnings.
  FatFS.remove(AUTORUN_FILE);

  File websiteFile = FatFS.open(WEBSITE_FILE, "w");
  if (!websiteFile) {
    boardHealth.warning = true;
    return;
  }

  UsbInfoFiles::writeWebsiteShortcut(websiteFile);
  websiteFile.flush();
  websiteFile.close();

  persistDeviceProfile();
}

void onUsbStorageUnplug(uint32_t cbData) {
  (void) cbData;
  if (!usbFlightOverride) {
    usbExitRequested = true;
  }
}

bool shouldExitUsbModeFromUsbFiles() {
  if (!storageReady) {
    return false;
  }

  return usbExitRequested;
}

void enterFlightModeFromUsb() {
  usbFlightOverride = true;
  usbExitRequested = false;

  if (storageReady) {
    loadFlightSettings();
    if (settingsNeedPersist) {
      persistDeviceProfile();
      settingsNeedPersist = false;
    }
  }

  if (usbDriveReady) {
    FatFSUSB.unplug();
    usbDriveReady = false;
    boardHealth.usbStorageOk = false;
  }

  tud_disconnect();

  setFlightState(FlightState::Calibrating);
  if (calibratePadOrientation()) {
    setFlightState(FlightState::Idle);
  } else {
    boardHealth.critical = true;
    recordFaultAndSafeStop();
    setFlightState(FlightState::Fault);
  }
}

void handleSerialCommand(const char *command) {
  if (!command || !command[0]) {
    return;
  }

  if (pyroTestPendingConfirm && millis() > pyroTestConfirmUntilMs) {
    pyroTestPendingConfirm = false;
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

  if (strcmp(upperCommand, "PYROTEST") == 0) {
    bool inFlightState =
      flightState == FlightState::Idle ||
      flightState == FlightState::Armed ||
      flightState == FlightState::Boost ||
      flightState == FlightState::Coast ||
      flightState == FlightState::PyroFired ||
      flightState == FlightState::Landed;

    if (pyroLatched) {
      pyroTestPendingConfirm = false;
      Serial.println("[PYROTEST] Ignored: pyro has already fired.");
      return;
    }

    if (!inFlightState) {
      pyroTestPendingConfirm = false;
      Serial.println("[PYROTEST] Blocked: command requires flight mode (not USB storage mode).");
      return;
    }

    if (!pyroTestPendingConfirm) {
      pyroTestPendingConfirm = true;
      pyroTestConfirmUntilMs = millis() + PYRO_TEST_CONFIRM_WINDOW_MS;
      Serial.println("[PYROTEST] WARNING: Send PYROTEST again within 5s to ignite pyro output.");
      return;
    }

    pyroTestPendingConfirm = false;
    Serial.println("[PYROTEST] WARNING: Igniting pyro output now.");
    firePyro();
    return;
  }

  pyroTestPendingConfirm = false;

  if (strcmp(upperCommand, "DUMP_OTP") == 0) {
    printAstroNavOtpSummary();
    return;
  }

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

  logFile.println("# AstroNav Nano Flight Log");
  logFile.printf("# Flight Number: %lu\n", static_cast<unsigned long>(currentFlightNumber));
  logFile.printf("# Boot Time Ms: %lu\n", static_cast<unsigned long>(bootMs));
  logFile.printf("# Samples: %u\n", missionLogCount);
  logFile.println("# Columns: counter;ms;state;power;pyro;ax_g;ay_g;az_g;gx_dps;gy_dps;gz_dps;temp_c;pressure_hpa;altitude_m;vertical_velocity_mps;roll_deg;pitch_deg;health_bits");
  logFile.println("counter;ms;state;power;pyro;ax_g;ay_g;az_g;gx_dps;gy_dps;gz_dps;temp_c;pressure_hpa;altitude_m;vertical_velocity_mps;roll_deg;pitch_deg;health_bits");
  for (uint16_t i = 0; i < missionLogCount; i++) {
    const FlightSample &sample = missionLog[i];
    logFile.print(static_cast<unsigned long>(sample.counter));
    logFile.print(';');
    logFile.print(static_cast<unsigned long>(sample.ms));
    logFile.print(';');
    logFile.print(stateText(static_cast<FlightState>(sample.state)));
    logFile.print(';');
    logFile.print(powerModeText(powerMode));
    logFile.print(';');
    logFile.print(static_cast<unsigned>(sample.pyro));
    logFile.print(';');
    logFile.print(sample.axMg / 1000.0f, 3);
    logFile.print(';');
    logFile.print(sample.ayMg / 1000.0f, 3);
    logFile.print(';');
    logFile.print(sample.azMg / 1000.0f, 3);
    logFile.print(';');
    logFile.print(sample.gxDps10 / 10.0f, 2);
    logFile.print(';');
    logFile.print(sample.gyDps10 / 10.0f, 2);
    logFile.print(';');
    logFile.print(sample.gzDps10 / 10.0f, 2);
    logFile.print(';');
    logFile.print(sample.tempCd10 / 10.0f, 1);
    logFile.print(';');
    logFile.print(sample.pressureHd10 / 10.0f, 2);
    logFile.print(';');
    logFile.print(sample.altitudeCm / 100.0f, 2);
    logFile.print(';');
    logFile.print(sample.velocityCms / 100.0f, 2);
    logFile.print(';');
    logFile.print(sample.rollD10 / 10.0f, 1);
    logFile.print(';');
    logFile.print(sample.pitchD10 / 10.0f, 1);
    logFile.print(';');
    logFile.println(static_cast<unsigned>(sample.healthBits));
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
  if (profile == LedProfile::Calibrating) {
    pulse = (millis() % 500UL) < 250UL ? 1.0f : 0.0f;
  } else if (profile == LedProfile::Landed) {
    pulse = (millis() % 400UL) < 200UL ? 1.0f : 0.0f;
  } else if (profile != LedProfile::Fault) {
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
    case LedProfile::Calibrating:
      scaleAndSet(255, 255, 0);
      break;
    case LedProfile::Idle:
      scaleAndSet(0, 255, 96);
      break;
    case LedProfile::IdleUsb:
      scaleAndSet(0, 128, 255);
      break;
    case LedProfile::Idle1S:
      scaleAndSet(255, 224, 0);
      break;
    case LedProfile::Idle2S:
      scaleAndSet(255, 128, 0);
      break;
    case LedProfile::Coast:
      scaleAndSet(176, 0, 255);
      break;
    case LedProfile::Warning:
      scaleAndSet(255, 0, 192);
      break;
    case LedProfile::Landed:
      scaleAndSet(0, 220, 160);
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
      setLedProfile(LedProfile::Calibrating);
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
      {
        uint32_t now = millis();
        static uint32_t lastFaultToggleMs = 0;
        static bool faultLedOn = false;
        if (now - lastFaultToggleMs >= 150) {
          lastFaultToggleMs = now;
          faultLedOn = !faultLedOn;
        }
        setStatusLED(faultLedOn ? 255 : 0, 0, 0);
      }
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
  if (flightState == FlightState::Boost || flightState == FlightState::Coast) {
    if (currentAltitude > peakAltitude) {
      peakAltitude = currentAltitude;
      apogeePeakHoldCount = 0;
      apogeePeakHoldStartMs = millis();
    } else {
      uint32_t holdDeltaMs = millis() - apogeePeakHoldStartMs;
      if (holdDeltaMs <= APOGEE_FIRE_MAX_MS) {
        if (currentAltitude >= peakAltitude - 0.02f && fabsf(currentVerticalVelocity) <= 0.50f) {
          if (apogeePeakHoldCount < 255) {
            apogeePeakHoldCount++;
          }
        } else {
          apogeePeakHoldCount = 0;
          apogeePeakHoldStartMs = millis();
        }
      } else {
        apogeePeakHoldCount = 0;
        apogeePeakHoldStartMs = millis();
      }
    }
  }

  float rawVelocity = (currentAltitude - lastAltitudeForVelocity) / (SAMPLE_PERIOD_MS / 1000.0f);
  currentVerticalVelocity = (currentVerticalVelocity * 0.75f) + (rawVelocity * 0.25f);
  lastAltitudeForVelocity = currentAltitude;

  bool imuReliable = boardHealth.imuStreamOk && isfinite(accelMag) && isfinite(gravityProjection);
  bool baroReliable = boardHealth.baroOk && isfinite(currentAltitude) && isfinite(currentVerticalVelocity);
  bool flightFallbackAllowed = (flightState == FlightState::Boost || flightState == FlightState::Coast || flightState == FlightState::PyroFired);

  if (flightState == FlightState::Idle) {
    bool launchDetected = false;
    if (imuReliable) {
      launchDetected = (accelMag > flightLaunchThresholdG);
    } else if (baroReliable) {
      launchDetected = (currentVerticalVelocity > 1.0f) && (currentAltitude > 1.0f);
    }

    if (launchDetected) {
      if (launchConfirmCount < 255) {
        launchConfirmCount++;
      }
    } else {
      launchConfirmCount = 0;
    }

    if (launchConfirmCount >= LAUNCH_CONFIRM_SAMPLES) {
      launchMs = millis();
      launchConfirmCount = 0;
      peakAltitude = currentAltitude;
      apogeePeakHoldCount = 0;
      apogeePeakHoldStartMs = millis();
      startMissionLog();
      apogeeConfirmCount = 0;
      landingConfirmCount = 0;
      setFlightState(FlightState::Boost);
    }
  } else if (flightState == FlightState::Boost || flightState == FlightState::Coast) {
    if (flightState == FlightState::Boost) {
      bool boostDecayDetected = false;
      if (imuReliable) {
        boostDecayDetected = (accelMag < 1.15f);
      } else if (baroReliable) {
        boostDecayDetected = (currentVerticalVelocity <= 0.5f) && (currentAltitude > 2.0f);
      }

      if (boostDecayDetected) {
        if (boostDecayConfirmCount < 255) {
          boostDecayConfirmCount++;
        }
      } else {
        boostDecayConfirmCount = 0;
      }

      if (boostDecayConfirmCount >= BOOST_DECAY_CONFIRM_SAMPLES) {
        boostDecayConfirmCount = 0;
        setFlightState(FlightState::Coast);
      }
    }

    bool apogeeCandidate = false;
    if (baroReliable) {
      apogeeCandidate = (peakAltitude - currentAltitude) >= flightApogeeDropM &&
                       currentVerticalVelocity <= flightApogeeVelocityThresholdMPS &&
                       (imuReliable ? (accelMag <= APOGEE_ACCEL_MAX_G) : true) &&
                       (millis() - launchMs) >= LAUNCH_MIN_TIME_MS;
    } else if (imuReliable && flightFallbackAllowed) {
      apogeeCandidate = (accelMag <= APOGEE_ACCEL_MAX_G) &&
                       currentVerticalVelocity <= flightApogeeVelocityThresholdMPS &&
                       (millis() - launchMs) >= LAUNCH_MIN_TIME_MS;
    }

    if (apogeeCandidate) {
      if (apogeeConfirmCount < 255) {
        apogeeConfirmCount++;
      }
    } else {
      apogeeConfirmCount = 0;
    }

    bool peakHoldFire = flightSettings.firePyroAtApogee &&
                        apogeePeakHoldCount >= APOGEE_HOLD_SAMPLES &&
                        (millis() - apogeePeakHoldStartMs) <= APOGEE_FIRE_MAX_MS;

    if (flightSettings.firePyroAtApogee && apogeeConfirmCount >= APOGEE_CONFIRM_SAMPLES) {
      firePyro();
      apogeeConfirmCount = 0;
      apogeePeakHoldCount = 0;
      apogeePeakHoldStartMs = 0;
    } else if (peakHoldFire) {
      firePyro();
      apogeePeakHoldCount = 0;
      apogeePeakHoldStartMs = 0;
    }

    if (!flightSettings.firePyroAtApogee && apogeeConfirmCount >= APOGEE_CONFIRM_SAMPLES) {
      apogeeConfirmCount = 0;
    }

    if (!pyroLatched && (millis() - launchMs) > MAX_FLIGHT_TIME_MS) {
      firePyro();
    }
  }

  bool postLaunchState = flightState == FlightState::Boost ||
                         flightState == FlightState::Coast ||
                         flightState == FlightState::PyroFired;
  if (postLaunchState && missionStarted && !missionLogFlushed) {
    bool landingCandidate = false;
    bool stationary = accelMag <= 1.15f &&
                      magnitude3(currentGx - gyroBiasX,
                                 currentGy - gyroBiasY,
                                 currentGz - gyroBiasZ) <= STATIONARY_GYRO_TOLERANCE_DPS;
    if (baroReliable && imuReliable) {
      landingCandidate = (fabsf(currentVerticalVelocity) <= flightLandingVelocityToleranceMPS &&
                          stationary);
    } else if (baroReliable) {
      landingCandidate = (fabsf(currentVerticalVelocity) <= flightLandingVelocityToleranceMPS &&
                          (millis() - launchMs) > LAUNCH_MIN_TIME_MS);
    } else if (imuReliable) {
      landingCandidate = (stationary &&
                          (millis() - launchMs) > LAUNCH_MIN_TIME_MS);
    }

    if (landingCandidate) {
      if (landingConfirmCount < 255) {
        landingConfirmCount++;
      }
    } else {
      landingConfirmCount = 0;
    }

    if (landingConfirmCount >= LANDING_CONFIRM_SAMPLES) {
      setFlightState(FlightState::Landed);
    }
  }
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

  initializeAstroNavOtp();
  printAstroNavOtpSummary();

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
    configureUsbVolumeLabel();
    ensureLogsDirectory();
    loadFlightSettings();
    loadOrCreateDeviceProfile();
    ensureDeviceSerial();
    syncAssetsFromDeviceProfile();
    persistDeviceProfile();
    settingsNeedPersist = false;
  }

  if (powerMode == PowerMode::USB && storageReady && !usbFlightOverride) {
    FatFSUSB.onUnplug(onUsbStorageUnplug);
    usbDriveReady = FatFSUSB.begin();
    boardHealth.usbStorageOk = usbDriveReady;
    if (!usbDriveReady) {
      boardHealth.warning = true;
    }
    ensureUsbInfoFiles();
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
    setFlightState(FlightState::Fault);
  } else if (powerMode == PowerMode::USB && !usbFlightOverride) {
    setFlightState(FlightState::UsbMode);
  } else {
    setFlightState(FlightState::Calibrating);
    if (!calibratePadOrientation()) {
      boardHealth.critical = true;
      recordFaultAndSafeStop();
      setFlightState(FlightState::Fault);
    } else {
      setFlightState(FlightState::Idle);
      if (!missionStarted) {
        startMissionLog();
      }
    }
  }

  if (storageReady) {
    persistDeviceProfile();
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

  if (powerMode == PowerMode::USB && !usbFlightOverride && shouldExitUsbModeFromUsbFiles()) {
    enterFlightModeFromUsb();
  }

  if (now >= nextLedMs) {
    updateStatusLed();
    nextLedMs = now + LED_PERIOD_MS;
  }

  updatePyroOutput();

  if (boardHealth.critical) {
    setFlightState(FlightState::Fault);
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

    if ((boardHealth.imuStreamOk || boardHealth.baroOk) && !boardHealth.critical) {
      updateFlightStateFromSamples();
    }

    if (missionStarted) {
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

