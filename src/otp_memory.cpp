#include "otp_memory.h"

#include <Arduino.h>
#include <string.h>

#include <boot/bootrom_constants.h>
#include <pico/bootrom.h>

#include "build_info.h"

namespace {

static constexpr uint16_t kAstroNavOtpStartRow = 0x00C0;
static constexpr size_t kAstroNavOtpRowSizeBytes = 3;
static constexpr size_t kAstroNavOtpRowCount = (sizeof(AstroNav_OTP_Data) + kAstroNavOtpRowSizeBytes - 1) / kAstroNavOtpRowSizeBytes;
static constexpr uint32_t kAstroNavMagicHeader = 0xA57B09A2u;
static constexpr char kSerialAlphabet[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
static constexpr size_t kReservedProductionSecondIndex = 0;
static constexpr size_t kReservedAheadCountIndex = 1;
static constexpr size_t kReservedCommitStartIndex = 2;
static constexpr size_t kReservedCommitLength = 7;

AstroNav_OTP_Data cachedOtpData = {};
AstroNavOtpStatus otpStatus = {
  false,
  false,
  false,
  false,
  false,
  false,
  false,
  false,
  ASTRONAV_BUILD_WARRANTY_SIGNATURE_OFFICIAL != 0,
  BOOTROM_OK,
};

otp_cmd_t makeOtpCommand(uint16_t startRow, bool isWrite) {
  otp_cmd_t command = {static_cast<uint32_t>(startRow)};
  if (isWrite) {
    command.flags |= OTP_CMD_WRITE_BITS;
  }
  return command;
}

void packOtpRows(const AstroNav_OTP_Data &data, uint32_t *rows) {
  uint8_t packedBytes[sizeof(AstroNav_OTP_Data)] = {0};
  memcpy(packedBytes, &data, sizeof(data));

  for (size_t rowIndex = 0; rowIndex < kAstroNavOtpRowCount; rowIndex++) {
    size_t offset = rowIndex * kAstroNavOtpRowSizeBytes;
    uint32_t rowValue = packedBytes[offset];
    if (offset + 1 < sizeof(packedBytes)) {
      rowValue |= static_cast<uint32_t>(packedBytes[offset + 1]) << 8;
    }
    if (offset + 2 < sizeof(packedBytes)) {
      rowValue |= static_cast<uint32_t>(packedBytes[offset + 2]) << 16;
    }
    rows[rowIndex] = rowValue;
  }
}

void unpackOtpRows(const uint32_t *rows, AstroNav_OTP_Data &data) {
  uint8_t packedBytes[sizeof(AstroNav_OTP_Data)] = {0};

  for (size_t rowIndex = 0; rowIndex < kAstroNavOtpRowCount; rowIndex++) {
    size_t offset = rowIndex * kAstroNavOtpRowSizeBytes;
    uint32_t rowValue = rows[rowIndex] & 0x00FFFFFFu;
    packedBytes[offset] = static_cast<uint8_t>(rowValue & 0xFFu);
    if (offset + 1 < sizeof(packedBytes)) {
      packedBytes[offset + 1] = static_cast<uint8_t>((rowValue >> 8) & 0xFFu);
    }
    if (offset + 2 < sizeof(packedBytes)) {
      packedBytes[offset + 2] = static_cast<uint8_t>((rowValue >> 16) & 0xFFu);
    }
  }

  memcpy(&data, packedBytes, sizeof(data));
}

AstroNav_OTP_Data buildExpectedOtpData() {
  AstroNav_OTP_Data data = {};
  data.magic_header = kAstroNavMagicHeader;
  memcpy(data.manufacturer_id, "YOUPSPACE", sizeof("YOUPSPACE"));
  memcpy(data.product_id, "NANO", sizeof("NANO"));
  data.hardware_major = ASTRONAV_BUILD_HARDWARE_MAJOR;
  data.hardware_minor = ASTRONAV_BUILD_HARDWARE_MINOR;
  memcpy(data.production_date, ASTRONAV_BUILD_PRODUCTION_DATE, sizeof(ASTRONAV_BUILD_PRODUCTION_DATE));
  data.serial_number = ASTRONAV_BUILD_SERIAL_NUMBER;
  data.initial_firmware[0] = ASTRONAV_BUILD_INITIAL_FIRMWARE_MAJOR;
  data.initial_firmware[1] = ASTRONAV_BUILD_INITIAL_FIRMWARE_MINOR;
  data.initial_firmware[2] = ASTRONAV_BUILD_INITIAL_FIRMWARE_PATCH;
  data.warranty_signature = ASTRONAV_BUILD_WARRANTY_SIGNATURE;
  memset(data.reserved, 0, sizeof(data.reserved));
  data.reserved[kReservedProductionSecondIndex] = static_cast<uint8_t>(ASTRONAV_BUILD_PRODUCTION_SECOND);
  data.reserved[kReservedAheadCountIndex] = static_cast<uint8_t>(ASTRONAV_BUILD_FIRMWARE_AHEAD_COUNT & 0xFF);

  const char *commit = ASTRONAV_BUILD_FIRMWARE_COMMIT_SHA;
  for (size_t index = 0; index < kReservedCommitLength && commit[index] != '\0'; index++) {
    char ch = commit[index];
    if (ch >= 'A' && ch <= 'F') {
      ch = static_cast<char>(ch - 'A' + 'a');
    }
    data.reserved[kReservedCommitStartIndex + index] = static_cast<uint8_t>(ch);
  }
  return data;
}

bool accessOtpRows(uint32_t *rows, bool isWrite, int &errorCode) {
  int result = rom_func_otp_access(
    reinterpret_cast<uint8_t *>(rows),
    static_cast<uint32_t>(kAstroNavOtpRowCount * sizeof(uint32_t)),
    makeOtpCommand(kAstroNavOtpStartRow, isWrite));
  errorCode = result;
  return result == BOOTROM_OK;
}

void logOtpStatus(const char *message, int errorCode) {
  Serial.printf("[OTP] %s (rc=%d)\n", message, errorCode);
}

void printReservedBytes(const uint8_t *bytes, size_t length) {
  for (size_t index = 0; index < length; index++) {
    Serial.printf("%02X", static_cast<unsigned int>(bytes[index]));
    if (index + 1 < length) {
      Serial.print(' ');
    }
  }
  Serial.println();
}

bool isAsciiHexChar(char value) {
  return (value >= '0' && value <= '9')
      || (value >= 'a' && value <= 'f')
      || (value >= 'A' && value <= 'F');
}

bool hasValidCommitCodeInReserved(const AstroNav_OTP_Data &data) {
  for (size_t index = 0; index < kReservedCommitLength; index++) {
    char value = static_cast<char>(data.reserved[kReservedCommitStartIndex + index]);
    if (!isAsciiHexChar(value)) {
      return false;
    }
  }
  return true;
}

bool isOtpBlank(const AstroNav_OTP_Data &data) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&data);
  for (size_t index = 0; index < sizeof(data); index++) {
    if (bytes[index] != 0) {
      return false;
    }
  }
  return true;
}

bool isValidOtpPayload(const AstroNav_OTP_Data &data) {
  const uint8_t *reserved = data.reserved;
  bool legacyReserved = true;
  for (size_t index = 1; index < sizeof(data.reserved); index++) {
    if (reserved[index] != 0) {
      legacyReserved = false;
      break;
    }
  }
  return data.magic_header == kAstroNavMagicHeader &&
         memcmp(data.manufacturer_id, "YOUPSPACE\0", sizeof(data.manufacturer_id)) == 0 &&
         memcmp(data.product_id, "NANO\0", sizeof(data.product_id)) == 0 &&
         data.hardware_major == ASTRONAV_BUILD_HARDWARE_MAJOR &&
         data.hardware_minor == ASTRONAV_BUILD_HARDWARE_MINOR &&
         data.production_date[sizeof(data.production_date) - 1] == '\0' &&
         data.serial_number != 0 &&
         data.warranty_signature != 0 &&
         (legacyReserved || hasValidCommitCodeInReserved(data)) &&
         reserved[9] == 0 && reserved[10] == 0 && reserved[11] == 0 &&
         reserved[12] == 0 && reserved[13] == 0 && reserved[14] == 0 && reserved[15] == 0;
}

void readCommitCodeFromReserved(const AstroNav_OTP_Data &data, char *buffer, size_t buffer_size) {
  if (!buffer || buffer_size == 0) {
    return;
  }

  size_t copyLen = (kReservedCommitLength < (buffer_size - 1)) ? kReservedCommitLength : (buffer_size - 1);
  for (size_t index = 0; index < copyLen; index++) {
    char ch = static_cast<char>(data.reserved[kReservedCommitStartIndex + index]);
    if (ch >= 'A' && ch <= 'F') {
      ch = static_cast<char>(ch - 'A' + 'a');
    }
    buffer[index] = ch;
  }
  buffer[copyLen] = '\0';
}

}

bool readAstroNavOtpData(AstroNav_OTP_Data &data) {
  alignas(4) uint32_t rows[kAstroNavOtpRowCount] = {0};
  int errorCode = BOOTROM_OK;
  if (!accessOtpRows(rows, false, errorCode)) {
    otpStatus.read_ok = false;
    otpStatus.blank = false;
    otpStatus.payload_valid = false;
    otpStatus.write_blocked = true;
    otpStatus.has_magic_header = false;
    otpStatus.last_error = errorCode;
    return false;
  }

  unpackOtpRows(rows, data);
  cachedOtpData = data;
  otpStatus.read_ok = true;
  otpStatus.blank = isOtpBlank(data);
  otpStatus.has_magic_header = data.magic_header == kAstroNavMagicHeader;
  otpStatus.payload_valid = isValidOtpPayload(data);
  otpStatus.signature_matches_build = otpStatus.has_magic_header && data.warranty_signature == ASTRONAV_BUILD_WARRANTY_SIGNATURE;
  otpStatus.last_error = BOOTROM_OK;
  return true;
}

bool astroNavOtpHasValidHeader() {
  AstroNav_OTP_Data data = {};
  return readAstroNavOtpData(data) && otpStatus.payload_valid;
}

bool astroNavOtpGetSerialNumber(uint32_t &serial_number) {
  AstroNav_OTP_Data data = {};
  if (!readAstroNavOtpData(data) || !otpStatus.payload_valid) {
    return false;
  }

  serial_number = data.serial_number;
  return true;
}

void formatAstroNavDisplaySerial(uint32_t serial_number, char *buffer, size_t buffer_size) {
  if (!buffer || buffer_size == 0) {
    return;
  }

  char encoded[11] = {0};
  uint32_t state = serial_number;
  bool hasDigit = false;
  bool hasLetter = false;
  const uint32_t alphabetLength = static_cast<uint32_t>(sizeof(kSerialAlphabet) - 1);

  for (int index = 0; index < 10; index++) {
    state = static_cast<uint32_t>((1664525u * state) + 1013904223u + static_cast<uint32_t>(index));
    char ch = kSerialAlphabet[state % alphabetLength];
    encoded[index] = ch;
    if (ch >= '0' && ch <= '9') {
      hasDigit = true;
    } else {
      hasLetter = true;
    }
  }

  if (!hasLetter) {
    encoded[1] = 'A';
  }
  if (!hasDigit) {
    encoded[2] = '7';
  }

  snprintf(buffer, buffer_size, "AN-%s", encoded);
}

void formatAstroNavOtpInitialFirmwareVersion(const AstroNav_OTP_Data &data, char *buffer, size_t buffer_size) {
  if (!buffer || buffer_size == 0) {
    return;
  }

  const unsigned int major = static_cast<unsigned int>(data.initial_firmware[0]);
  const unsigned int minor = static_cast<unsigned int>(data.initial_firmware[1]);
  const unsigned int patch = static_cast<unsigned int>(data.initial_firmware[2]);
  const unsigned int ahead = static_cast<unsigned int>(data.reserved[kReservedAheadCountIndex]);

  if (hasValidCommitCodeInReserved(data)) {
    char commit[8] = {0};
    readCommitCodeFromReserved(data, commit, sizeof(commit));
    snprintf(buffer, buffer_size, "V%u.%u.%u_%u_%s", major, minor, patch, ahead, commit);
    return;
  }

  snprintf(buffer, buffer_size, "V%u.%u.%u", major, minor, patch);
}

bool astroNavOtpWasProgrammedThisBoot() {
  return otpStatus.write_succeeded;
}

void initializeAstroNavOtp() {
  AstroNav_OTP_Data currentData = {};
  if (!readAstroNavOtpData(currentData)) {
    logOtpStatus("OTP read failed. Refusing to write OTP.", otpStatus.last_error);
    return;
  }

  if (otpStatus.payload_valid) {
    logOtpStatus("Valid AstroNav OTP payload already present. Skipping write.", BOOTROM_OK);
    return;
  }

  if (!otpStatus.blank || ASTRONAV_BUILD_SERIAL_NUMBER == 0) {
    otpStatus.write_blocked = true;
    logOtpStatus(otpStatus.blank
      ? "Blank OTP found, but no production serial is present. Refusing to write."
      : "Nonblank invalid OTP payload found. Refusing to rewrite OTP.", BOOTROM_OK);
    return;
  }

  AstroNav_OTP_Data expectedData = buildExpectedOtpData();
  alignas(4) uint32_t rows[kAstroNavOtpRowCount] = {0};
  packOtpRows(expectedData, rows);

  otpStatus.write_attempted = true;
  int errorCode = BOOTROM_OK;
  if (!accessOtpRows(rows, true, errorCode)) {
    otpStatus.write_succeeded = false;
    otpStatus.last_error = errorCode;
    logOtpStatus("OTP write skipped or failed. Continuing flight controller startup.", errorCode);
    return;
  }

  otpStatus.write_succeeded = true;
  if (readAstroNavOtpData(currentData) && otpStatus.payload_valid) {
    logOtpStatus("OTP payload programmed successfully.", BOOTROM_OK);
  } else {
    otpStatus.write_succeeded = false;
    logOtpStatus("OTP write completed but verification read did not confirm the header. Continuing startup.", otpStatus.last_error);
  }
}

void printAstroNavOtpSummary() {
  AstroNav_OTP_Data data = {};
  if (!readAstroNavOtpData(data)) {
    Serial.printf("[OTP] Read failed. bootrom_rc=%d\n", otpStatus.last_error);
    return;
  }

  char displaySerial[16] = {0};
  formatAstroNavDisplaySerial(data.serial_number, displaySerial, sizeof(displaySerial));

  Serial.println("[OTP] AstroNav OTP summary");
  Serial.printf("[OTP] magic_header=0x%08lX\n", static_cast<unsigned long>(data.magic_header));
  Serial.printf("[OTP] manufacturer_id=%s\n", data.manufacturer_id);
  Serial.printf("[OTP] product_id=%s\n", data.product_id);
  Serial.printf("[OTP] hardware_major=%u\n", static_cast<unsigned int>(data.hardware_major));
  Serial.printf("[OTP] hardware_minor=%u\n", static_cast<unsigned int>(data.hardware_minor));
  Serial.printf("[OTP] production_date=%s\n", data.production_date);
  Serial.printf("[OTP] production_second=%u\n", static_cast<unsigned int>(data.reserved[kReservedProductionSecondIndex]));
  Serial.printf("[OTP] production_timestamp=%s-%02u\n", data.production_date, static_cast<unsigned int>(data.reserved[kReservedProductionSecondIndex]));
  Serial.printf("[OTP] serial_number=%s\n", displaySerial);
  char initialFirmwareVersion[28] = {0};
  formatAstroNavOtpInitialFirmwareVersion(data, initialFirmwareVersion, sizeof(initialFirmwareVersion));
  Serial.printf("[OTP] first_firmware_version=%s\n", initialFirmwareVersion);
  Serial.printf("[OTP] initial_firmware=%u.%u.%u\n",
                static_cast<unsigned int>(data.initial_firmware[0]),
                static_cast<unsigned int>(data.initial_firmware[1]),
                static_cast<unsigned int>(data.initial_firmware[2]));
  Serial.printf("[OTP] warranty_signature=0x%08lX\n", static_cast<unsigned long>(data.warranty_signature));
  Serial.printf("[OTP] magic_header_valid=%s\n", data.magic_header == kAstroNavMagicHeader ? "YES" : "NO");
  Serial.printf("[OTP] payload_valid=%s\n", otpStatus.payload_valid ? "YES" : "NO");
  Serial.printf("[OTP] blank=%s\n", otpStatus.blank ? "YES" : "NO");
  Serial.printf("[OTP] write_blocked=%s\n", otpStatus.write_blocked ? "YES" : "NO");
  Serial.printf("[OTP] profile_matches_current_build=%s\n", otpStatus.signature_matches_build ? "YES" : "NO");
  Serial.println("[OTP] note: OTP is immutable after first write, so this can be NO on newer firmware builds.");
  Serial.printf("[OTP] official_build_signature=%s\n", otpStatus.official_signature ? "YES" : "NO");
  Serial.printf("[OTP] programmed_this_boot=%s\n", otpStatus.write_succeeded ? "YES" : "NO");
  Serial.print("[OTP] reserved=");
  printReservedBytes(data.reserved, sizeof(data.reserved));
}

const AstroNavOtpStatus &getAstroNavOtpStatus() {
  return otpStatus;
}