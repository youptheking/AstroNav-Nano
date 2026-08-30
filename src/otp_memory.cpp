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

AstroNav_OTP_Data cachedOtpData = {};
AstroNavOtpStatus otpStatus = {
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
  data.reserved[0] = static_cast<uint8_t>(ASTRONAV_BUILD_PRODUCTION_SECOND);
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

}

bool readAstroNavOtpData(AstroNav_OTP_Data &data) {
  alignas(4) uint32_t rows[kAstroNavOtpRowCount] = {0};
  int errorCode = BOOTROM_OK;
  if (!accessOtpRows(rows, false, errorCode)) {
    otpStatus.read_ok = false;
    otpStatus.has_magic_header = false;
    otpStatus.last_error = errorCode;
    return false;
  }

  unpackOtpRows(rows, data);
  cachedOtpData = data;
  otpStatus.read_ok = true;
  otpStatus.has_magic_header = data.magic_header == kAstroNavMagicHeader;
  otpStatus.signature_matches_build = otpStatus.has_magic_header && data.warranty_signature == ASTRONAV_BUILD_WARRANTY_SIGNATURE;
  otpStatus.last_error = BOOTROM_OK;
  return true;
}

bool astroNavOtpHasValidHeader() {
  AstroNav_OTP_Data data = {};
  return readAstroNavOtpData(data) && data.magic_header == kAstroNavMagicHeader;
}

bool astroNavOtpGetSerialNumber(uint32_t &serial_number) {
  AstroNav_OTP_Data data = {};
  if (!readAstroNavOtpData(data) || data.magic_header != kAstroNavMagicHeader) {
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

bool astroNavOtpWasProgrammedThisBoot() {
  return otpStatus.write_succeeded;
}

void initializeAstroNavOtp() {
  AstroNav_OTP_Data currentData = {};
  if (readAstroNavOtpData(currentData) && currentData.magic_header == kAstroNavMagicHeader) {
    logOtpStatus("Valid AstroNav OTP payload already present. Skipping write.", BOOTROM_OK);
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
  if (readAstroNavOtpData(currentData) && currentData.magic_header == kAstroNavMagicHeader) {
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
  Serial.printf("[OTP] production_second=%u\n", static_cast<unsigned int>(data.reserved[0]));
  Serial.printf("[OTP] production_timestamp=%s-%02u\n", data.production_date, static_cast<unsigned int>(data.reserved[0]));
  Serial.printf("[OTP] serial_number=%s\n", displaySerial);
  Serial.printf("[OTP] initial_firmware=%u.%u.%u\n",
                static_cast<unsigned int>(data.initial_firmware[0]),
                static_cast<unsigned int>(data.initial_firmware[1]),
                static_cast<unsigned int>(data.initial_firmware[2]));
  Serial.printf("[OTP] warranty_signature=0x%08lX\n", static_cast<unsigned long>(data.warranty_signature));
  Serial.printf("[OTP] magic_header_valid=%s\n", data.magic_header == kAstroNavMagicHeader ? "YES" : "NO");
  Serial.printf("[OTP] signature_matches_build=%s\n", otpStatus.signature_matches_build ? "YES" : "NO");
  Serial.printf("[OTP] official_build_signature=%s\n", otpStatus.official_signature ? "YES" : "NO");
  Serial.printf("[OTP] programmed_this_boot=%s\n", otpStatus.write_succeeded ? "YES" : "NO");
  Serial.print("[OTP] reserved=");
  printReservedBytes(data.reserved, sizeof(data.reserved));
}

const AstroNavOtpStatus &getAstroNavOtpStatus() {
  return otpStatus;
}