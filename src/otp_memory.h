#pragma once

#include <stddef.h>
#include <stdint.h>

struct __attribute__((packed)) AstroNav_OTP_Data {
  uint32_t magic_header;
  char manufacturer_id[10];
  char product_id[5];
  uint8_t hardware_major;
  uint8_t hardware_minor;
  char production_date[17];
  uint32_t serial_number;
  uint8_t initial_firmware[3];
  uint32_t warranty_signature;
  uint8_t reserved[16];
};

struct AstroNavOtpStatus {
  bool read_ok;
  bool blank;
  bool payload_valid;
  bool has_magic_header;
  bool write_attempted;
  bool write_succeeded;
  bool write_blocked;
  bool signature_matches_build;
  bool official_signature;
  int last_error;
};

static_assert(sizeof(AstroNav_OTP_Data) == 65, "AstroNav OTP payload size must stay fixed.");

void initializeAstroNavOtp();
bool readAstroNavOtpData(AstroNav_OTP_Data &data);
bool astroNavOtpHasValidHeader();
bool astroNavOtpGetSerialNumber(uint32_t &serial_number);
void formatAstroNavDisplaySerial(uint32_t serial_number, char *buffer, size_t buffer_size);
void formatAstroNavOtpInitialFirmwareVersion(const AstroNav_OTP_Data &data, char *buffer, size_t buffer_size);
bool astroNavOtpWasProgrammedThisBoot();
void printAstroNavOtpSummary();
const AstroNavOtpStatus &getAstroNavOtpStatus();