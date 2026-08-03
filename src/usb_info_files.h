#pragma once

#include <Arduino.h>
#include <FS.h>

namespace UsbInfoFiles {
void writeHowtoIni(File &file);
void writeWebsiteShortcut(File &file);
void writeSettingsIni(File &file,
					  float estimatedHeightM,
					  float estimatedSpeedMps,
					  float heightMarginM,
					  float speedMarginMps,
					  float launchThresholdG);
void writeDebugIni(File &file,
				  const char *faultReason,
				  bool coreTickOk,
				  bool heapOk,
				  bool spiOk,
				  bool imuWhoAmIOk,
				  bool imuConfigOk,
				  bool imuStreamOk,
				  bool baroOk,
				  bool flashFsOk,
				  bool usbStorageOk,
				  bool vinOk,
				  bool warning,
				  bool critical,
				  float vinVoltage,
				  uint8_t healthBits,
				  const char *runtimeMode,
				  const char *state,
				  const char *powerMode);
}
