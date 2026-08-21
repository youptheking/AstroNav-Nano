#include "usb_info_files.h"

namespace UsbInfoFiles {

static void printFloatValue(File &file, const char *key, float value, uint8_t digits) {
  file.print(key);
  file.print('=');
  file.print(value, digits);
  file.println();
}

void writeHowtoText(File &file) {
  file.println("AstroNav Nano USB Guide");
  file.println("Author: YoupSpace");
  file.println();
  file.println("What is on the drive:");
  file.println("- Howto.txt: this guide");
  file.println("- Settings.ini: one combined INI file with settings, profile, and debug sections");
  file.println("- Website.url: clickable website shortcut");
  file.println();
  file.println("How to leave USB mode:");
  file.println("1. Preferred: use Safely Remove Hardware / Eject in Windows.");
  file.println("2. Or send EXITUSB over the serial monitor.");
  file.println("3. Wait a moment for the board to switch to flight mode.");
  file.println();
  file.println("What the settings mean:");
  file.println("- ESTIMATED_HEIGHT_M: your best estimate of apogee height in meters.");
  file.println("  This helps the apogee detector know how much altitude drop to expect.");
  file.println("- HEIGHT_MARGIN_M: extra height margin around the estimate.");
  file.println("  Higher values make apogee and landing detection more forgiving.");
  file.println("- ESTIMATED_SPEED_MPS: expected climb speed in meters per second.");
  file.println("  This helps tune how quickly the apogee logic reacts.");
  file.println("- SPEED_MARGIN_MPS: extra speed margin around the estimate.");
  file.println("  Bigger values widen the safe window for speed checks.");
  file.println("- LAUNCH_THRESHOLD_G: acceleration needed before launch is confirmed.");
  file.println("  Lower values trigger earlier, higher values require a harder launch.");
  file.println("- FIRE_PYRO_APOGEE: enable pyro firing at apogee when TRUE.");
  file.println("  Set FALSE to keep logging the apogee event without firing the pyro at apogee.");
  file.println();
  file.println("How to edit Settings.ini:");
  file.println("1. Open Settings.ini in a text editor.");
  file.println("2. Change only the key=value lines you understand.");
  file.println("3. Save the file, then safely eject the drive.");
  file.println("4. The board reloads the settings on the next USB session or reboot.");
  file.println();
  file.println("Settings.ini sections:");
  file.println("- [files]: canonical file names and log locations.");
  file.println("- [settings]: editable flight tuning values.");
  file.println("- [profile]: device identity and stored flight history.");
  file.println("- [debug]: live health snapshot for the PC software.");
}

void writeWebsiteShortcut(File &file) {
  file.println("[InternetShortcut]");
  file.println("URL=https://youpspace.com/");
  file.println("IconIndex=0");
}

void writeAutorunInf(File &file) {
  file.println("[Autorun]");
  file.println("label=AstroNav Nano");
  // file.println("icon=Howto.txt");
  file.println("action=Open AstroNav Nano");
  file.println("open=Howto.txt");
}

void writeSettingsSection(File &file,
             float estimatedHeightM,
             float estimatedSpeedMps,
             float heightMarginM,
             float speedMarginMps,
             float launchThresholdG,
             bool firePyroAtApogee) {
  file.println("[settings]");
  file.println("; AstroNav Nano flight settings");
  file.println("; Edit the values below, then save the file.");
  file.println("; This file uses INI-style key/value pairs.");
  printFloatValue(file, "ESTIMATED_HEIGHT_M", estimatedHeightM, 1);
  printFloatValue(file, "HEIGHT_MARGIN_M", heightMarginM, 1);
  printFloatValue(file, "ESTIMATED_SPEED_MPS", estimatedSpeedMps, 1);
  printFloatValue(file, "SPEED_MARGIN_MPS", speedMarginMps, 1);
  printFloatValue(file, "LAUNCH_THRESHOLD_G", launchThresholdG, 2);
  file.printf("FIRE_PYRO_APOGEE=%s\n", firePyroAtApogee ? "TRUE" : "FALSE");
}

void writeDebugSection(File &file,
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
           const char *powerMode) {
  file.println("[debug]");
  file.printf("FAULT_REASON=%s\n", faultReason && faultReason[0] ? faultReason : "none");
  file.printf("RUNTIME_MODE=%s\n", runtimeMode ? runtimeMode : "unknown");
  file.printf("STATE=%s\n", state ? state : "unknown");
  file.printf("POWER_MODE=%s\n", powerMode ? powerMode : "unknown");
  printFloatValue(file, "VIN_VOLTAGE", vinVoltage, 3);
  file.printf("HEALTH_BITS=%u\n", healthBits);
  file.printf("HEALTHY=%s\n", (coreTickOk && heapOk && spiOk && imuWhoAmIOk && imuConfigOk && imuStreamOk && baroOk && !critical) ? "TRUE" : "FALSE");
  file.println();
  file.println("[checks]");
  file.printf("CORE_TICK_OK=%s\n", coreTickOk ? "TRUE" : "FALSE");
  file.printf("HEAP_OK=%s\n", heapOk ? "TRUE" : "FALSE");
  file.printf("SPI_OK=%s\n", spiOk ? "TRUE" : "FALSE");
  file.printf("IMU_WHOAMI_OK=%s\n", imuWhoAmIOk ? "TRUE" : "FALSE");
  file.printf("IMU_CONFIG_OK=%s\n", imuConfigOk ? "TRUE" : "FALSE");
  file.printf("IMU_STREAM_OK=%s\n", imuStreamOk ? "TRUE" : "FALSE");
  file.printf("BARO_OK=%s\n", baroOk ? "TRUE" : "FALSE");
  file.printf("FLASH_FS_OK=%s\n", flashFsOk ? "TRUE" : "FALSE");
  file.printf("USB_STORAGE_OK=%s\n", usbStorageOk ? "TRUE" : "FALSE");
  file.printf("VIN_OK=%s\n", vinOk ? "TRUE" : "FALSE");
  file.printf("WARNING=%s\n", warning ? "TRUE" : "FALSE");
  file.printf("CRITICAL=%s\n", critical ? "TRUE" : "FALSE");
}

}
