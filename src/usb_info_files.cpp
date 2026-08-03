#include "usb_info_files.h"

namespace UsbInfoFiles {

void writeHowto(File &file) {
  file.println("AstroNav Nano USB Guide");
  file.println("Author: YoupSpace");
  file.println();
  file.println("What is on the drive:");
  file.println("- HOWTO.TXT: user guide");
  file.println("- SETTINGS.TXT: values you can edit and save");
  file.println("- PERSONAL.TXT: read-only copy of the device profile stored in flash");
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
  file.println();
  file.println("Extra files on the drive:");
  file.println("- PERSONAL.TXT: flash-backed device profile copy");
  file.println("- WEBSITE.TXT: your site link");
  file.println();
  file.println("PERSONAL.TXT is regenerated from flash-backed data and is only for viewing.");
}

void writeWebsite(File &file) {
  file.println("YoupSpace website");
  file.println("https://youpspace.com/");
  file.println();
  file.println("Open this link in your browser.");
}

}
