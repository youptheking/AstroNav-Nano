#pragma once

#include <Arduino.h>
#include <FS.h>

namespace UsbInfoFiles {
void writeHowto(File &file);
void writeWebsite(File &file);
}
