# AstroNav Nano Self-Test Software Design

This document describes the firmware in enough detail that the current program can be recreated from it.

## Audience and scope

This design guide is intended for people who want to understand what the firmware is doing, how it boots, how it validates hardware, and how the build metadata and production tooling fit into the project.

This document is helpful for:

- new developers joining the project
- hardware testers validating the board
- production teams using the optional manufacturing metadata flow
- anyone debugging a boot, health, or flight-state issue

This document is not a substitute for the actual safety procedures and launch protocol. It explains the software and hardware behavior, not the legal or operational risk decisions.

## Chapter map

1. Purpose and system overview
2. Hardware assumptions and pin map
3. Power and input-voltage detection
4. Build-time metadata and production signing
5. Persistent state and startup flow
6. Hardware validation and test functions
7. Flight-state logic and sensor behavior
8. USB configuration and operational notes

## Purpose

The firmware is a board health and wiring verification program for the AstroNav RP2350A-based PCB. It boots, initializes the board peripherals, checks the IMU and barometer, measures the board input voltage through a resistor divider on GPIO27, reports the detected power source, and keeps a status LED updated with overall board state.

The project also includes a platform build step that generates build metadata and optional production signing. This is handled by [generate_version.py](generate_version.py), which writes [include/build_info.h](include/build_info.h) during compilation.

The program is intentionally simple and deterministic:

- It does not control mission logic.
- It does not stream sensor data externally.
- It performs self-test and continuous health reporting only.

## Hardware Assumptions

The program assumes:

- MCU platform: Raspberry Pi Pico 2 / RP2350 running the Arduino framework under PlatformIO.
- SPI bus is used for the IMU and BMP581.
- A single WS2812 status LED is attached.
- GPIO27 is connected to an input-voltage sense divider.
- GPIO1 is a pyro output that must be held low.
- GPIO2 and GPIO3 are servo outputs that must be held low.

## Pin Map

### SPI and peripheral pins

- GPIO4: MISO
- GPIO6: SCK
- GPIO7: MOSI
- GPIO9: IMU chip select
- GPIO12: BMP581 chip select
- GPIO0: WS2812 status LED

### Additional control pins

- GPIO1: pyro channel, output low at boot
- GPIO2: servo output, output low at boot
- GPIO3: servo output, output low at boot
- GPIO27: input-voltage sense

## Input Voltage Sensing

GPIO27 measures a divided version of the board input rail.

### Divider values

- Top resistor: 20k from input rail to the sense node
- Bottom resistor: 10k from sense node to ground
- GPIO27 is connected to the sense node

### Electrical meaning

The sense pin sees one third of the real input voltage:

- `V_sense = V_in / 3`
- `V_in = V_sense * 3`

The firmware treats this reading as the board input rail before the regulator. It is not a regulated 3.3 V rail reading.

### Source classification

The measured input rail is classified into one of three expected power sources:

- USB power: 4.5 V to 5.5 V
- 1S LiPo: 3.0 V to 4.35 V
- 2S LiPo: 6.0 V to 8.4 V

Anything outside those ranges is reported as `Unknown`.

## Libraries Used

The firmware includes:

- Arduino core
- SPI
- Adafruit NeoPixel
- SparkFun BMP581 Arduino Library
- math.h

## Build-time Metadata and Production Signing

Before the firmware is compiled, the Python build script validates the git tag, generates the version string, and writes the header file that exposes build metadata to the firmware.

Optional production metadata:

- A local secrets directory is checked for a manufacturing key file: warranty_key.txt
- A serial registry CSV is checked for serial tracking: serial_registry.csv
- If the key or registry are absent, the build still succeeds and prints a warning instead of failing
- If the files are present and valid, the script uses them to generate official warranty signatures and serial registration data

This allows open-source or local builds to compile without the manufacturing assets, while keeping the production path intact when those files are available.

## Persistent State

The program stores a small set of global runtime state:

- `boardHealth`: boolean summary of each test and overall warning/critical state
- `lastSummaryMs`: timestamp for periodic console summaries
- `bootWhiteUntilMs`: initial LED hold time after boot
- current IMU sample values
- current barometer values
- current measured input voltage
- current detected power source text

## Constants

### Sensor scaling

The IMU raw values are converted using fixed scale factors:

- Accelerometer: 1024 counts per g
- Gyroscope: 16.4 counts per deg/s

These numbers are used after reading raw 16-bit sensor data.

### ADC conversion

The voltage-sense code assumes a 12-bit ADC reading and a 3.3 V ADC reference.

- ADC counts range: 0 to 4095
- ADC reference: 3.3 V

The input voltage is computed from the measured sense voltage and divider ratio.

### Flash signature

The flash test uses the constant string `ASTRONAV_PCB_TEST` as a checksum source.

## Startup Flow

The setup sequence is important and should be preserved.

1. Start Serial at 115200.
2. Initialize the WS2812 LED.
3. Set LED brightness to 40.
4. Set the LED state to Busy.
5. Hold the boot-white state for about 2 seconds using `bootWhiteUntilMs`.
6. Delay 400 ms.
7. Set ADC resolution to 12 bits.
8. Wait up to 3 seconds for Serial connection.
9. Print a banner to the console.
10. Configure GPIO1, GPIO2, and GPIO3 as outputs and drive them low.
11. Configure the SPI chip-select pins as outputs.
12. Configure GPIO27 as input.
13. Put the chip-select pins high by default.
14. Pulse the BMP581 chip-select low then high to force SPI mode.
15. Configure SPI pins and start the SPI peripheral.
16. Run core tick, flash, and heap tests.
17. Read and classify input voltage from GPIO27.
18. Initialize the BMP581 and read one sample.
19. Detect and configure the IMU.
20. Verify the IMU data stream by reading several samples.
21. Read one final IMU frame into the live status variables.
22. Compute warning and critical status flags.
23. Print a boot summary.
24. Delay 500 ms.
25. Update the status LED.
26. Store the current time in `lastSummaryMs`.

## Boot-Time Pin Behavior

At boot, the firmware intentionally forces the following outputs low:

- GPIO1 low
- GPIO2 low
- GPIO3 low

This is done immediately after the console banner, before other peripheral initialization, so those outputs do not float or accidentally activate.

## Test Functions

### Core tick test

A simple time test verifies that `millis()` advances across a short delay.

Expected result:

- `millis()` after a 2 ms delay must be greater than the starting value.

### Flash read test

The flash test XORs every byte in the flash signature string.

Expected result:

- The checksum must be non-zero.

This is a lightweight code-integrity / storage sanity check rather than a full flash readback test.

### Heap test

The heap test allocates 256 bytes with `malloc`, fills the buffer with a known pattern, verifies a few bytes, and frees it.

Expected result:

- Allocation succeeds
- First, second, and last test bytes match the pattern

## Voltage Measurement Algorithm

The code reads GPIO27 multiple times and averages the samples.

### Algorithm

1. Take 8 ADC samples.
2. Add the raw counts into a 32-bit total.
3. Compute the average counts.
4. Convert counts to sense voltage:
   - `senseVoltage = averageCounts * 3.3 / 4095`
5. Convert sense voltage to input voltage:
   - `inputVoltage = senseVoltage * 3`
6. Return the measured input voltage if it is greater than 0.1 V.

### Result handling

The measured voltage is stored in `currentVinVoltage`.
The corresponding source label is stored in `currentPowerSource`.

## Power Source Classification Rules

The measured input voltage is mapped to a source label:

- 4.5 V to 5.5 V -> `USB power`
- 3.0 V to 4.35 V -> `1S LiPo`
- 6.0 V to 8.4 V -> `2S LiPo`
- otherwise -> `Unknown`

If the source is `Unknown`, the firmware sets the warning flag.

## IMU Behavior

The IMU uses SPI and register-level access.

### WHO_AM_I / transport detection

The IMU transport is probed by reading register `0x72` using two possible SPI modes:

- SPI mode 3 first
- SPI mode 0 second

The first valid non-`0x00`, non-`0xFF` response is considered transport OK and also selects the working SPI mode.

The WHO_AM_I value must equal `0xE9`.

### IMU configuration sequence

After confirming WHO_AM_I, the firmware writes:

- `REG_MISC2 = 0x02`
- `PWR_MGMT0 = 0x0F`
- `ACCEL_CONFIG0 = 0x06`
- `GYRO_CONFIG0 = 0x06`

Then it reads those registers back and requires them to match.

### IMU sample decoding

The firmware reads 12 bytes starting at accelerometer data register `0x00`.

The bytes are interpreted as little-endian 16-bit signed values:

- accel X: bytes 0-1
- accel Y: bytes 2-3
- accel Z: bytes 4-5
- gyro X: bytes 6-7
- gyro Y: bytes 8-9
- gyro Z: bytes 10-11

Then the raw counts are converted to physical units:

- accel in g
- gyro in deg/s

### IMU stream verification

The firmware reads 8 samples and counts how many are finite and not clearly invalid.

Expected result:

- At least 6 valid samples out of 8

## Barometer Behavior

The BMP581 is initialized over SPI after the SPI bus is configured.

The firmware reads sensor data using the library call, converts pressure from Pa to hPa, and accepts the reading only if pressure is in a plausible atmospheric range.

Expected result:

- Temperature and pressure read successfully
- Pressure is between 300 hPa and 1200 hPa

## Health Model

The health summary uses two higher-level state flags.

### Warning

Warning is set when any non-critical monitored subsystem is degraded, including:

- barometer read failure
- IMU stream failure
- unknown input-source classification

### Critical

Critical is set when any fundamental board function fails:

- core tick
- flash check
- heap test
- SPI transport
- IMU WHO_AM_I
- IMU configuration
- IMU stream

Critical overrides warning behavior in LED selection.

## LED Behavior

The WS2812 LED is the user-visible health indicator.

### States

- Busy: white during boot hold period
- Healthy: green
- Warning: orange
- Critical: red

### State selection

The LED is updated by `updateHealthLed()`:

1. If the boot-white timer is still active, show Busy.
2. Else if critical, show Critical.
3. Else if warning, show Warning.
4. Else show Healthy.

## Serial Output

The firmware prints both boot-time and runtime summaries.

### Boot summary

The boot output lists:

- core tick
- flash read
- heap alloc
- IMU transport
- IMU WHO_AM_I
- IMU config
- IMU stream
- barometer
- VIN/input rail and classified power source
- PASS / WARN / FAIL result

### Runtime summary

Once per second, the firmware prints a condensed line containing:

- overall health
- individual health fields
- measured VIN and classified source
- current accel, gyro, temperature, and pressure values

## Main Loop Behavior

The loop runs continuously.

Per cycle it does the following:

1. Re-read input voltage from GPIO27 and refresh the source label.
2. If the IMU is initialized, read a new IMU frame.
3. If the IMU frame is valid, update the live accel/gyro values.
4. Read the barometer.
5. Recompute warning and critical flags.
6. Update the LED.
7. Print a runtime summary every 1 second.
8. Delay 10 ms.

## Pseudocode

```text
setup():
  start serial
  init LED
  set LED busy
  start boot timer
  set ADC to 12-bit
  wait for serial briefly
  print banner

  configure GPIO1, GPIO2, GPIO3 as outputs
  drive GPIO1 low
  drive GPIO2 low
  drive GPIO3 low

  configure chip selects and VIN sense pin
  force BMP581 into SPI mode
  configure SPI pins and start SPI

  run core tick test
  run flash test
  run heap test

  read VIN input voltage
  classify power source

  init barometer and read sample
  configure IMU and verify stream

  compute warning/critical
  print boot summary
  update LED

loop():
  read VIN input voltage
  classify power source

  if IMU initialized:
    read IMU frame
    if valid:
      update live IMU values
    else:
      mark IMU stream failed

  read barometer
  recompute warning/critical
  update LED

  every 1 s:
    print runtime summary

  delay 10 ms
```

## Rebuild Notes

To recreate this program from scratch, implement the following in the same order:

1. Define all pins and constants.
2. Create global health and sensor state.
3. Implement LED state helpers.
4. Implement the core tick, flash, and heap tests.
5. Implement GPIO27 voltage reading and source classification.
6. Implement SPI register helpers.
7. Implement IMU frame decoding and stream verification.
8. Implement IMU configuration.
9. Implement barometer reading.
10. Implement boot and runtime summary printing.
11. Configure outputs low in `setup()` before peripheral work.
12. Initialize SPI and sensors.
13. Update health flags and LED in the loop.

## Build Configuration

The project is configured in PlatformIO for the `rpipico2` environment using the Arduino framework.

Libraries declared in `platformio.ini`:

- Adafruit NeoPixel
- SparkFun BMP581 Arduino Library
- ICM-42688

## Practical Meaning of the VIN Reading

The voltage read from GPIO27 is the board input rail, not the regulated MCU rail.

This means it tells you which supply is present:

- USB power should read around 5 V
- 1S LiPo should read around 3.0 to 4.35 V
- 2S LiPo should read around 6.0 to 8.4 V

That is the value the firmware uses for user reporting and source detection.
