# AstroNav Nano Firmware Documentation

## Overview

This firmware is the flight and safety controller for the AstroNav Nano board. Its purpose is to:

- detect board health at boot
- initialize the IMU and barometer
- classify the power source
- enter USB mass-storage mode only when USB power is present
- allow USB tuning of selected flight parameters
- detect launch, apogee, pyro firing, landing, and fault states
- continuously log as much sensor and state information as practical

The firmware is designed for reliability and safe behavior under real flight conditions, while keeping the configuration simple and user-accessible over USB.

## Power and USB behavior

### USB power detection

The board checks voltage on the VIN sense input before deciding whether it is running on USB power or battery power.

- USB range: 4.5 V to 5.5 V
- 1S LiPo: 3.0 V to 4.35 V
- 2S LiPo: 6.0 V to 8.4 V
- anything else: Unknown

The board only enters USB mass-storage mode when USB power is detected.

### USB ejection exit behavior

When the device is in USB mode and the host safely ejects or disconnects the drive, the firmware sets a USB exit request, and the board transitions into flight mode.

This is intentionally used as the device programming and launch preparation path while keeping the USB configuration surface simple.

## Flight state machine

The firmware uses the following states:

- Booting
- Calibrating
- Idle
- UsbMode
- Armed
- Boost
- Coast
- PyroFired
- Landed
- Fault

The state transitions are intentionally limited so that only valid transitions occur. A bad transition is treated as a warning and may be logged as a fault condition.

### State description

- Booting: startup health checks and low-level initialization
- Calibrating: IMU and barometer are used to establish a stable reference frame
- Idle: the board is ready for launch and waiting for launch-trigger conditions
- UsbMode: USB drive is active and settings can be edited
- Boost: launch acceleration is detected and flight is in ascent
- Coast: acceleration has dropped, flight is still in progress but no longer powered by boost
- PyroFired: apogee detection triggered pyro output
- Landed: landing confirmed from stable ground conditions
- Fault: critical fault, red blinking LED, system latched in a safe state

## Launch, apogee, and landing detection

Flight state changes are evaluated every sensor sample. The sensor loop runs at 100 ms, which gives about 10 samples per second. This is the current target cadence and is intentionally kept efficient so the system can respond quickly without excessive memory or CPU load.

### Launch detection

The board watches for sustained acceleration above the configured launch threshold.

- launch threshold parameter: LAUNCH_THRESHOLD_G
- default value: 1.35

Launch detection uses the IMU first. If the IMU is not trustworthy, the system falls back to the altitude/velocity trend to decide whether the craft is in launch.

### Apogee detection

Once the flight is in boost/coast, the code checks for:

- altitude drop from the peak value
- downward velocity below threshold
- low acceleration condition
- minimum flight time requirement

This is used to fire the pyro system at the correct time.

### Landing detection

After pyro is fired, landing confirmation requires the board to see that the craft has settled and is no longer descending quickly.

The system uses:

- low altitude
- low vertical velocity
- low acceleration
- gravity alignment check when available

The landing check is intentionally conservative. It uses multiple sensor cues and requires sustained confirmation before declaring Landed.

## Sensor fallback logic

The firmware is designed to use all available sensor values and to keep relying on the healthiest sources when one sensor misbehaves.

### IMU priority

The IMU is used for:

- launch detection
- orientation calculation
- acceleration magnitude
- gravity direction estimation

### Barometer priority

The barometer is used for:

- altitude estimation
- vertical velocity estimation
- apogee and landing confirmation

### Fallback behavior

When one sensor is noisy, stale, or invalid:

- the firmware uses the remaining valid sensor path to continue flight-state logic
- a bad sensor does not automatically force a fault unless the fault is critical
- the state logic keeps using whichever source is still healthy

This is important for reliability during short flights where a sensor can occasionally misread or drift.

## Logging strategy

The firmware logs as much useful information as possible.

### Mission log format

The CSV log includes:

- counter
- time in ms
- state
- power mode
- pyro flag
- acceleration axes
- gyro axes
- temperature
- pressure
- altitude
- vertical velocity
- roll and pitch
- health bits

This is designed to support post-flight debugging and data review.

### Logging cadence

The default sample period is 100 ms, which produces roughly 10 samples per second.

This is chosen to balance:

- data richness
- flash storage capacity
- response time for launch detection
- minimal impact on other system functions

## USB settings

The current USB settings remain intentionally small and practical:

- ESTIMATED_HEIGHT_M
- HEIGHT_MARGIN_M
- ESTIMATED_SPEED_MPS
- SPEED_MARGIN_MPS
- LAUNCH_THRESHOLD_G

These are the current user-editable parameters in the USB settings file. They are sufficient for the current product phase and leave space for later tuning extensions if needed.

The values are kept in the same structure as the current firmware design and are edited over USB without requiring a serial command interface.

## Fault handling and LED behavior

Faults are treated as high-priority events.

- fault state is latched
- pyro output is disabled safely
- status LED is red
- red LED blinks rapidly to attract attention

This is the primary visual signal that the board should not be launched when red blinking is active.

If a person ignores the warning and attempts launch anyway, the system still behaves safely and maintains fault-safe outputs.

## Safety principles

The design follows these priorities:

1. Safe default behavior
2. No unsafe state transitions
3. Use every good sensor available
4. Log every major flight event and state change
5. Keep the user-visible interface simple over USB
6. Keep the system ready for launch while never masking a real fault condition

## Compile in VS Code with PlatformIO

### 1) Install PlatformIO

In VS Code:

- open the Extensions view
- install the PlatformIO IDE extension
- reload VS Code after installation

### 2) Open the project folder

Open the repository folder in VS Code, then let PlatformIO detect the project.

### 3) Build the firmware

Use either:

- the PlatformIO toolbar in the left sidebar, or
- the terminal command:

```powershell
pio run
```

### 4) Upload to the board

Connect the RP2350 board and use the upload target:

```powershell
pio run --target upload --upload-port COM3
```

If your board is on a different COM port, replace `COM3` with the correct port shown in Device Manager or the PlatformIO port selector.

### 5) Monitor serial output

```powershell
pio device monitor
```

or from the PlatformIO toolbar.

### 6) Common issue on Windows

If PlatformIO complains that Git is missing when installing a dependency, make sure Git is installed and available on PATH. A common Windows path is:

```text
C:\Program Files\Git\cmd
```

After installing Git, reload VS Code and retry the build.

## Documentation note

The current source files are the source of truth for behavior. This README documents the intended product behavior and operation in a form that can be reviewed and extended later.
