#pragma once

#include <Arduino.h>
#include <SPI.h>

enum class FlightState;

struct ImuSample {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
};

struct BoardHealth {
  bool coreTickOk = false;
  bool heapOk = false;
  bool spiOk = false;
  bool imuWhoAmIOk = false;
  bool imuConfigOk = false;
  bool imuStreamOk = false;
  bool baroOk = false;
  bool flashFsOk = false;
  bool usbStorageOk = false;
  bool vinOk = false;
  bool warning = false;
  bool critical = false;
};

extern SPISettings spiSettings;
extern BoardHealth boardHealth;
extern float gravityAxisX;
extern float gravityAxisY;
extern float gravityAxisZ;
extern float gyroBiasX;
extern float gyroBiasY;
extern float gyroBiasZ;
extern float filteredTemperature;
extern float filteredPressure;
extern float filteredAltitude;
extern float baselinePressureHpa;
extern float currentTemperature;
extern float currentPressure;
extern float currentAltitude;
extern float currentVerticalVelocity;
extern float currentRollDeg;
extern float currentPitchDeg;
extern float currentAx;
extern float currentAy;
extern float currentAz;
extern float currentGx;
extern float currentGy;
extern float currentGz;
extern bool imuInitialized;
extern bool pyroLatched;
extern uint32_t bootMs;
extern uint32_t bootPulseUntilMs;
extern uint32_t pyroPulseUntilMs;

constexpr uint8_t PIN_MISO = 4;
constexpr uint8_t PIN_SCK = 6;
constexpr uint8_t PIN_MOSI = 7;
constexpr uint8_t PIN_CS_IMU = 9;
constexpr uint8_t PIN_CS_BMP = 12;
constexpr uint8_t PIN_PYRO = 1;
constexpr uint8_t PIN_SERVO_1 = 2;
constexpr uint8_t PIN_SERVO_2 = 3;
constexpr uint8_t PIN_VIN_SENSE = 27;
constexpr uint8_t WS2812_PIN = 0;

constexpr uint8_t ICM45686_EXPECTED_ID = 0xE9;
constexpr uint8_t ICM_REG_ACCEL_DATA_X1_UI = 0x00;
constexpr uint8_t ICM_REG_GYRO_DATA_X1_UI = 0x06;
constexpr uint8_t ICM_REG_TEMP_DATA1_UI = 0x0C;
constexpr uint8_t ICM_REG_PWR_MGMT0 = 0x10;
constexpr uint8_t ICM_REG_ACCEL_CONFIG0 = 0x1B;
constexpr uint8_t ICM_REG_GYRO_CONFIG0 = 0x1C;
constexpr uint8_t ICM_REG_REG_MISC2 = 0x7F;

bool testCoreTick();
bool testHeap();
bool readInputVoltage(float &voltage);
bool readBaroSample(float &temperature, float &pressureHpa);
bool readImuFrame(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
bool verifyImuStream();
bool configureImu();
bool calibratePadOrientation();
bool setFlightState(enum FlightState nextState);
void firePyro();
void updatePyroOutput();
void recordFaultAndSafeStop();
uint8_t readRegister(uint8_t csPin, uint8_t regAddr, SPISettings settings);
uint8_t readRegister(uint8_t csPin, uint8_t regAddr);
void writeRegister(uint8_t csPin, uint8_t regAddr, uint8_t value, SPISettings settings);
void writeRegister(uint8_t csPin, uint8_t regAddr, uint8_t value);
ImuSample decodeSample(const uint8_t *rawData);
