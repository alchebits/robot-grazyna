#ifndef GRAZYNA_V1_HPP
#define GRAZYNA_V1_HPP

#include <Arduino.h>
#include <MPU6050.h>
#include "L298NMotors.hpp"
#include "KalmanAngle.hpp"
#include "PID.hpp"
#include "Velocity.hpp"
#include "Bluetooth.hpp"


class GrazynaV1
{
public:
  static const uint16_t L1_PIN = 7;
  static const uint16_t L2_PIN = 8;
  static const uint16_t R1_PIN = 5;
  static const uint16_t R2_PIN = 6;
  static const uint16_t LPWM_PIN = 3;
  static const uint16_t RPWM_PIN = 9;
  static const uint16_t BLUETOOTH_RX_PIN = 11;
  static const uint16_t BLUETOOTH_TX_PIN = 10;

  static const uint16_t VOLTAGE_CHECK_PIN = A0;
  static const float MIN_SAFE_BATTERY_VOLTAGE;
  static const float MAX_DC_MOTORS_VOLTAGE;
  // static constexpr int16_t ACCELOFFSETS[6] = {-59,     -921,    1497,    70,      -7,      143};
  static constexpr int16_t ACCELOFFSETS[6] = {-80 ,    -898  , 1508   , 68     , -5     , 144};

  static const uint16_t CHECK_BATTERY_STEPS = 200;
  static const int32_t SENSOR_READ_MEAN_COUNTER = 1;
  static const uint8_t GRAZYNA_MPU6050_GYRO_FS = MPU6050_GYRO_FS_250;
  static const uint8_t GRAZYNA_MPU6050_ACCEL_FS = MPU6050_ACCEL_FS_2;

  static const float DEFAULT_TARGET_ANGLE;
  static const int BLUETOOT_TRANSFER_DELAY_MS = 200;

public:
  GrazynaV1();
  ~GrazynaV1();

  void setup();
  void loop();
protected:
  void getSensorValues();
  void calcLoopTime();
  void calculateSensorValues();
  void filterSensorValues();
  void pwmCalculatePID();
  void calculateVelocityX();
  void motorsControl();
  void bluetoothCommunication();

  float getBatteryVoltage() const;
  uint16_t mapPercentForMaxVoltage(unsigned percent, float maxDCVoltage, float batteryVoltage) const;
  void turnOFF();
  float getGyroRes(uint8_t mpu6050_gyro_fs);
  float getAccRes(uint8_t mpu6050_accel_fs);
  void printRawSensorValues();
protected:
  MPU6050 accel;
  L298NMotors m_motors;
  KalmanAngle m_kalman;
  PID m_pid;
  Velocity m_velocity;
  Bluetooth m_bluetooth;

  int32_t m_ax, m_ay, m_az;
  int32_t m_gx, m_gy, m_gz;
  int32_t m_buffAX, m_buffAY, m_buffAZ;
  int32_t m_buffGX, m_buffGY, m_buffGZ;
  float m_roll, m_pitch;
  float m_accAngle, m_gyroRate;
  float m_filteredAccAngle;
  uint64_t m_lastMillis, m_tmpMillis, m_btLastMillis, m_loopTimeMs;
  float m_batteryVoltage;
  bool m_isOFF;
  int16_t m_checkVoltageCounter;
  uint16_t m_sensorReadCounter;
  int m_leftMotorSpeed;
  int m_rightMotorSpeed;

  // adjustables
  int m_kp, m_ki, m_kd, m_divider;
  uint16_t m_loopMs;
  float m_targetAngle;
};


#endif
