#include "GrazynaV1.hpp"
#include "utils.hpp"
#include "PID.h"

#include <SPI.h>
#include <I2Cdev.h>
#include <math.h>

const float GrazynaV1::MIN_SAFE_BATTERY_VOLTAGE = 6.4f;
const float GrazynaV1::MAX_DC_MOTORS_VOLTAGE = 5.0f;
GrazynaV1::GrazynaV1() :
m_motors(L1_PIN, L2_PIN, R1_PIN, R2_PIN, LPWM_PIN, RPWM_PIN),
m_batteryVoltage(0),
m_isOFF(false),
m_checkVoltageCounter(0),
m_sensorReadCounter(0)
{
}

GrazynaV1::~GrazynaV1()
{
}

void GrazynaV1::setup()
{
  Serial.begin(9600);
  pinMode(VOLTAGE_CHECK_PIN, INPUT);

  accel.initialize();
  accel.setFullScaleGyroRange(GRAZYNA_MPU6050_GYRO_FS);         //0 = +/- 250 degrees/sec | 1 = +/- 500 degrees/sec | 2 = +/- 1000 degrees/sec | 3 =  +/- 2000 degrees/sec
  accel.setFullScaleAccelRange(GRAZYNA_MPU6050_ACCEL_FS);        // MPU6050_ACCEL_FS_2 = +/- 2g | MPU6050_ACCEL_FS_4 = +/- 4g | MPU6050_ACCEL_FS_8 = +/- 8g | MPU6050_ACCEL_FS_16 =  +/- 16g
  accel.setXAccelOffset(ACCELOFFSETS[0]);
  accel.setYAccelOffset(ACCELOFFSETS[1]);
  accel.setZAccelOffset(ACCELOFFSETS[2]);
  accel.setXGyroOffset(ACCELOFFSETS[3]);
  accel.setYGyroOffset(ACCELOFFSETS[4]);
  accel.setZGyroOffset(ACCELOFFSETS[5]);

  m_batteryVoltage = getBatteryVoltage();
  m_lastMillis = millis();
}

void GrazynaV1::loop()
{

    runEvery(50){
      if( (++m_checkVoltageCounter % CHECK_BATTERY_STEPS) == 0)
      {
        m_batteryVoltage = getBatteryVoltage();
        if(m_batteryVoltage < MIN_SAFE_BATTERY_VOLTAGE)
        {
          turnOFF();
        }
      }
      if(m_isOFF){
        // too lov battery voltage detected
        //Serial.println("IS OFF");
        //return;
      }

      getSensorValues();
      //printRawSensorValues();
      calculateSensorValues();
      filterSensorValues();

      Serial.print(m_accAngle);
      Serial.print(" ");
      Serial.println(m_filteredAccAngle);
      pwmCalculatePID();
      motorsControl();
  }
}

void GrazynaV1::getSensorValues(){
  int16_t ax, ay, az, gx, gy, gz;
  for(int16_t i = 0 ; i < SENSOR_READ_MEAN_COUNTER ; i++)
  {
      accel.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      m_buffAX += ax;
      m_buffAY += ay;
      m_buffAZ += az;
      m_buffGX += gx;
      m_buffGY += gy;
      m_buffGZ += gz;
  }

    m_ax = m_buffAX / SENSOR_READ_MEAN_COUNTER;
    m_ay = m_buffAY / SENSOR_READ_MEAN_COUNTER;
    m_az = m_buffAZ / SENSOR_READ_MEAN_COUNTER;
    m_gx = m_buffGX / SENSOR_READ_MEAN_COUNTER;
    m_gy = m_buffGY / SENSOR_READ_MEAN_COUNTER;
    m_gz = m_buffGZ / SENSOR_READ_MEAN_COUNTER;
    m_buffAX = m_buffAY = m_buffAZ = m_buffGX = m_buffGY = m_buffGZ = 0;
}

void GrazynaV1::calculateSensorValues()
{
  float accRes = getAccRes(GRAZYNA_MPU6050_ACCEL_FS);
  float xG = m_ax * accRes;  // 1G value == 9.8 m/sec^2
  float yG = m_ay * accRes;  // 1G value == 9.8 m/sec^2
  float zG = m_az * accRes;  // 1G value == 9.8 m/sec^2

  m_roll = atan2( sqrt(xG*xG + yG*yG), zG) * 180/M_PI; // lewo prawo 0 to gora
  m_pitch = atan2( sqrt(yG*yG + zG*zG), xG) * 180/M_PI; // przod tyl 90 to gora

  float gyroRes = getGyroRes(GRAZYNA_MPU6050_GYRO_FS);
  float degPerSecY = m_gy * gyroRes;

  m_accAngle = atan2(-zG, -xG) * 180/M_PI + 90;
  m_gyroRate = degPerSecY;
}

void GrazynaV1::filterSensorValues()
{
  m_tmpMillis = millis();
  uint32_t deltaMillis = m_tmpMillis - m_lastMillis;
  m_lastMillis = m_tmpMillis;

  m_filteredAccAngle = m_kalman.calculate(m_accAngle, m_gyroRate, deltaMillis);
}

void GrazynaV1::pwmCalculatePID()
{
    // double val = Compute(m_pitch-89);
}

void GrazynaV1::motorsControl()
{
  // unsigned speedPercent = mapPercentForMaxVoltage(abs(val), MAX_DC_MOTORS_VOLTAGE, m_batteryVoltage);
  // if(val >= 0)
  //   m_motors.backward();
  // else
  // m_motors.forward();
  // int calculatedSpeedPercent = mapPercentForMaxVoltage(100, MAX_DC_MOTORS_VOLTAGE, m_batteryVoltage);
  // Serial.println(calculatedSpeedPercent);
  // m_motors.setSpeed(calculatedSpeedPercent);
}

float GrazynaV1::getBatteryVoltage() const
{
  return ((5.0f*analogRead(VOLTAGE_CHECK_PIN)) / 1023.0f) * 2; // voltage divider is used
}

unsigned GrazynaV1::mapPercentForMaxVoltage(unsigned percent,  float maxDCVoltage, float batteryVoltage) const
{
  return (maxDCVoltage/batteryVoltage) * percent;
}

void GrazynaV1::turnOFF()
{
  m_isOFF = true;
  m_motors.setSpeed(0);
  m_motors.stop();
}

float GrazynaV1::getGyroRes(uint8_t mpu6050_gyro_fs)
{
  switch(mpu6050_gyro_fs)
  {
    case MPU6050_GYRO_FS_250:
      return 250/32768.0f;
    case MPU6050_GYRO_FS_500:
      return 500/32768.0f;
    case MPU6050_GYRO_FS_1000:
      return 1000/32768.0f;
    case MPU6050_GYRO_FS_2000:
      return 2000/32768.0f;
    default:
      return 1;
  }
}

float GrazynaV1::getAccRes(uint8_t mpu6050_accel_fs)
{
  switch(mpu6050_accel_fs)
  {
    case MPU6050_ACCEL_FS_2:
      return 2.0f/32768.0f;
    case MPU6050_ACCEL_FS_4:
      return 4.0f/32768.0f;
    case MPU6050_ACCEL_FS_8:
      return 8.0f/32768.0f;
    case MPU6050_ACCEL_FS_16:
      return 16.0f/32768.0f;
    default:
      return 1.0f;
  }
}

void GrazynaV1::printRawSensorValues()
{
  Serial.println(m_ax);
  Serial.print(", ");
  Serial.print(m_ay);
  Serial.print(", ");
  Serial.print(m_az);
  Serial.print(" | ");
  Serial.print(m_gx);
  Serial.print(", ");
  Serial.print(m_gy);
  Serial.print(", ");
  Serial.print(m_gz);
  Serial.println();
}
