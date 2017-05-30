#include "GrazynaV1.hpp"
#include "utils.hpp"

#include <SPI.h>
#include <I2Cdev.h>
#include <math.h>

const float GrazynaV1::MIN_SAFE_BATTERY_VOLTAGE = 6.4f;
const float GrazynaV1::MAX_DC_MOTORS_VOLTAGE = 6.0f;
GrazynaV1::GrazynaV1() :
m_motors(L1_PIN, L2_PIN, R1_PIN, R2_PIN, LPWM_PIN, RPWM_PIN),
m_pid(1,1,1,-50, 50),
m_batteryVoltage(0),
m_isOFF(false),
m_checkVoltageCounter(0),
m_sensorReadCounter(0),
m_kp(10), m_ki(3), m_kd(3), m_divider(2), m_loopMs(1),
m_targetAngle(0.0f)
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

    runEvery(m_loopMs){
      if( (++m_checkVoltageCounter % CHECK_BATTERY_STEPS) == 0)
      {
        m_batteryVoltage = getBatteryVoltage();
      }
      getSensorValues();
      calculateSensorValues();
      filterSensorValues();
      pwmCalculatePID();
      if(m_batteryVoltage > MIN_SAFE_BATTERY_VOLTAGE){
        motorsControl();
      }else{
        turnOFF();
        Serial.println("isOff");
        Serial.println(m_batteryVoltage);
      }

      tuning();

      // Serial.print(m_pid.getPTerm());
      // Serial.print(" ");
      // Serial.print(m_pid.getITerm());
      // Serial.print(" ");
      // Serial.print(m_pid.getDTerm());
      // Serial.print(" ");
      //
      // //printRawSensorValues();
      // Serial.print(m_accAngle);
      // Serial.print(" ");
      // Serial.print(m_filteredAccAngle);
      // Serial.println();
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
    m_pid.setKp(m_kp);
    m_pid.setKi(m_ki);
    m_pid.setKd(m_kd);
    int speed = m_pid.step(m_targetAngle, m_filteredAccAngle) / m_divider;
    speed = constrain(speed, -100, 100);  // -100 to 100 percent
    // speed = speed < 0 ? -20+speed : 20+speed;

    m_leftMotorSpeed = speed;
    m_rightMotorSpeed = speed;

    if(abs(m_filteredAccAngle) > 30){
      m_leftMotorSpeed = 0;
      m_rightMotorSpeed = 0;
    }
}

void GrazynaV1::motorsControl()
{
  unsigned leftSpeedPercent = mapPercentForMaxVoltage(abs(m_leftMotorSpeed), MAX_DC_MOTORS_VOLTAGE, m_batteryVoltage);
  unsigned rightSpeedPercent = mapPercentForMaxVoltage(abs(m_rightMotorSpeed), MAX_DC_MOTORS_VOLTAGE, m_batteryVoltage);
  //Serial.println(leftSpeedPercent);
  if(m_leftMotorSpeed >= 0)
  {
    if(m_leftMotorSpeed == 0)
      m_motors.leftStop();
    else
      m_motors.leftBackward();
  }
  else{
    m_motors.leftForward();
  }

  if(m_rightMotorSpeed >= 0)
  {
    if(m_rightMotorSpeed == 0)
      m_motors.rightStop();
    else
      m_motors.rightBackward();
  }
  else{
    m_motors.rightForward();
  }

  m_motors.setLeftSpeed(leftSpeedPercent);
  m_motors.setRightSpeed(rightSpeedPercent);
}

void GrazynaV1::tuning()
{
 if(!Serial.available())    return ;
 delay(10);
 char param = Serial.read();                            // get parameter byte
 if(!Serial.available())    return ;
 char cmd = Serial.read();                              // get command byte
 Serial.flush();
 switch (param) {
   case 'q':
     if(cmd=='+')    m_kp++;
     if(cmd=='-')    m_kp--;
     break;
   case 'w':
     if(cmd=='+')    m_ki++;
     if(cmd=='-')    m_ki--;
     break;
   case 'e':
     if(cmd=='+')    m_kd++;
     if(cmd=='-')    m_kd--;
     break;
   case 'r':
     if(cmd=='+')    m_divider++;
     if(cmd=='-')    m_divider--;
     break;
   case 't':
     if(cmd=='+')    m_targetAngle += 0.1f;
     if(cmd=='-')    m_targetAngle -= 0.1f;
     break;
   case 'y':
     if(cmd=='+')    m_loopMs++;
     if(cmd=='-')    m_loopMs--;
     break;
     case 'a':
       if(cmd=='+')    m_kp+=10;
       if(cmd=='-')    m_kp-=10;
       break;
     case 's':
       if(cmd=='+')    m_ki+=10;
       if(cmd=='-')    m_ki-=10;
       break;
     case 'd':
       if(cmd=='+')    m_kd+=10;
       if(cmd=='-')    m_kd-=10;
       break;
     case 'f':
       if(cmd=='+')    m_divider+=10;
       if(cmd=='-')    m_divider-=10;
       break;
     case 'g':
       if(cmd=='+')    m_targetAngle+=1.0f;
       if(cmd=='-')    m_targetAngle-=1.0f;
       break;
     case 'h':
       if(cmd=='+')    m_loopMs+=10;
       if(cmd=='-')    m_loopMs-=10;
       break;
   default:
     Serial.print("?");           Serial.print(param);
     Serial.print(" ?");          Serial.println(cmd);
   }

 Serial.println();
 Serial.print("    Kp:");       Serial.print(m_kp);
 Serial.print("    Ki:");       Serial.print(m_ki);
 Serial.print("    Kd:");       Serial.print(m_kd);
 Serial.print("   div:");       Serial.print(m_divider);
 Serial.print("target:");       Serial.print(m_targetAngle);
 Serial.print("loopms:");       Serial.println(m_loopMs);

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
