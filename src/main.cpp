  #include <Arduino.h>
  #include "SPI.h"
  #include "I2Cdev.h"
  #include "MPU6050.h"
  #include "PID.h"
  #include <math.h>

//  #define PI 3.14159265

  #define runEvery(t) for (static long _lasttime;\
                           (uint16_t)((uint16_t)millis() - _lasttime) >= (t);\
                           _lasttime += (t))

 float Roll, Pitch;
 float offsetX = 0;

  MPU6050 accel;
  int ax, ay, az;
  int gx, gy, gz;

  void getGyroValues();
  void motorControl(double value);
  void setup()
  {
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);

    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);

    Wire.begin();
    Serial.begin(9600);
    accel.initialize();
    accel.setXAccelOffset(-66);
    accel.setYAccelOffset(-947);
    accel.setZAccelOffset(1503);
    accel.setXGyroOffset(31);
    accel.setYGyroOffset(5);
    accel.setZGyroOffset(88);

  }

  void loop()
  {
    runEvery(10){
      getGyroValues();
      int sign = (ax > 0) ? 1.0f : -1.0f;
      double val = Compute( Roll/3*sign );
      motorControl(val);
      Serial.println(val);

    }
  }

  void getGyroValues(){
    accel.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //  ax += 16384;
  //  az -= 16384;

    float X = ax / 16384.0f;
    float Y = ay / 16384.0f;
    float Z = -az / 16384.0f;

    Roll = atan2( sqrt(Y*Y + X*X), Z) * 180/M_PI;
    Pitch = atan2( sqrt(X*X + Z*Z), Y) * 180/M_PI;

    // Serial.print(Roll);
    // Serial.print(" = ");
    // Serial.print(Pitch);
    // Serial.println();
    //
    // Serial.print(ax);
    // Serial.print(" | ");
    // Serial.print(ay);
    // Serial.print(" | ");
    // Serial.print(az);
    // Serial.print(" == ");
    // Serial.print(gx);
    // Serial.print(" | ");
    // Serial.print(gy);
    // Serial.print(" | ");
    // Serial.println(gz);
  }

  void motorControl(double value)
  {
    if(value > 0)
    {
      digitalWrite(3, LOW);
      digitalWrite(4, HIGH);
      digitalWrite(5, LOW);
      digitalWrite(6, HIGH);
    }
    else
    {
      digitalWrite(4, LOW);
      digitalWrite(3, HIGH);
      digitalWrite(6, LOW);
      digitalWrite(5, HIGH);
    }

    value = max(255, abs(value));
    analogWrite(9, value); // 0-255 max
    analogWrite(10, value); // 0-255 max
  }
