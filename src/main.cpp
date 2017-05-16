  #include <Arduino.h>
  #include "SPI.h"
  #include "I2Cdev.h"
  #include "MPU6050.h"

  MPU6050 accelgyro;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

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
    accelgyro.initialize();
    accelgyro.setXAccelOffset(-1801);
    accelgyro.setYAccelOffset(-923);
    accelgyro.setZAccelOffset(3238);
    accelgyro.setXGyroOffset(21);
    accelgyro.setYGyroOffset(-1);
    accelgyro.setZGyroOffset(31);
  }

  void loop()
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if(ay < 0)
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

      Serial.println(ay);

      analogWrite(9, 200.0f);
      analogWrite(10,75.0f);

    delay(50);
  }
