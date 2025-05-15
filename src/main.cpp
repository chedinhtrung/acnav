#include <Arduino.h>
#include <Wire.h>
#include "mpu6050.h"
#include "config.h"

MPU6050_Imu imu;

float angle = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000);
  delay(4000);
  imu.setup();
}

unsigned long last_update = millis();
unsigned long last_gyro = micros();

unsigned long readtime = micros();
unsigned long delta = micros();

void loop() {
  if (micros() - imu.last_read > DT*1e3) {
    ImuData imud = imu.imu_read();
    angle += imud.gyro.z * DT * 1e-3;
  }
  
  if (millis() - last_update > 30){
    last_update = millis();
    Serial.printf("DT %i ", delta);
    Serial.println(angle, 4);
  }
 
}
