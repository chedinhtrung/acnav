#include <Arduino.h>
#include <Wire.h>
#include "mpu6050.h"
#include "config.h"

#include <quatern_eskf.h>

hw_timer_t *timer = NULL;
volatile bool kf_run = false;
void IRAM_ATTR onKFTrigger(){
  kf_run = true;
}

QEskf eskf;

MPU6050_Imu imu;

float angle = 0;

void setup() {
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onKFTrigger, true);
  timerAlarmWrite(timer, DT*1000, true);
  timerAlarmEnable(timer);

  Serial.begin(115200);
  
  Wire.begin(21, 22);
  Wire.setClock(400000);
  delay(4000);
  imu.setup();
  /*
  eskf.state.state.q = Quaternion(1,0,0,0);
  eskf.state.state.ab = MSVector3(0,0,0);
  eskf.state.state.gb = MSVector3(0,0,0);
  */
  
}

unsigned long last_update = millis();

unsigned long readtime = micros();

unsigned long delta = 0;

unsigned long euler_time = micros();

char buf[30];
void loop() {
  
  if (kf_run) {
    readtime = micros();
    kf_run = false;
    ImuData imud = imu.imu_read();
    //Serial.write((byte*)&imud, sizeof(ImuData));
    eskf.propagate(imud.gyro, DT*1e-3);
    eskf.update(imud.accel);
    delta = micros()-readtime;
  }
  
  if (millis() - last_update > 50){
    last_update = millis();
    euler_time = micros();
    MSVector3 euler = eskf.state.q.to_euler()*(180.0f/M_PI);

    unsigned long delta_euler = micros() - euler_time;
    Serial.printf("%i  %i ", delta, delta_euler);
    euler.print(buf);
    Serial.println(buf);
    
  }
    
 
}
