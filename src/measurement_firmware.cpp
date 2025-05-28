#include <Arduino.h>
#include <Wire.h>
#include "mpu6050.h"
#include "config.h"
#include "WiFi.h"
#include <quatern_eskf.h>
#include "remote_sensor.h"

#define EULER_DEBUG 1

hw_timer_t *timer = NULL;
volatile bool kf_run = false;
void IRAM_ATTR onKFTrigger(){
  kf_run = true;
}

QEskf eskf;

MPU6050_Imu imu;

RemoteSensor ref;

void setup() {
  WiFi.mode(WIFI_STA);

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onKFTrigger, true);
  timerAlarmWrite(timer, DT*1000, true);
  timerAlarmEnable(timer);

  Serial.begin(115200);
  
  Wire.begin(21, 22);
  Wire.setClock(400000);
  delay(4000);
  imu.setup();

  // Find MAC address
  //WiFi.mode(WIFI_STA);  // Important for getting the STA MAC
  //Serial.print("MAC Address (STA): ");
  //Serial.println(WiFi.macAddress());
  //delay(40000);

  ref.init();
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
    
    #ifdef EULER_DEBUG
    MSVector3 euler = eskf.state.q.to_euler()*(180.0f/M_PI);
    euler.print(buf);
    Serial.println(buf);
    #endif

    ref.sendState(eskf.state.q);
    
  }
    
 
}
