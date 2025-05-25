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
  eskf.KF_DT = DT*1e-3;
  /*
  eskf.state.state.q = Quaternion(1,0,0,0);
  eskf.state.state.ab = MSVector3(0,0,0);
  eskf.state.state.gb = MSVector3(0,0,0);
  */
  
}

unsigned long last_update = millis();
unsigned long last_gyro = micros();

unsigned long readtime = micros();

char buf[30];
void loop() {
  
  if (kf_run) {
    kf_run = false;
    unsigned long imu_last_read = imu.last_read;
    ImuData imud = imu.imu_read();
    eskf.propagate(imud.gyro, DT*1e-3);
    //eskf.update(imud.accel);
  }
  
  if (millis() - last_update > 30){
    last_update = millis();
    
    /*
    Serial.printf("%f %f %f %f", 
      eskf.state.state.q.w, eskf.state.state.q.i, eskf.state.state.q.j, eskf.state.state.q.k);
      */
    MSVector3 gv = eskf.state.state.q.T().rotate(MSVector3(0.0f,0.0f,1.0f));
    //gv.print(buf);
    //Serial.println(buf);
    //Serial.printf("%f %f %f %f %f %f %f %f %f",
    //eskf.P(0,0), eskf.P(1,1),eskf.P(2,2),eskf.P(3,3),eskf.P(4,4),eskf.P(5,5),eskf.P(6,6),eskf.P(7,7),eskf.P(8,8));
    //Serial.println();
    
  }
    
 
}
