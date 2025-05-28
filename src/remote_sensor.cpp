#include "remote_sensor.h"
#include "Arduino.h"

RemoteSensor* RemoteSensor::instance = nullptr;

esp_err_t RemoteSensor::init(){
    
    instance = this;
    esp_err_t initsuccess = esp_now_init();
    if (initsuccess != ESP_OK){
        Serial.println("espnow init fail");
        return initsuccess;
    }

    esp_now_peer_info_t peerInfo = {};
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    memcpy(&(peerInfo.peer_addr), &peer_mac_addr, 6); 
    esp_err_t add_peer_success = esp_now_add_peer(&peerInfo);
    if (add_peer_success != ESP_OK){
         Serial.println("espnow add peer fail");
        return add_peer_success;
    }
    
    #ifdef REFERENCE
    esp_now_register_recv_cb(onDataReceive);
    #endif
    
    #ifdef MEASURE
    esp_now_register_recv_cb(onSyncRecv);
    #endif
    return add_peer_success;
}

#ifdef REFERENCE

void RemoteSensor::onDataReceive(const uint8_t* mac, const uint8_t* data, int len){
    if (len == sizeof(Quaternion)){
        memcpy(&(instance->q), data, sizeof(Quaternion));
        MSVector3 euler = instance->q.to_euler()*(180.0f/M_PI);
        char buf[50];
        euler.print(buf);
        Serial.println(buf);
    }
}

#endif

#ifdef MEASURE

esp_err_t RemoteSensor::sendState(const Quaternion &quat){
    return esp_now_send(peer_mac_addr, (uint8_t*)&quat, sizeof(Quaternion));
}

void RemoteSensor::onSyncRecv(const uint8_t* mac, const uint8_t* data, int len){

}

#endif