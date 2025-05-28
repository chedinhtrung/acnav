#ifndef REMOTE_SENSOR
#define REMOTE_SENSOR

#include <esp_now.h>
#include "quaternion.h"
#include "WiFi.h"

class RemoteSensor {
    static RemoteSensor* instance;
    Quaternion q; 
    unsigned long last_sync = -1;

    public:
    esp_err_t init();
    
    #ifdef REFERENCE
    public:
    uint8_t peer_mac_addr[6] = {0xA0,0xA3,0xB3,0x26,0x59,0x30};
    static void onDataReceive(const uint8_t* mac, const uint8_t* data, int len);
    esp_err_t sendSync();
    #endif

    #ifdef MEASURE
    public:
    uint8_t peer_mac_addr[6] = {0xD8,0xBC,0x38,0xE5,0xED,0x60};
    unsigned long last_stateupdate = -1;
    static void onSyncRecv(const uint8_t* mac, const uint8_t* data, int len);
    esp_err_t sendState(const Quaternion &quat);
    #endif
    
};

#endif