/*
    Unified data exchange format between Rover, Drone, and MissionControl
*/
#ifndef DEXCH 
#define DEXCH
#define MSG_STARTBYTE 0x51
#define FLIGHT_DISP_UPDATE_RATE 10
#define NAV_UPDATE_RATE 10
#define MSG_LEN_TYPE uint8_t
#define CHECKSUM_TYPE uint8_t
#define MSG_BUF_SIZE 128

#include <cstdint>
#include <type_traits>

enum MessageType : uint8_t {
    M_INFO,
    M_DEBUG,
    M_ERROR,
    M_CMD,
    M_NAV,
    M_SENS,
    M_FLIGHTDISP
};

enum MessageSource : uint8_t {
    ROVER,
    DRONE,
    MISSION_CTL,
    SERVER
};

#pragma pack(1)       // No padding! 

// Note: declare Payload types before Message types!
struct NavPayload { 
    float lon = 0.0;
    float lat = 0.0;
    int16_t heading = 0;
};

struct FlightDispPayload { 
    int16_t roll = 0;           // To be set: (int)100*roll. 
    int16_t pitch = 0;          // To be set: (int)100*pitch
    int8_t vspeed = 0;          
    int8_t gspeed = 0;
    int16_t alt = 0;
};

struct SensPayload { 
    uint8_t vol;
    int8_t temp;
};

unsigned int get_payload_len(MessageType mt);

// Message is serialized in this order
struct Message { 
    MSG_LEN_TYPE len = 0;
    MessageType type = M_INFO;
    MessageSource source = MISSION_CTL;
    uint8_t payload[MSG_BUF_SIZE];
    CHECKSUM_TYPE checksum = 0;
    bool valid = 1;

    Message() = default;
    
    template<typename PL_TYPE>
    Message(PL_TYPE pl);
    
    template<typename PL_TYPE>
    PL_TYPE get_data();  
};

// Template defs must be in header
// Missing these will lead to undefined reference error

template<typename PL_TYPE>
Message::Message(PL_TYPE pl){
    if constexpr (std::is_same<PL_TYPE, NavPayload>::value){
        type = M_NAV;
    } else if constexpr (std::is_same<PL_TYPE, FlightDispPayload>::value) {
        type = M_FLIGHTDISP;
    } else if constexpr (std::is_same<PL_TYPE, SensPayload>::value) {
        type=M_SENS;
    }
    // Further payload types here

    len = sizeof(Message) - sizeof(len) - sizeof(payload) - sizeof(valid) + get_payload_len(type);
    memcpy(payload, &pl, sizeof(PL_TYPE));
}

template<typename PL_TYPE>
PL_TYPE Message::get_data(){
    // WARNING: it is caller's responsibility to guarantee correct type!
    PL_TYPE result; 
    memcpy(&result, payload, sizeof(PL_TYPE));
    return result;
}

#pragma pack()

int serialize(uint8_t* buf, Message &m);

bool parse(Message &m, uint8_t data);

#endif