#include "dataexchange.h"
#include <cstring>
#include <iostream>

/*
Serialized Message structure: 

| StartByte | len  | type | source | --- Payload --- | Checksum |
    1B         1B     1B      1B          len             1B

valid is not serialized, hence not considered in Message::get_payload_len()
*/

unsigned int get_payload_len(MessageType mt){
    switch (mt){
        case M_NAV:
            return sizeof(NavPayload);
        case M_FLIGHTDISP:
            return sizeof(FlightDispPayload);
        
        // Further message types here! 
        default: 
            return 0;
    }
};

int serialize(uint8_t* buf, Message &m){
    // WARNING: function does NOT check that buf is at least MSG_BUF_SIZE.

    int parsed = 0;
    unsigned int payload_len = get_payload_len(m.type);

    memset(buf, 0, MSG_BUF_SIZE);
    uint8_t* buf0 = buf;
    // Header
    memcpy(buf, &m.len, sizeof(MSG_LEN_TYPE)); buf += sizeof(MSG_LEN_TYPE); parsed += sizeof(MSG_LEN_TYPE);
    memcpy(buf, &m.type, sizeof(MessageType)); buf += sizeof(MessageType); parsed += sizeof(MessageType);
    memcpy(buf, &m.source, sizeof(MessageSource)); buf += sizeof(MessageSource); parsed += sizeof(MessageSource);
    // Payload
    memcpy(buf, m.payload, payload_len); buf += payload_len; parsed += payload_len;
    // Checksum
    uint8_t checksum = 0;
    for (uint8_t* ptr = buf0; ptr < buf0 + parsed; ptr++){
        checksum ^= *ptr;
    }; 
    m.checksum = checksum;
    memcpy(buf, &m.checksum, sizeof(CHECKSUM_TYPE)); buf += sizeof(CHECKSUM_TYPE); parsed += sizeof(CHECKSUM_TYPE);
    // Move buffer ptr back
    buf -= parsed;
    return parsed; // Returns the actual length of the buffer 
}

bool parse(Message &m, uint8_t data){
    /*
        Parse next byte of the message (meant for UART use)
    */
    static int M_PARSE_BYTECOUNT = 0;
    static uint8_t M_PARSE_SUM = 0;
    static int M_PARSE_STEP = 0;
    MSG_LEN_TYPE payload_len;
    switch (M_PARSE_STEP){
        case 0:
            if (data == MSG_STARTBYTE){
                M_PARSE_STEP++;
                return 0;
            }
        case 1: 
            memcpy(&m.len + M_PARSE_BYTECOUNT, &data, 1);
            M_PARSE_BYTECOUNT++; M_PARSE_SUM ^= data;
            if (M_PARSE_BYTECOUNT == sizeof(MSG_LEN_TYPE)){
                M_PARSE_BYTECOUNT = 0;
                M_PARSE_STEP++;
            } 
            return 0;
            
        case 2: 
            m.type = (MessageType)data; M_PARSE_SUM ^= data;
            M_PARSE_STEP++;
            return 0;

        case 3: 
            m.source = (MessageSource)data; M_PARSE_SUM ^= data;
            M_PARSE_STEP++;
            return 0;
    
        case 4: 
            payload_len = get_payload_len(m.type);
            memcpy(&m.payload + M_PARSE_BYTECOUNT, &data, 1);
            M_PARSE_BYTECOUNT += 1; M_PARSE_SUM ^= data;
            if (M_PARSE_BYTECOUNT == payload_len){
                M_PARSE_STEP++;
                M_PARSE_BYTECOUNT = 0;
            }
            return 0;
        
        case 5:
            m.checksum = data;
            M_PARSE_BYTECOUNT = 0;
            M_PARSE_STEP = 0;
            // Checksum
            bool isvalid;
            isvalid = M_PARSE_SUM == m.checksum; 
            M_PARSE_SUM = 0;
            m.valid = isvalid;
            return isvalid;
        
        default: 
            return 0;
    }
}




