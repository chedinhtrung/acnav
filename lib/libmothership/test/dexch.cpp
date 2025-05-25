#include "dataexchange.h"
#include <iostream>
#include <cstring> 

int main() {
    
    // Test message format
    NavPayload payload;
    payload.heading = 1000;
    payload.lat = 10;
    payload.lon = 10;

    Message m(payload);

    std::cout << "message length " << (unsigned int)m.len << "\n";
    NavPayload decoded_payload = m.get_data<NavPayload>();
    printf("Decoded payload heading:%i, lat:%f, lon:%f \n", 
        decoded_payload.heading, decoded_payload.lat, decoded_payload.lon);


    
    // Test message serialization
    uint8_t buf[MSG_BUF_SIZE];
    int len = serialize(buf, m);
    std::cout << "Serialized:" << len << "\n";  //payload = 10B, type = 1B, source = 1B, len = 1B, checksum = 1B => 14B 

    
    // Test parsing
    
    Message new_msg;
    bool valid = false;
    parse(new_msg, MSG_STARTBYTE);
    for (uint8_t* ptr = buf; ptr < buf+len; ptr++){
        valid = parse(new_msg, *ptr);
    } 
    if (!valid){
        std::cout << "Invalid msg";
        return 1;
    }
    printf("Len: %i, Type: %i, Source: %i, ", 
            m.len, m.type, m.source);
    NavPayload new_nav = m.get_data<NavPayload>();
    
    printf("lon: %f, lat: %f, head: %i", new_nav.lon, new_nav.lat, new_nav.heading);
    
    return 0;
}