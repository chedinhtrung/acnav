#ifndef NETWORK 
#define NETWORK
#define WIN32_LEAN_AND_MEAN

#define PLATFORM_UNIX   // For Unix build. Change to PLATFORM_WIN if windows because it uses a different lib for sockets
#ifdef PLATFORM_WIN
#include <winsock2.h>
#include <ws2tcpip.h>
#endif

#ifdef PLATFORM_UNIX
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>       // for close()
#include <string.h> 
#endif

#include "dataexchange.h"
#include <queue>

class DataChannel {
    public: 
        

};

class UdpDataChannel {
    public: 
        
};

#endif