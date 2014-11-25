/**********************************************************************
     Author:   Grey Wind
     Date:     26/10/2014
     Version:  v0.1.0
     Details:
               26/10-v0.1.0: Created basic outline for UDP classes.
**********************************************************************/
#ifndef _UDP_WRAPPER_H
#define _UDP_WRAPPER_H



#ifdef WIN32

#include <winsock2.h>

#elif defined (linux)

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>

#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define closesocket(s) close(s)
typedef int SOCKET;
typedef struct sockaddr_in SOCKADDR_IN;
typedef struct sockaddr SOCKADDR;
typedef struct in_addr IN_ADDR;

#else

#error UDP driver not defined for this platform

#endif

#include <stdexcept>
#include <cstdio>
#include <cstring>
#include <string>

static void udp_init(void);
static void udp_end(void);

class UDPClient{
private: 
    SOCKET            m_socket;
    int               m_port;
    std::string       m_addr;
    struct addrinfo*  m_addrinfo;
    bool              m_server_knows_me;

public:
    //Constructor&Destructor
    UDPClient(const std::string& addr, int port);
    ~UDPClient();

    //Getters
    int          get_socket() const;
    int          get_port() const;
    std::string  get_addr() const;

    //Methods
    int          send(const char *msg, size_t size);
    int          recv(char *msg, size_t max_size);
};

class UDPServer{
private:
    SOCKET            m_socket;
    int               m_port;
    std::string       m_addr;
    struct addrinfo*  m_addrinfo;
    bool              m_client_is_known;
    SOCKADDR_IN       m_client;
    int               m_fromsize;

public:
    //Constructor&Destructor
    UDPServer(const std::string& addr, int port);
    ~UDPServer();

    //Getters
    int          get_socket() const;
    int          get_port() const;
    std::string  get_addr() const;

    //Methods
    int          recv(char* msg, size_t max_size);
    int          send(const char* msg, size_t size);
    int          send(const char* msg, size_t size, const std::string& addr, int port);
};

#endif  //_UDP_WRAPPER_H
