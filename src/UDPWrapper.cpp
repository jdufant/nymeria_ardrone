/**********************************************************************
     Author:   Grey Wind
     Date:     26/10/2014
     Version:  v0.1.0
     Details:
               26/10-v0.1.0: Created basic outline for UDP classes.
**********************************************************************/
#include <nymeria_ardrone/UDPWrapper.h>

static void udp_init(void){
#ifdef WIN32

    WSADATA wsa;
    int err = WSAStartup(MAKEWORD(2, 2), &wsa);
    if(err < 0){
        throw std::runtime_error("WSAStartup failed!");
    }

#endif
}

static void udp_end(void){
#ifdef WIN32
    WSACleanup();
#endif
}


UDPClient::UDPClient(const std::string& addr, int port): m_port(port), m_addr(addr){
    char decimal_port[16];
    snprintf(decimal_port, sizeof(decimal_port), "%d", m_port);
    decimal_port[sizeof(decimal_port)/sizeof(decimal_port[0])-1] = '\0';
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;

    int r(getaddrinfo(addr.c_str(), decimal_port, &hints, &m_addrinfo));
    if(r != 0 || m_addrinfo == NULL){
        throw std::runtime_error(("Invalid address or port: \"" + addr + ":" + decimal_port + "\"").c_str());
    }

    m_socket = socket(m_addrinfo->ai_family, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
    if(m_socket == INVALID_SOCKET){
        freeaddrinfo(m_addrinfo);
        throw std::runtime_error(("Could not create socket for  \"" + addr + ":" + decimal_port + "\"").c_str());
    }

    m_server_knows_me = false;
}


UDPClient::~UDPClient(){
    freeaddrinfo(m_addrinfo);
    close(m_socket);
}

int UDPClient::get_socket() const{
    return m_socket;
}

int UDPClient::get_port() const{
    return m_port;
}

std::string UDPClient::get_addr() const{
    return m_addr;
}

int UDPClient::send(const char *msg, size_t size){
    int n = sendto(m_socket, msg, size, 0, m_addrinfo->ai_addr, m_addrinfo->ai_addrlen);
    if(n < 0){
        throw std::runtime_error("Could not send from UDP socket.");
    }
    m_server_knows_me = true;
    return n;
}

int UDPClient::recv(char *msg, size_t max_size){
    if(!m_server_knows_me)
        throw std::runtime_error("UDP Client tried to receive before sending.");

    int n = recvfrom(m_socket, msg, max_size-1,0, m_addrinfo->ai_addr, &m_addrinfo->ai_addrlen);
    if(n < 0){
        throw std::runtime_error(("Could not receive from UDP socket."));
    }
    msg[n] = '\0';
    return n;
}

UDPServer::UDPServer(const std::string& addr, int port): m_port(port), m_addr(addr){
    char decimal_port[16];
    snprintf(decimal_port, sizeof(decimal_port), "%d", m_port);
    decimal_port[sizeof(decimal_port)/sizeof(decimal_port[0])-1] = '\0';

    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;

    int r(getaddrinfo(addr.c_str(), decimal_port, &hints, &m_addrinfo));
    if(r != 0 || m_addrinfo == NULL){
        throw std::runtime_error(("Invalid address or port: \"" + addr + ":" + decimal_port + "\"").c_str());
    }

    m_socket = socket(m_addrinfo->ai_family, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
    if(m_socket == INVALID_SOCKET){
        freeaddrinfo(m_addrinfo);
        throw std::runtime_error(("Could not create socket for  \"" + addr + ":" + decimal_port + "\"").c_str());
    }

    r = bind(m_socket, m_addrinfo->ai_addr, m_addrinfo->ai_addrlen);
    if(r == SOCKET_ERROR){
        freeaddrinfo(m_addrinfo);
        close(m_socket);
        throw std::runtime_error(("Could not bind socket with  \"" + addr + ":" + decimal_port + "\"").c_str());
    }

    m_client_is_known = false;
}


UDPServer::~UDPServer(){
    freeaddrinfo(m_addrinfo);
    close(m_socket);
}

int UDPServer::get_socket() const{
    return m_socket;
}

int UDPServer::get_port() const{
    return m_port;
}

std::string UDPServer::get_addr() const{
    return m_addr;
}

int UDPServer::recv(char* msg, size_t max_size){
    SOCKADDR_IN _m_client = { 0 };
    m_client = _m_client;
    m_fromsize = sizeof m_client;
    int n=recvfrom(m_socket, msg, max_size-1, 0, (SOCKADDR *) &m_client,(socklen_t*) &m_fromsize);
    if(n < 0){
        throw std::runtime_error(("Could not receive from UDP Server socket."));
    }
    msg[n] = '\0';

    m_client_is_known = true;
    return n;
}


int UDPServer::send(const char* msg, size_t size){
    if(!m_client_is_known)
        throw std::runtime_error("UDP Server tried to send before receiveing.");

    int n = sendto(m_socket, msg, size, 0, (SOCKADDR *) &m_client, m_fromsize);
    if(n < 0){
        throw std::runtime_error("Could not send from UDP Server socket.");
    }
    return n;
}

int UDPServer::send(const char* msg, size_t size, const std::string& addr, int port){

    char decimal_port[16];
    snprintf(decimal_port, sizeof(decimal_port), "%d", port);
    decimal_port[sizeof(decimal_port)/sizeof(decimal_port[0])-1] = '\0';

    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;

    struct addrinfo*  client_addrinfo;

    int r(getaddrinfo(addr.c_str(), decimal_port, &hints, &client_addrinfo));
    if(r != 0 || client_addrinfo == NULL){
        throw std::runtime_error(("Invalid destination address or port: \"" + addr + ":" + decimal_port + "\"").c_str());
    }

    int n = sendto(m_socket, msg, size, 0, client_addrinfo->ai_addr, client_addrinfo->ai_addrlen);
    if(n < 0){
        throw std::runtime_error("Could not send from UDP Server socket.");
    }

    freeaddrinfo(client_addrinfo);
    return n;
}
