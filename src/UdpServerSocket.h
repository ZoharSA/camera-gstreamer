#ifndef __OS_UDP_SRVER_SOCKET_H__
#define __OS_UDP_SRVER_SOCKET_H__

#include <iostream>
#include <unistd.h>
#include <stdexcept>
#include <netinet/tcp.h>
#include <UdpServerSocket.h>
#include <netinet/in.h>
#include <cstring>

namespace OS
{

class UdpServerSocket
{
public:
    struct PointerLength
    {
        void * buffer;
        unsigned length;

        static PointerLength fromStdString(const std::string & source) {
            return {const_cast<char *>(source.c_str()), static_cast<unsigned>(source.size())}; }
        std::string toString() const { return std::string(reinterpret_cast<char *>(buffer), length); }
    };
    UdpServerSocket(UdpServerSocket && other): _fd(other._fd) { other._fd = -1; }
    void operator=(UdpServerSocket && other) { close(); _fd = other._fd; other._fd = -1; }

    UdpServerSocket():
        _fd(::socket(AF_INET, SOCK_DGRAM, 0))
    {
        if (_fd < 0)
            std::cerr << "Unable to open UDP socket" << "\n";
        std::cout << "Opening socket fd: " << _fd << std::endl;
    }

    ~UdpServerSocket()
    {
        close();
    }

    void close()
    {
        if (_fd < 0)
            return;
        std::cout << "Closing socket fd: " << _fd << std::endl;
        ::close(_fd);
    }

    int fileDescriptor() { return _fd; }

    static const constexpr char* ANY_ENDPOINT_IP = "";

    static constexpr int RECEIVE_NO_FLAGS = 0;

    unsigned receive(PointerLength buffer, int flags = RECEIVE_NO_FLAGS)
    {
        ssize_t result = ::recv(_fd, buffer.buffer, buffer.length, flags);
        if (result < 0) {
            if (errno == ECONNRESET) {
                throw std::runtime_error("Socket: connection reset by peer on recv");
            } else {
                std::cerr << "Unable to receive on socket" << "\n";
                throw std::runtime_error("Unable to receive on socket");
            }
        }
        return static_cast<unsigned>(result);
    }

    void bind(unsigned short port)
    {
        struct sockaddr_in address;
        memset(&address, 0, sizeof(address)); 

        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY; 
        address.sin_port = htons(port);

        std::cout << "Binding socket (" << _fd << ") to port " << port << std::endl;
		int result = ::bind(_fd, (const struct sockaddr *)&address, sizeof(address));
        if (result < 0) {
            std::cerr << "Unable to bind to port " << port << "\n";
            throw std::runtime_error("Unable to bind to address");
        }
	}

private:

    int _fd;

    UdpServerSocket(const UdpServerSocket &) = delete;
    void operator=(const UdpServerSocket &) = delete;
};

}

#endif // __OS_UDP_SRVER_SOCKET_H__
