
#ifndef LIVOX_NETWORK_UTIL_H_
#define LIVOX_NETWORK_UTIL_H_

#include <stdint.h>
#ifdef WIN32
#include <winsock2.h>
#include <Ws2tcpip.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif // WIN32
#include <stdio.h>
#include <stdio.h>
#include <fcntl.h>
#include <string>

namespace livox {
namespace lidar {

namespace util {

typedef int socket_t;
socket_t CreateSocket(uint16_t port, bool nonblock = true, bool reuse_port = true, bool is_broadcast = false, const std::string netif = "", const std::string multicast_ip = "");
//socket_t CreateSocket(uint16_t port, bool nonblock = true, bool reuse_port = true, bool is_broadcast = false);

void CloseSock(socket_t sock);

bool FindLocalIp(const struct sockaddr_in &client_addr, uint32_t &local_ip);

size_t RecvFrom(socket_t &sock, void *buff,  size_t buf_size, int flag, struct sockaddr *addr, int* addrlen);

}  // namespace util
} // namespace lidar
}  // namespace livox

#endif  // LIVOX_NETWORK_UTIL_H_
