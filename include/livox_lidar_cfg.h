
#ifndef LIVOX_LIDAR_CONFIG_H_
#define LIVOX_LIDAR_CONFIG_H_

#if defined(__linux__)
    #include <sys/epoll.h>
    #include <sys/time.h>
    #include <unistd.h>
    #define HAVE_EPOLL 1
#elif defined(_WIN32)
    #include <winsock2.h>
    #define HAVE_SELECT 1
#elif defined(__APPLE__) || defined(__FreeBSD__) || defined(__OpenBSD__) || defined (__NetBSD__)
    #include <sys/event.h>
    #include <sys/types.h>
    #include <fcntl.h>
    #include <unistd.h>
    #define HAVE_KQUEUE 1
#else
    #include <sys/poll.h>
    #define HAVE_POLL 1
#endif

#endif  // CONFIG_H_


