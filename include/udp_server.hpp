
#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <unistd.h>

class UdpServer
{
    int _fd;
    sockaddr_in _servaddr;
    sockaddr_in _dest_addr;

  public:
    UdpServer(uint16_t port)
    {
        _fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (_fd < 0)
        {
            perror("cannot open server socket");
            return;
        }

        bzero(&_servaddr, sizeof(_servaddr));
        _servaddr.sin_family = AF_INET;
        _servaddr.sin_addr.s_addr = INADDR_ANY;
        _servaddr.sin_port = htons(port);

        struct timeval read_timeout;
        read_timeout.tv_sec = 0;
        read_timeout.tv_usec = 10;
        setsockopt(_fd, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout));

        if (bind(_fd, (const struct sockaddr *)&_servaddr, sizeof(_servaddr)) < 0)
        {
            perror("bind failed");
            return;
        }

        bzero(&_dest_addr, sizeof(_dest_addr));
        _dest_addr.sin_family = AF_INET;
        inet_aton("192.168.4.153", &_dest_addr.sin_addr);
        _dest_addr.sin_port = htons(8080);
    }

    bool send(const char *data, size_t len)
    {
        int result = sendto(_fd, data, len, 0, (sockaddr *)&_dest_addr, sizeof(_dest_addr));
        if (static_cast<size_t>(result) != len)
        {
            return false;
        }
        return true;
    }

    int receive(char *buff, size_t buff_size)
    {
        sockaddr_in addr;
        socklen_t slen = sizeof(sockaddr_in); // for correct addr reading
        return recvfrom(_fd, buff, buff_size, MSG_WAITALL, (struct sockaddr *)&addr, &slen);
    }
};