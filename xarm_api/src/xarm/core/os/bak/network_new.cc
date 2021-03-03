/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
 // #define _WINSOCK_DEPRECATED_NO_WARNINGS

#ifdef _WIN32
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <net/if.h>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "xarm/core/os/network.h"

#define DB_FLG "[NetWork] "
#define PRINT_ERR printf

#define PERRNO(ret, db_flg, str)        			  	 \
{                                     				 	 \
    if (-1 == ret) {                    				 \
      PRINT_ERR("%s%s, errno=%d\n", db_flg, str, errno); \
      return -1;                        				 \
    }                                   				 \
  														 \
}

int socket_init(char *local_ip, int port, int is_server) {
	int sockfd = static_cast<int> (socket(AF_INET, SOCK_STREAM, 0));
	PERRNO(sockfd, DB_FLG, "Error: socket");

	int on = 1;
	int keepAlive = 1;     // Turn on keepalive attribute
	int keepIdle = 60;      // If there is no data in n seconds, probe
	int keepInterval = 10;  // Detection interval,, unit:seconds
	int keepCount = 3;     // 3 detection attempts
	struct timeval timeout = { 2, 0 };

	int ret =
		setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (char *)&on, sizeof(on));
	PERRNO(ret, DB_FLG, "Error: setsockopt[SO_REUSEADDR]");
	ret = setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, (char *)&keepAlive,
		sizeof(keepAlive));
	PERRNO(ret, DB_FLG, "Error: setsockopt[SO_KEEPALIVE]");
	ret = setsockopt(sockfd, IPPROTO_TCP, TCP_KEEPIDLE, (char *)&keepIdle,
		sizeof(keepIdle));
	PERRNO(ret, DB_FLG, "Error: setsockopt[TCP_KEEPIDLE]");
	ret = setsockopt(sockfd, IPPROTO_TCP, TCP_KEEPINTVL, (char *)&keepInterval,
		sizeof(keepInterval));
	PERRNO(ret, DB_FLG, "Error: setsockopt[TCP_KEEPINTVL]");
	ret = setsockopt(sockfd, IPPROTO_TCP, TCP_KEEPCNT, (char *)&keepCount,
		sizeof(keepCount));
	PERRNO(ret, DB_FLG, "Error: setsockopt[TCP_KEEPCNT]");
	ret = setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout,
		sizeof(struct timeval));
	PERRNO(ret, DB_FLG, "Error: setsockopt[SO_SNDTIMEO]");

	if (is_server) {
		struct sockaddr_in local_addr;
		local_addr.sin_family = AF_INET;
		local_addr.sin_port = htons(port);
		local_addr.sin_addr.s_addr = inet_addr(local_ip);
		ret = bind(sockfd, (struct sockaddr *)&local_addr, sizeof(local_addr));
		PERRNO(ret, DB_FLG, "error: bind");

		int ret = listen(sockfd, 10);
		PERRNO(ret, DB_FLG, "error: listen");
	}
	return sockfd;
}

int socket_connect_server(int *socket, char server_ip[], int server_port) {
	struct sockaddr_in server_addr;
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(server_port);
	inet_aton(server_ip, &server_addr.sin_addr);
	int ret =
		connect(*socket, (struct sockaddr *)&server_addr, sizeof(server_addr));
	PERRNO(ret, DB_FLG, "error: connect");
	return 0;
}

int socket_send_data(int client_fp, unsigned char *data, int len) {
	int ret = send(client_fp, (void *)data, len, 0);
	PERRNO(ret, DB_FLG, "error: socket_send_data");
	return ret;
}
