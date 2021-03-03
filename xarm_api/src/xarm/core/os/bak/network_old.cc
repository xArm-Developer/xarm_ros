/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
 // #define _WINSOCK_DEPRECATED_NO_WARNINGS

#include <stdio.h>
#include <string.h>
#include <iostream>

#ifdef _WIN32
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib")
#include <cstring>
#include<ws2tcpip.h>
#include<MSTCPiP.h>
#define errno WSAGetLastError()
#else
#include <errno.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#endif

#ifdef _WIN32
#ifndef inet_pton
extern "C" {
	WINSOCK_API_LINKAGE  INT WSAAPI inet_pton(INT Family, PCSTR pszAddrString, PVOID pAddrBuf);
}
#endif
#endif

#include "xarm/core/os/network.h"

#define DB_FLG "[net work] "
#define PRINT_ERR printf

#define PERRNO(ret, db_flg, str)        \
  {                                     \
    if (-1 == ret) {                    \
      PRINT_ERR("%s%s, errno=%d\n", db_flg, str, errno); \
      return -1;                        \
    }                                   \
  \
}

#ifdef _WIN32

int socket_init(char *local_ip, int port, int is_server) {
	// int iResult;
	// WSADATA wsaData;
	// iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
	// if (iResult != 0) {
	// 	printf("WSAStartup failed: %d\n", iResult);
	// 	return -1;
	// }
	// struct addrinfo *result = NULL, *ptr = NULL, hints;
	// ZeroMemory(&hints, sizeof(hints));
	// hints.ai_family = AF_INET; // AF_UNSPEC;
	// hints.ai_socktype = SOCK_STREAM;
	// hints.ai_protocol = IPPROTO_TCP;
	// if (is_server) hints.ai_flags = AI_PASSIVE;
	// iResult = getaddrinfo(local_ip, port, &hints, &result);
	// if (iResult != 0) {
	// 	printf("getaddrinfo failed: %d\n", iResult);
	// 	WSACleanup();
	// 	return -1;
	// }
	// SOCKET sockfd = INVALID_SOCKET;
	// // Attempt to connect to the first address returned by
	// // the call to getaddrinfo
	// ptr = result;

	// // Create a SOCKET for connecting to server
	// sockfd = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
	// if (sockfd == INVALID_SOCKET) {
	// 	printf("Error at socket(): %ld\n", WSAGetLastError());
	// 	freeaddrinfo(result);
	// 	WSACleanup();
	// 	return -1;
	// }
	// if (is_server) {
	// 	iResult = bind(sockfd, result->ai_addr, (int)result->ai_addrlen);
	// 	if (iResult == SOCKET_ERROR) {
	// 		printf("bind failed with error: %d\n", WSAGetLastError());
	// 		freeaddrinfo(result);
	// 		closesocket(sockfd);
	// 		WSACleanup();
	// 		return -1;
	// 	}
	// }

	WORD sockVersion = MAKEWORD(2, 2);
	WSADATA data;
	PERRNO(WSAStartup(sockVersion, &data), DB_FLG, "ESAStartup");
	int sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	PERRNO(sockfd, DB_FLG, "error: socket");

	int on = 1;
	int keepAlive = 1;     // Turn on keepalive attribute
	int keepIdle = 1;      // If there is no data in n seconds, probe
	int keepInterval = 1;  // Detection interval,5 seconds
	int keepCount = 3;     // 3 detection attempts
	struct timeval timeout = { 2, 0 };

	int ret =
		setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (char *)&on, sizeof(on));
	PERRNO(ret, DB_FLG, "error: setsockopt");
	ret = setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, (char *)&keepAlive,
		sizeof(keepAlive));
	PERRNO(ret, DB_FLG, "error: setsockopt");

	tcp_keepalive alive_in;
	tcp_keepalive alive_out;
	alive_in.keepalivetime = 1000;  // 1s TCP_KEEPIDLE
	alive_in.keepaliveinterval = 1000; //1s TCP_KEEPINTVL
	alive_in.onoff = 1;
	unsigned long ulBytesReturn = 0;
	ret = WSAIoctl(sockfd, SIO_KEEPALIVE_VALS, &alive_in, sizeof(alive_in),
		&alive_out, sizeof(alive_out), &ulBytesReturn, NULL, NULL);
	if (ret == SOCKET_ERROR)
	{
		PERRNO(ret, DB_FLG, "WSAIoctl failed");
		// TRACE(L"setsockopt failed: %d\n", WSAGetLastError());
	}
	/*
	ret = setsockopt(sockfd, IPPROTO_TCP, TCP_KEEPIDLE, (char *)&keepIdle,
		sizeof(keepIdle));
	PERRNO(ret, DB_FLG, "error: setsockopt");
	ret = setsockopt(sockfd, IPPROTO_TCP, TCP_KEEPINTVL, (char *)&keepInterval,
		sizeof(keepInterval));
	PERRNO(ret, DB_FLG, "error: setsockopt");
	ret = setsockopt(sockfd, IPPROTO_TCP, TCP_KEEPCNT, (char *)&keepCount,
		sizeof(keepCount));
	PERRNO(ret, DB_FLG, "error: setsockopt");
	*/
	ret = setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout,
		sizeof(struct timeval));
	PERRNO(ret, DB_FLG, "error: setsockopt");

	if (is_server) {
		struct sockaddr_in local_addr;
		local_addr.sin_family = AF_INET;
		local_addr.sin_port = htons(port);
		// local_addr.sin_addr.s_addr = inet_addr(local_ip);
		inet_pton(AF_INET, local_ip, (void *)&local_addr.sin_addr.S_un.S_addr);
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
	//inet_aton(server_ip, &server_addr.sin_addr);
	inet_pton(AF_INET, server_ip, &server_addr.sin_addr);
	//InetPton(AF_INET, server_ip, &server_addr.sin_addr);
	int ret =
		connect(*socket, (struct sockaddr *)&server_addr, sizeof(server_addr));
	PERRNO(ret, DB_FLG, "error: connect");
	return 0;
}

int socket_send_data(int client_fp, unsigned char *data, int len) {
	int ret = send(client_fp, (char *)data, len, 0);
	PERRNO(ret, DB_FLG, "error: socket_send_data");
	// if (ret == -1) { PRINT_ERR(DB_FLG "error: socket_send_data\n"); }
	return ret;
}

#else

int socket_init(char *local_ip, int port, int is_server) {
	int sockfd = socket(AF_INET, SOCK_STREAM, 0);
	PERRNO(sockfd, DB_FLG, "error: socket");

	int on = 1;
	int keepAlive = 1;     // Turn on keepalive attribute
	int keepIdle = 60;      // If there is no data in n seconds, probe
	int keepInterval = 10;  // Detection interval,5 seconds
	int keepCount = 3;     // 3 detection attempts
	struct timeval timeout = { 2, 0 };

	int ret =
		setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (void *)&on, sizeof(on));
	PERRNO(ret, DB_FLG, "error: setsockopt");
	ret = setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, (void *)&keepAlive,
		sizeof(keepAlive));
	PERRNO(ret, DB_FLG, "error: setsockopt");
	ret = setsockopt(sockfd, SOL_TCP, TCP_KEEPIDLE, (void *)&keepIdle,
		sizeof(keepIdle));
	PERRNO(ret, DB_FLG, "error: setsockopt");
	ret = setsockopt(sockfd, SOL_TCP, TCP_KEEPINTVL, (void *)&keepInterval,
		sizeof(keepInterval));
	PERRNO(ret, DB_FLG, "error: setsockopt");
	ret = setsockopt(sockfd, SOL_TCP, TCP_KEEPCNT, (void *)&keepCount,
		sizeof(keepCount));
	PERRNO(ret, DB_FLG, "error: setsockopt");
	ret = setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout,
		sizeof(struct timeval));
	PERRNO(ret, DB_FLG, "error: setsockopt");

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
	// if (ret == -1) { PRINT_ERR(DB_FLG "error: socket_send_data\n"); }
	return ret;
}

#endif
