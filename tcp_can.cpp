#include "tcp_can.hpp"
#include "can_node.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h> 

static uint32_t endian_swap(uint32_t num) {
	return ((num>>24)&0xff) | // move byte 3 to byte 0
					((num<<8)&0xff0000) | // move byte 1 to byte 2
					((num>>8)&0xff00) | // move byte 2 to byte 1
					((num<<24)&0xff000000); // byte 0 to byte 3
}

/*
*
*/
#undef DEBUG_RESOLVE_HOST
#undef DEBUG_TCPCAN

#include <netinet/in.h>
#include <netinet/tcp.h>

#define check(expr) if (!(expr)) { perror(#expr); }

static void enable_keepalive(int sock) {
	// Enable / Disable keep alive
    int yes = 1; 
    check(setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(int)) != -1);

    // 这个参数是多久没有发送数据时，开始发送Keep-Alive包的时间，也就是链路空闲时间。
    int idle = 10; // 單位 秒
    check(setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(int)) != -1);

    // 这个参数是指发送Keep-Alive probe后，对方多久没有回应，然后重新再发送keep alive probe的时间间隔
    int interval = 3; // 單位 秒
    check(setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(int)) != -1);

    // 这个参数指，连续发送多少次keep alive probe，对方没有回应，认为连接已经失效的重试次数
    int maxpkt = 10;
    check(setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &maxpkt, sizeof(int)) != -1);
}

int TcpCan::Connect(const char *devAddr, uint16_t port)
{
	if(devAddr == 0 || strlen(devAddr) == 0)
		return -1;

	if(m_devAddr != devAddr) {
		if(m_devAddr)
			free(m_devAddr);
		m_devAddr = strdup(devAddr);
	}
	if(m_devAddr == 0 || strlen(m_devAddr) <= 0)
		return -1;

	m_port = port;

	string ip4_str;
	struct addrinfo hints;
	struct addrinfo *res;
	char hostaddr[256];
#ifdef DEBUG_RESOLVE_HOST
	printf("Resolve host %s ...\n", devAddr);
#endif
	memset(&hints, 0, sizeof(struct addrinfo));
	hints.ai_family = AF_INET;

	int ret = getaddrinfo(devAddr, NULL, &hints, &res);
	if(ret != 0) {
		printf("getaddrinfo: (%s) %s\n", devAddr, gai_strerror(ret));
	} else {
#ifdef DEBUG_RESOLVE_HOST
		printf("Looking for %s\n", devAddr);
#endif
		struct addrinfo *tmp;
		for(tmp = res; tmp != NULL; tmp = tmp->ai_next) {
#ifdef DEBUG_RESOLVE_HOST
			printf("flags: 0x%x\tfamily: %d\tsocktype: %d\tprotocol: %d\n",
				tmp->ai_flags,
				tmp->ai_family,
				tmp->ai_socktype,
				tmp->ai_protocol);
#endif			
			getnameinfo(tmp->ai_addr, tmp->ai_addrlen, hostaddr, sizeof(hostaddr), NULL, 0, NI_NUMERICHOST);
			if(tmp->ai_protocol == 6) { // TCP
				//debug_printf("%s\n", hostaddr);
				ip4_str = hostaddr;
				break;
			}
		}
	}

	if(ip4_str.empty()) {
		printf("No IP resolved !!!\n");
		return -1;
	} else {
#ifdef DEBUG_RESOLVE_HOST
		printf("IP resolved %s\n", ip4_str.c_str());
#endif
	}

	int sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) {
		printf("ERROR opening socket !!!\n");
		return -1;
	}
#if 1
	int flag = 1;
	if(setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &flag,
				sizeof(flag)) < 0)
		printf("setsockopt TCP_NODELAY failed\n");
#endif
#if 1
	struct timeval timeout;      
	timeout.tv_sec = 2; // 2s
	timeout.tv_usec = 0; 

	if(setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &timeout,
				sizeof(timeout)) < 0)
		printf("setsockopt SO_SNDTIMEO failed\n");
#endif

	enable_keepalive(sockfd);

	m_socketFd = sockfd;

	printf("%s:%u Connecting ...\n", ip4_str.c_str(), port);

	/* build the server's Internet address */
	struct sockaddr_in serveraddr;
	bzero((char *) &serveraddr, sizeof(serveraddr));
	serveraddr.sin_family = PF_INET;
	serveraddr.sin_addr.s_addr = inet_addr(ip4_str.c_str());
	serveraddr.sin_port = htons(port);

	/* connect: create a connection with the server */
	if(connect(m_socketFd, (struct sockaddr *)&serveraddr, sizeof(struct sockaddr)) < 0) {
		printf("ERROR connecting %s(%s):%u !!!\n", devAddr, ip4_str.c_str(), port);
		close(m_socketFd);
		m_socketFd = -1;
		return -1;
	}

	m_isConnected = true;

	printf("%s:%u Connected\n", ip4_str.c_str(), port);

	return m_socketFd;
}

int TcpCan::Disconnect()
{
	if(m_socketFd >= 0)
		close(m_socketFd);

	m_socketFd = -1;
	m_isConnected = false;

	return 0;
}

int TcpCan::Reconnect()
{
	Disconnect();
	return Connect(m_devAddr, m_port);
}

int TcpCan::Write(TcpCanFrame & f)
{
	if(m_socketFd < 0 || m_isConnected == false)
		return -1;

	f.id = endian_swap(f.id);

	if(m_isWriteCache) {
		std::unique_lock<std::mutex> lock(m_socketMutex);

		m_writeCache.push_back(f);
#ifdef DEBUG_TCPCAN
	printf("WC : ");
	printh((const uint8_t *)&f, sizeof(f));
#endif
		if(m_writeCache.size() >= 50) // Flush cache to avoid packet segment
		//if(m_writeCache.size() >= 80) // Maximum CAN frame per TCP packet of ZQWL CANET
		{
			lock.unlock();
			FlushWriteCache();
		}

		return m_writeCache.size();
	}

	/* send the message line to the server */
	int n = write(m_socketFd, (uint8_t *)&f, sizeof(f));
	if(n < 0) {
		printf("%s:%u : Socket write : %s (%d)\n", m_devAddr, m_port, strerror(errno), errno);
		Disconnect();
		return -2;
	}
#ifdef DEBUG_TCPCAN
	printf("W : ");
	printh((const uint8_t *)&f, sizeof(f));
#endif
	return 0;
}

void TcpCan::EnableWriteCache()
{
	m_isWriteCache = true;
}

void TcpCan::DisableWriteCache()
{
	m_isWriteCache = false;
}

void TcpCan::FlushWriteCache()
{
	if(m_writeCache.size() == 0)
		return;

	std::unique_lock<std::mutex> lock(m_socketMutex);

	/* send the message line to the server */
	int n = write(m_socketFd, (uint8_t *)m_writeCache.data(), sizeof(TcpCanFrame) * m_writeCache.size());
	if(n < 0) {
		printf("%s:%u : Socket write : %s (%d)\n", m_devAddr, m_port, strerror(errno), errno);
		Disconnect();
		return;
	}
#ifdef DEBUG_TCPCAN
	printf("FC : %lu\n", m_writeCache.size());
#endif
	
	m_writeCache.clear();

	return;
}

#define MAX_FRAME_COUNT 20

int TcpCan::Read(vector<TcpCanFrame> & vf, uint32_t timeout)
{
	if(m_socketFd < 0 || m_isConnected == false)
		return -1;

	fd_set fdsr;
	struct timeval tv;

	FD_ZERO(&fdsr);
	FD_SET(m_socketFd, &fdsr);

	tv.tv_sec = 0;
	tv.tv_usec = timeout * 1000; /* Default 10 ms */

	int r = select(m_socketFd+1, &fdsr, NULL, NULL, &tv);
	if(r > 0) {
		char buf[sizeof(TcpCanFrame) * MAX_FRAME_COUNT + 1];
		//int len = TEMP_FAILURE_RETRY(recv(m_socketFd, (char *)buf, sizeof(TcpCanFrame) * MAX_FRAME_COUNT, 0));
		int len = recv(m_socketFd, (char *)buf, sizeof(TcpCanFrame) * MAX_FRAME_COUNT, 0);
		if(len < 0) {
 			printf("%s:%u : Socket recv : %s (%d)\n", m_devAddr, m_port, strerror(errno), errno);
			Disconnect();
			return -2;
		} else if(len == 0) {
			printf("%s:%u : Connection closed\n", m_devAddr, m_port);
			Disconnect();
			return -3;
		} else {
			int r = 0;
			while((r + sizeof(TcpCanFrame)) <= len) {
				TcpCanFrame *pf = (TcpCanFrame *)&buf[r];
				pf->id = endian_swap(pf->id);
				vf.push_back(*pf);
				r += sizeof(TcpCanFrame);
#ifdef DEBUG_TCPCAN
				printf("R : ");
				printh((const uint8_t *)pf, sizeof(TcpCanFrame));
#endif
			}

		}
		return len;
	}

	return 0;
}

int TcpCan::Write(uint32_t id, uint8_t dlc, uint8_t *data)
{
	if(m_socketFd < 0 || m_isConnected == false)
		return -1;

	TcpCanFrame f = {0};
	f.id = id;
	f.LEN = dlc;
	memcpy(&f.data[0], data, dlc);

	return Write(f);
}
