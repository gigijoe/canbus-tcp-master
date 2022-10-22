#include "socket_can.hpp"
#include "can_node.hpp"

#include <climits>
#include <stdint.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h> 
#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>

#include "canframelen.h"
#include "socket_can.hpp"

#include <stdarg.h>

#undef DEBUG_SOCKETCAN

#define STR_SIZE 256

static int ssystem(const char *fmt, ...)
{
	int r = 0;
	char str[STR_SIZE];
	va_list args;

	va_start(args, fmt);
	vsnprintf(str, STR_SIZE, fmt, args);
	r = system(str);
	va_end(args);

	return r;
}

int SocketCan::Connect(uint16_t port, uint32_t bitrate)
{
	ssystem("sudo ifconfig can%d down", port);
	ssystem("sudo ip link set can%d up type can bitrat %d", port, bitrate);
	//ssystem("sudo ifconfig can%d txqueuelen 100", port);
	ssystem("sudo ifconfig can%d txqueuelen 65536", port);

	struct sockaddr_can addr; 
	struct ifreq ifr; 

	int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字 
	sprintf(ifr.ifr_name, "can%u", port);
	ioctl(s, SIOCGIFINDEX, &ifr); //指定 can[port] 设备 

	can_err_mask_t optval = (CAN_ERR_TX_TIMEOUT | 
		CAN_ERR_LOSTARB |
		CAN_ERR_CRTL | 
		CAN_ERR_PROT | 
		CAN_ERR_TRX | 
		CAN_ERR_ACK | 
		CAN_ERR_BUSOFF | 
		CAN_ERR_BUSERROR |  
		CAN_ERR_RESTARTED);
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &optval, sizeof(optval));

	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	bind(s, (struct sockaddr *)&addr, sizeof(addr));//将套接字与 can0 绑定 

	//禁用过滤规则
	//setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
#if 0
	int loopback = 0;
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
#endif
#if 1
	struct timeval timeout;      
	timeout.tv_sec = 0; 
	timeout.tv_usec = 300; // 300 us 

	if(setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &timeout,
				sizeof(timeout)) < 0)
		printf("setsockopt SO_SNDTIMEO failed\n");
#endif

#if 0
	fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);
#endif
	m_socketFd = s;

	return 0;
}

int SocketCan::Write(can_frame & f)
{
	if(m_socketFd == -1)
		return 0;

#ifdef DEBUG_SOCKETCAN
	printf("W : ");
	//printh((const uint8_t *)&f, sizeof(f.can_id) + sizeof(f.can_dlc) + 8);
	Print(f);
#endif
	int nbytes = write(m_socketFd, &f, sizeof(f));
	if(nbytes != sizeof(f))
		printf("Error write !!!\n");

	return nbytes;
}

int SocketCan::Read(vector<can_frame> & vf, uint32_t timeout)
{
	if(m_socketFd == -1)
		return -1;

	struct can_frame f; 
	
	fd_set fdsr;
	struct timeval tv;

	FD_ZERO(&fdsr);
	FD_SET(m_socketFd, &fdsr);

	tv.tv_sec = 0;
	tv.tv_usec = timeout * 1000; /* Default 10 ms */

	int r = select(m_socketFd+1, &fdsr, NULL, NULL, &tv);
	if(r > 0) {
		int	nbytes = read(m_socketFd, &f, sizeof(f));
		if(nbytes != sizeof(f))
			printf("Error read !!!\n");
		else {
#ifdef DEBUG_SOCKETCAN
			printf("R : ");
			//printh((const uint8_t *)&f, sizeof(f.can_id) + sizeof(f.can_dlc) + 8);
			Print(f);
#endif
			if(f.can_id & CAN_ERR_FLAG ) {
				if(f.can_id & CAN_ERR_TX_TIMEOUT)
					printf("TX timeout (by netdevice driver)\n");
				if(f.can_id & CAN_ERR_LOSTARB   )
					printf("lost arbitration: 0x%02hhx\n",f.data[0]);
				if(f.can_id & CAN_ERR_CRTL      )
					printf("controller problems: 0x%02hhx\n",f.data[1]);
				if(f.can_id & CAN_ERR_PROT      )
					printf("protocol violations: 0x%02hhx 0x%02hhx\n",f.data[2],f.data[3]);
				if(f.can_id & CAN_ERR_TRX       )
					printf("transceiver status: 0x%02hhx\n",f.data[4]);
				if(f.can_id & CAN_ERR_ACK       )
					printf("received no ACK on transmission\n");
				if(f.can_id & CAN_ERR_BUSOFF    )
					printf("bus off\n");
				if(f.can_id & CAN_ERR_BUSERROR  )
					printf("bus error (may flood!)\n");
				if(f.can_id & CAN_ERR_RESTARTED )
					printf("controller restarted\n");
			} else {
				vf.push_back(f);
			}

			static enum cfl_mode mode = CFL_WORSTCASE;

			m_stat.recv_frames++;
			m_stat.recv_bits_payload += f.can_dlc * 8;
			m_stat.recv_bits_dbitrate += can_frame_dbitrate_length(
					&f, mode, sizeof(f));
			m_stat.recv_bits_total += can_frame_length(&f,
								    mode, nbytes);

		}
		return nbytes;
	}
	
	return 0;
}

int SocketCan::Write(uint32_t id, uint8_t dlc, uint8_t *data)
{
	if(m_socketFd == -1)
		return 0;

	struct can_frame f = {0}; 
	f.can_id = id;
	f.can_dlc = dlc;
	memcpy(&f.data[0], data, dlc);
#ifdef DEBUG_SOCKETCAN
	printf("W : ");
	//printh((const uint8_t *)&f, sizeof(f.can_id) + sizeof(f.can_dlc) + 8);
	Print(f);
#endif
	int nbytes = write(m_socketFd, &f, sizeof(f));

	return nbytes;
}

int SocketCan::Print(can_frame & f)
{
	printf("[0x%03x] (%d) ", f.can_id, f.can_dlc);
	printh((const uint8_t *)&f.data[0], 8);

	return 0;
}

int SocketCan::BusLoad()
{
	int percent;

	if(m_bitrate > 0)
		percent = ((m_stat.recv_bits_total-m_stat.recv_bits_dbitrate) * 100) / m_bitrate
			+ (m_stat.recv_bits_dbitrate * 100) / m_stat.dbitrate;
	else
		percent = 0;

	m_stat.recv_frames = 0;
	m_stat.recv_bits_total = 0;
	m_stat.recv_bits_dbitrate = 0;
	m_stat.recv_bits_payload = 0;

	return percent;
}
