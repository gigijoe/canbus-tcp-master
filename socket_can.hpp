#ifndef SOCKET_CAN_HPP_
#define SOCKET_CAN_HPP_

#include <climits>
#include <stdint.h>
#include <unistd.h> 

#include <linux/can.h>

#include <mutex>

#include "can_bus.hpp"

using namespace std;

/*
 * Controller Area Network Identifier structure
 *
 * bit 0-28	: CAN identifier (11/29 bit)
 * bit 29	: error frame flag (0 = data frame, 1 = error frame)
 * bit 30	: remote transmission request flag (1 = rtr frame)
 * bit 31	: frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
 */
typedef uint32_t canid_t;

/*
 * Controller Area Network Error Frame Mask structure
 *
 * bit 0-28	: error class mask (see include/socketcan/can/error.h)
 * bit 29-31	: set to zero
 */
typedef uint32_t can_err_mask_t;

class SocketCan : public CanBus {
protected:
	char *m_devName;
	int m_socketFd;

public:
	SocketCan() : CanBus(), m_devName(nullptr), m_socketFd(-1) {}
	~SocketCan() {
		if(m_socketFd != -1)
			close(m_socketFd);
	}

	int Connect(uint16_t port, uint32_t bitrate);

	int Write(can_frame & f);
	int Read(vector<can_frame> & vf, uint32_t timeout = 10);

	int Write(uint32_t id, uint8_t dlc, uint8_t *data);

	int Print(can_frame & f);
};

#endif
