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
*
* https://blog.csdn.net/aiyanzielf/article/details/111708394
*
* https://www.waveshare.com/2-ch-can-hat.htm
* https://www.waveshare.com/wiki/2-CH_CAN_HAT
*
*/

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
	char *m_busName;
	uint32_t m_bitrate;
	int m_socketFd;

	struct {
		uint32_t dbitrate;
		uint32_t recv_frames;
		uint32_t recv_bits_total;
		uint32_t recv_bits_payload;
		uint32_t recv_bits_dbitrate;
	} m_stat;

	canid_t m_flag;
	int m_busErrno;

public:
	SocketCan() : CanBus(), m_busName(nullptr), m_bitrate(0), m_socketFd(-1), m_flag(0), m_busErrno(0) {}
	~SocketCan() {
		if(m_socketFd != -1)
			close(m_socketFd);
		if(m_busName)
			free(m_busName);
	}

	int Connect(uint16_t port, uint32_t bitrate);

	int Write(can_frame & f);
	int Read(vector<can_frame> & vf, uint32_t timeout = 10);
	int Write(uint32_t id, uint8_t dlc, uint8_t *data);

	int Print(can_frame & f);

	int BusLoad(); // Return percentage of bus loading

	inline canid_t Flag() const { return m_flag; }
	void ResetFlag() { m_flag = 0; }

	const char *BusName() const { return m_busName; }
	const int BusErrno() const { return m_busErrno; }
	void ResetBusError() { m_busErrno = 0; }
};

#endif
