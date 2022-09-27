#ifndef TCP_CAN_HPP_
#define TCP_CAN_HPP_

#include <climits>
#include <stdint.h>

#include <mutex>

#include "can_bus.hpp"

using namespace std;

typedef struct __attribute__((packed)) {
	union {
		uint8_t info;
		struct {
			uint8_t LEN : 4;
			uint8_t RESVD : 2;
			uint8_t RTR : 1;
			uint8_t FF : 1;
		};
	};
	uint32_t id;
	uint8_t data[8];
} TcpCanFrame;

class TcpCan : public CanBus {
protected:
	char *m_devAddr;
	uint16_t m_port;
	int m_socketFd;
	bool m_isConnected;

	bool m_isWriteCache;
	vector<TcpCanFrame> m_writeCache;
	std::mutex m_socketMutex;

public:
	TcpCan() : CanBus(), m_devAddr(0), m_port(0), m_socketFd(-1), m_isConnected(false), m_isWriteCache(false) {}

	int Connect(const char *devAddr, uint16_t port);
	int Disconnect();
	int Reconnect();
	int Write(TcpCanFrame & f);

	void EnableWriteCache();
	void DisableWriteCache();
	void FlushWriteCache();

	int Read(vector<TcpCanFrame> & vf, uint32_t timeout = 10);
	bool IsConnected() { return (m_socketFd > -1); }

	int Write(uint32_t id, uint8_t dlc, uint8_t *data);
};

#endif
