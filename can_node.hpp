#ifndef CAN_NODE_HPP_
#define CAN_NODE_HPP_

#include <climits>
#include <stdint.h>

#include <queue>
#include <mutex>
#include <condition_variable>

#include <linux/can.h>

using std::chrono::steady_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::seconds;

using namespace std;

#define DEBUG 0
#define debug_printf(fmt, ...) \
			do { if (DEBUG) fprintf(stderr, fmt, ##__VA_ARGS__); } while (0)

void printh(const uint8_t *data, size_t size);

/*
*
*/

#include "tcp_can.hpp"
#include "socket_can.hpp"

class CanNode {
protected:
	TcpCan *m_tcpCan;
	SocketCan *m_socketCan;
	uint8_t m_id;

	TcpCanFrame m_tcpCanFrame;
	queue<TcpCanFrame> m_rxTcpCanFrameQueue;
	queue<can_frame> m_rxCanFrameQueue;

	steady_clock::time_point m_lastRxTimeStamp;

	bool m_active;

	int _Read(TcpCanFrame & rf);
	int _Read(can_frame & rf);

	int _Write(uint32_t id, uint8_t dlc, uint8_t *data);

public:
	CanNode(TcpCan & tcpCan, uint8_t id) : m_tcpCan(&tcpCan), m_socketCan(nullptr), 
		m_id(id), m_lastRxTimeStamp(), m_active(true) {}
	CanNode(SocketCan & socketCan, uint8_t id) : m_tcpCan(nullptr), m_socketCan(&socketCan), 
		m_id(id), m_lastRxTimeStamp(), m_active(true) {}

	virtual int Read(TcpCanFrame & rf) = 0;
	virtual int Read(can_frame & rf) = 0;

	void OnRx(TcpCanFrame & f);
	void OnRx(can_frame & f);

	static const double timeout; // ms
	double LastRxDuration();

	inline TcpCan *TcpCanInstance() const { return m_tcpCan; }
	inline SocketCan *SocketCanInstance() const { return m_socketCan; }

	inline const bool IsActive() const { return m_active; }
	inline void Active() { m_active = true; }
	inline void Deactive() { m_active = false; }
};

class CanMotor : public CanNode {
protected:
	int8_t m_temperature; // R
	uint8_t m_posKp, m_posKi, m_velKp, m_velKi, m_curKp, m_curKi; // R
	float m_voltage, m_current;
	int16_t m_velocity;

	int32_t m_encoderPosition; 
	double m_multiTurnAngle;
	bool m_reverseDirection;
	int32_t m_maxPos, m_minPos;

public:
	CanMotor(TcpCan & tcpCan, uint8_t id) : CanNode(tcpCan, id), m_temperature(-40), 
		m_posKp(255), m_posKi(255), m_velKp(255), m_velKi(255), m_curKp(255), m_curKi(255),
		m_voltage(0.0f), m_current(0.0f), m_velocity(0), 
		m_encoderPosition(0), m_multiTurnAngle(0.0f), m_reverseDirection(false), m_maxPos(0), m_minPos(0) {}

	CanMotor(SocketCan & socketCan, uint8_t id) : CanNode(socketCan, id), m_temperature(-40), 
		m_posKp(255), m_posKi(255), m_velKp(255), m_velKi(255), m_curKp(255), m_curKi(255),
		m_voltage(0.0f), m_current(0.0f), m_velocity(0), 
		m_encoderPosition(0), m_multiTurnAngle(0.0f), m_reverseDirection(false), m_maxPos(0), m_minPos(0) {}

	virtual int Reset() = 0;
	virtual int ReadStatus(uint8_t id = 0) = 0;
	virtual int ReadPID() = 0;
	virtual int WritePID(uint8_t posKp, uint8_t posKi, uint8_t velKp, uint8_t velKi, uint8_t curKp, uint8_t curKi) = 0;
	virtual int ReadPosition() = 0;
	virtual int WritePosition(int32_t position, uint16_t max_speed) = 0;
	virtual int WriteTorque(uint16_t torque) = 0;
	virtual int WriteBrake(bool onOff) = 0;
	virtual int WriteAcceleration(uint16_t accelerate) = 0;
	virtual int WriteDeceleration(uint16_t decelerate) { printf("Setup deceleration NOT support !!!\n"); return 0; }
	virtual int WritePosKpKi(uint16_t kp, uint16_t ki) = 0;
	virtual int WriteHeartBeatInterval(uint16_t ms) { printf("Heart beat NOT support !!!\n"); return 0; }
	virtual int WriteCurrentPositionAsZero() = 0;

	void PrintStatus() {
		printf("Temp : %d\n", m_temperature);
		printf("Voltage : %f\n", m_voltage);
		printf("Current : %f\n", m_current);
		printf("Position : %u / Kp : %d, Ki : %d\n", m_encoderPosition, m_posKp, m_posKi);
	}

	inline int32_t EncoderPosition() const { return m_encoderPosition; }
	inline double MultiTurnAngle() const { return m_multiTurnAngle; }
	inline float Voltage() const { return m_voltage; }
	inline float Current() const { return m_current; }
	inline int8_t Temperature() const { return m_temperature; }

	inline void ReverseDirection(bool yesNo = true) { m_reverseDirection = yesNo; }
	inline void PositionLimitation(int32_t maxPos, int32_t minPos) {
		m_maxPos = maxPos;
		m_minPos = minPos;
	}
};

class RMDx6 : public CanMotor { // V2 protocol
private:
	void OnRead(uint8_t data[8]);

	int Write(uint8_t data[8]);

public:
	RMDx6(TcpCan & tcpCan, uint8_t id) : CanMotor(tcpCan, id) {}
	RMDx6(SocketCan & socketCan, uint8_t id) : CanMotor(socketCan, id) {}

	int Read(TcpCanFrame & rf);
	int Read(can_frame & rf);

	int Reset();
	int ReadStatus(uint8_t id = 0);
	int ReadPID();
	int WritePID(uint8_t posKp, uint8_t posKi, uint8_t velKp, uint8_t velKi, uint8_t curKp, uint8_t curKi);
	int ReadPosition();
	int WritePosition(int32_t position, uint16_t max_speed); // unit : 0.01degree / 1 dps (Degree Per Second)
	int WriteTorque(uint16_t torque); // unit : 0.01A
	int WriteBrake(bool onOff);
	int WriteAcceleration(uint16_t accelerate);
	int WritePosKpKi(uint16_t kp, uint16_t ki);
	int WriteCurrentPositionAsZero();
};

class RMDx6v3 : public CanMotor { // V3 protocol
private:
	void OnRead(uint8_t data[8]);

	int Write(uint8_t data[8]);

public:
	RMDx6v3(TcpCan & tcpCan, uint8_t id) : CanMotor(tcpCan, id) {}
	RMDx6v3(SocketCan & socketCan, uint8_t id) : CanMotor(socketCan, id) {}

	int Read(TcpCanFrame & rf);
	int Read(can_frame & rf);

	int Reset();
	int ReadStatus(uint8_t id = 0);
	int ReadPID();
	int WritePID(uint8_t posKp, uint8_t posKi, uint8_t velKp, uint8_t velKi, uint8_t curKp, uint8_t curKi);
	int ReadPosition();
	int WritePosition(int32_t position, uint16_t max_speed); // unit : 0.01degree / 1 dps (Degree Per Second)
	int WriteTorque(uint16_t torque); // unit : 0.01A
	int WriteBrake(bool onOff);
	int WriteAcceleration(uint16_t accelerate);
	int WriteDeceleration(uint16_t decelerate);
	int WritePosKpKi(uint16_t kp, uint16_t ki);
	int WriteCurrentPositionAsZero();

	static int WriteAll(RMDx6v3 & dev, uint8_t data[8]);
	static int ResetAll(RMDx6v3 & dev);
};

class M8010L : public CanMotor {
private:
	typedef struct __attribute__((packed)) {
		union {
			uint16_t bits;
			struct {
				uint8_t n3 : 2;
				uint8_t errorOrigin : 1;
				uint8_t reachOrigin : 1;
				uint8_t n2 : 1;
				uint8_t reachTarget : 1;
				uint8_t n1 : 4;
				uint8_t emergencyStopAllow : 1;
				uint8_t voltageOut : 1;
				uint8_t warning : 1;
				uint8_t opAllow : 1;
				uint8_t active : 1;
				uint8_t ready : 1;
			};
		};
	} StatusBits;

	StatusBits m_statusBits;

	void OnRead(uint32_t id, uint8_t data[8]);

public:
	M8010L(TcpCan & tcpCan, uint8_t id) : CanMotor(tcpCan, id), m_statusBits() {}
	M8010L(SocketCan & socketCan, uint8_t id) : CanMotor(socketCan, id), m_statusBits() {}

	int Read(TcpCanFrame & rf);
	int Read(can_frame & rf);

	int Reset();
	int ReadStatus(uint8_t id = 0);
	int ReadPID();
	int WritePID(uint8_t posKp, uint8_t posKi, uint8_t velKp, uint8_t velKi, uint8_t curKp, uint8_t curKi);
	int ReadPosition();
	int WritePosition(int32_t position, uint16_t max_speed); // unit : 0.01degree / 1 dps (Degree Per Second)
	int WriteTorque(uint16_t torque); // unit : 0.01A
	int WriteBrake(bool onOff);
	int WriteAcceleration(uint16_t accelerate);
	int WritePosKpKi(uint16_t kp, uint16_t ki);
	int WriteHeartBeatInterval(uint16_t ms);
	int WriteCurrentPositionAsZero();
};

#endif
