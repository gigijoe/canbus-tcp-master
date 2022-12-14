#include <cstdlib>
#include <cstring>

#include <thread>

#include <math.h>

#include "tcp_can.hpp"
#include "can_node.hpp"

void printh(const uint8_t *data, size_t size)
{
	for(int i=0;i<size;i++) {
		printf("0x%02x ", data[i]);
		if(i != 0 && i % 16 == 0)
			printf("\n");
	}
	printf("\n");
}

/*
*
*/

const double CanNode::timeout = 2000;

int CanNode::_Read(TcpCanFrame & f) 
{
	if(m_rxTcpCanFrameQueue.size() > 0) {
		f = m_rxTcpCanFrameQueue.front();
		m_rxTcpCanFrameQueue.pop();
		return 0;
	}

	return -1;
}

void CanNode::OnRx(TcpCanFrame & f)
{
	m_rxTcpCanFrameQueue.push(f);

	m_lastRxTimeStamp = steady_clock::now();
}

int CanNode::_Read(can_frame & f) 
{
	if(m_rxCanFrameQueue.size() > 0) {
		f = m_rxCanFrameQueue.front();
		m_rxCanFrameQueue.pop();
		return 0;
	}

	return -1;
}

void CanNode::OnRx(can_frame & f)
{
	m_rxCanFrameQueue.push(f);

	m_lastRxTimeStamp = steady_clock::now();
}

double CanNode::LastRxDuration()
{
	steady_clock::time_point t(steady_clock::now());
	return static_cast<double>(duration_cast<milliseconds>(t - m_lastRxTimeStamp).count());
}

int CanNode::_Write(uint32_t id, uint8_t dlc, uint8_t *data)
{
	int r = 0;
	if(m_tcpCan) {
		r = m_tcpCan->Write(id, dlc, data);
	}
	if(m_socketCan) {
		r = m_socketCan->Write(id, dlc, data);
	}
	return r;
}

void CanNode::DelayMs(uint32_t ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

/*
*
*/

void RMDx6::OnRead(uint8_t data[8])
{
	switch(data[0]) {
		case 0x30: // Read PID
		case 0x31: // Write PID. The content of the reply data is the same as the sent data.
			debug_printf("Current Kp = %u / Ki = %u\n", data[2], data[3]);
			debug_printf("Speed Kp = %u / Ki = %u\n", data[4], data[5]);
			debug_printf("Position Kp = %u / Ki = %u\n", data[6], data[7]);
			m_curKp = data[2];
			m_curKi = data[3];
			m_velKp = data[4];
			m_velKi = data[5];
			m_posKp = data[6];
			m_posKi = data[7];
			break;
		case 0x9A: // Read Status1
			m_temperature = data[1];
			m_voltage = (float)(((uint16_t)data[4] << 8) | data[3]) * 0.1f; // 0.1V 
			{
				uint16_t err = ((uint16_t)data[7] << 8) | data[6];
				if(err & (1 << 1)) {
					debug_printf("M[%d] Blocking !!!\n", m_id);
				}
				if(err & (1 << 2)) {
					debug_printf("M[%d] Low pressure !!!\n", m_id);
				}
				if(err & (1 << 3)) {
					debug_printf("M[%d] Over voltage !!!\n", m_id);
				}
				if(err & (1 << 4)) {
					debug_printf("M[%d] Over current !!!\n", m_id);
				}
				if(err & (1 << 6)) {
					debug_printf("M[%d] Power overrun !!!\n", m_id);
				}
				if(err & (1 << 8)) {
					debug_printf("M[%d] Speeding !!!\n", m_id);
				}
				if(err & (1 << 12)) {
					debug_printf("M[%d] Over temperature !!!\n", m_id);
				}
				if(err & (1 << 13)) {
					debug_printf("M[%d] Encoder calibration error !!!\n", m_id);
				}
			}
			debug_printf("RMDx6[%d] : m_temperature = %d\n", m_id, m_temperature);
			debug_printf("RMDx6[%d] : m_voltage = %f\n", m_id, m_voltage); // 0.1V
			break;
		case 0x90:
			debug_printf("RMDx6[%d] : encoder position = %u\n", m_id, (((uint16_t)data[3] << 8) | data[2]));
			debug_printf("RMDx6[%d] : encoder original position = %u\n", m_id, (((uint16_t)data[5] << 8) | data[4]));
			debug_printf("RMDx6[%d] : encoder zero offset = %u\n", m_id, (((uint16_t)data[7] << 8) | data[6]));
			break;
		case 0x92: // Read Multi Turn Angle
// V2.0 protocol
				m_multiTurnAngle = static_cast<double>(((int64_t)data[6] << 40) +
				((uint64_t)data[5] << 32) +
				((uint64_t)data[4] << 24) +
				((uint64_t)data[3] << 16) +
				((uint64_t)data[2] << 8) +
				data[1]) * 0.01f;
				if(m_reverseDirection)
					m_multiTurnAngle = 0 - m_multiTurnAngle;
				debug_printf("RMDx6[%d] : Multi Turn Angle = (%c)%lf\n", m_id, (data[7] == 0) ? '+' : '-', // 0.01 degree
					m_multiTurnAngle
				);
			break;
		case 0x94: // Read Single Circle Angle
			debug_printf("RMDx6[%d] : Single Circle Angle = %d\n", m_id, (((int32_t)data[7] << 8) + data[6])); // 0.01 degree
			break;
		case 0x9C: // Read Status2
		case 0xA1: // Write torque
		case 0xA4: // Write position
			m_temperature = data[1];
			//m_current = static_cast<float>(((int16_t)data[3] << 8) + data[2]) * 0.01; // 0.01A
			{
				float v = static_cast<float>(((int16_t)data[3] << 8) + data[2]);
				if(v > 2048)
					v = 2048;
				if(v < -2048)
					v = -2048;
				m_current = v * 33.0f / 2048;
			}
			m_velocity = ((int16_t)data[5] << 8) | data[4];
			m_encoderPosition = (((int32_t)data[7] << 8) | data[6]); // 0~65535
			debug_printf("RMDx6[%d] : m_temperature = %d\n", m_id, m_temperature);
			debug_printf("RMDx6[%d] : m_current = %f\n", m_id, m_current);
			debug_printf("RMDx6[%d] : m_velocity = %d\n", m_id, m_velocity); // 1 dps
			debug_printf("RMDx6[%d] : m_encoderPosition = %d\n", m_id, m_encoderPosition); // 0~65535 
			break;
	}	
}

int RMDx6::Read(TcpCanFrame & rf)
{
	if(_Read(rf) < 0)
		return -1;

	if(m_id != (rf.id & 0x1f) || // ID 1~32
			rf.FF != 0 || // Handle standard frame ONLY
			rf.RTR != 0 || // Handle data frame ONLY
			(rf.id & 0xffffffe0) != 0x140)
		return 0; // Nothing to do, handle next frame

	OnRead(&rf.data[0]);

	return 0;
}
/*
static can_frame CanFrame(TcpCanFrame & f)
{
	can_frame cf = {0};
	cf.can_id = f.id;
	cf.can_dlc = f.LEN;
	memcpy(&cf.data[0], &f.data[0], 8);
}
*/

int RMDx6::Read(can_frame & rf) 
{ 
	if(_Read(rf) < 0)
		return -1;

	if(m_id != (rf.can_id & 0x1f) || // ID 1~32
			(rf.can_id & 0xffffffe0) != 0x140)
		return 0; // Nothing to do, handle next frame

	OnRead(&rf.data[0]);

	return 0; 
}

int RMDx6::Write(uint8_t data[8])
{
	return _Write(m_id + 0x140, (uint8_t)0x08, &data[0]);
}

int RMDx6::Reset()
{
	int r = 0;

	uint8_t data[8] = {0};
	data[0] = 0x76;
	r = Write(&data[0]);

	return r;
}

int RMDx6::Shutdown()
{
	int r = 0;

	uint8_t data[8] = {0};
	data[0] = 0x80; // Shutdown
	r = Write(data);

	return r;
}

int RMDx6::ReadStatus(uint8_t id)
{
	int r = 0;

	uint8_t data[8] = {0};
	data[0] = 0x9A; // Read Status1
	r = Write(&data[0]);
	data[0] = 0x9C; // Read Status2
	r = Write(&data[0]);

	return r;
}

int RMDx6::ReadPID()
{
	int r = 0;

	uint8_t data[8] = {0};
	data[0] = 0x30; // Read PID
	r = Write(&data[0]);

	return r;
}

int RMDx6::WritePID(uint8_t posKp, uint8_t posKi, uint8_t velKp, uint8_t velKi, uint8_t curKp, uint8_t curKi)
{
	int r = 0;

	uint8_t data[8] = {0};
	data[0] = 0x31; // Write PID
	data[1] = 0x00;
	data[2] = curKp;
	data[3] = curKi;
	data[4] = velKp;
	data[5] = velKi;
	data[6] = posKp;
	data[7] = posKi;
	r = Write(&data[0]);

	return r;
}

int RMDx6::ReadPosition()
{
	int r = 0;

	uint8_t data[8] = {0};
	//data[0] = 0x9C; // Read Status2
	data[0] = 0x92; // Read Multi Turn Angle
	r = Write(&data[0]);
	data[0] = 0x94;
	r = Write(&data[0]);

	return r;
}

int RMDx6::WritePosition(int32_t position, uint16_t max_speed)
{
	if(m_active == false)
		return 0;

	if(m_maxPos > m_minPos) {
		if(position > m_maxPos)
			position = m_maxPos;
		else if(position < m_minPos)
			position = m_minPos;
	}

	if(m_reverseDirection)
		position = 0 - position;

	// 0~360 degree <-> 0~1638400 / 1 RPM (Round Per Minute)
	int64_t _position = (int64_t)position * 216000 / 36000; // encoder position
	uint16_t _max_speed = max_speed * 6; // dps * 6 (reducer)

	if(_max_speed < 5)
		_max_speed = 5;

	int r = 0;

	uint8_t data[8] = {0};
	data[0] = 0xA4;
	data[1] = 0x00;
	data[2] = _max_speed & 0xFF;
	data[3] = (_max_speed >> 8) & 0xFF;
	data[4] = _position & 0xFF;
	data[5] = (_position >> 8) & 0xFF;
	data[6] = (_position >> 16) & 0xFF;
	data[7] = (_position >> 24) & 0xFF;
	r = Write(&data[0]);

	return r;
}

int RMDx6::WriteTorque(uint16_t torque)
{
	return 0;
}

int RMDx6::WriteBrake(bool onOff)
{
	int r = 0;

	uint8_t data[8] = {0};
	data[0] = onOff ? 0x78 : 0x77;
	r = Write(&data[0]);

	return r;
}

int RMDx6::WriteAcceleration(uint16_t accelerate) // 1 dps/s / 0 ~ 10000
{
	int r = 0;

	uint8_t data[8] = {0};
	data[0] = 0x43;
	data[1] = 0x00;
	data[2] = 0;
	data[3] = 0;
	data[4] = accelerate & 0xFF;
	data[5] = (accelerate >> 8) & 0xFF;
	data[6] = 0;
	data[7] = 0;
	r = Write(&data[0]);

	return r;
}

int RMDx6::WritePosKpKi(uint16_t kp, uint16_t ki)
{
	return WritePID((uint8_t)(kp & 0xff), (uint8_t)(ki & 0xff), 0, 0, 0, 0);
}

int RMDx6::WriteCurrentPositionAsZero()
{
	int r = 0;

	uint8_t data[8] = {0};
	data[0] = 0x64;
	r = Write(data);

	data[0] = 0x76;
	r = Write(data);

	return r;
}

int RMDx6::WriteMaximumCurrent(uint16_t torque)
{
	return 0;
}

/*
*
*/

void RMDx6v3::OnRead(uint8_t data[8])
{
	switch(data[0]) {
		case 0x30: // Read PID
		case 0x31: // Write PID. The content of the reply data is the same as the sent data.
			debug_printf("Current Kp = %u / Ki = %u\n", data[2], data[3]);
			debug_printf("Speed Kp = %u / Ki = %u\n", data[4], data[5]);
			debug_printf("Position Kp = %u / Ki = %u\n", data[6], data[7]);
			m_curKp = data[2];
			m_curKi = data[3];
			m_velKp = data[4];
			m_velKi = data[5];
			m_posKp = data[6];
			m_posKi = data[7];
			break;
		case 0x60:
/*		
			m_encoderPosition = (int32_t)((uint32_t)data[7] << 24) +
				 ((uint32_t)data[6] << 16) +
				 ((uint32_t)data[5] << 8) +
				 (uint32_t)data[4];
*/
			m_encoderPosition = (int32_t)((uint32_t)data[7] << 24) |
				 ((uint32_t)data[6] << 16) |
				 ((uint32_t)data[5] << 8) |
				 (uint32_t)data[4];
			debug_printf("RMDx6v3[%d] : m_encoderPosition = %d\n", m_id, m_encoderPosition); 
			break;
		case 0x92: // Read Multi Turn Angle
/*			
			m_multiTurnAngle = static_cast<double>(((int32_t)data[7] << 24) +
				 ((int32_t)data[6] << 16) +
				 ((int32_t)data[5] << 8) +
				 data[4]) * 0.01f;
*/
			m_multiTurnAngle = static_cast<double>(((int32_t)data[7] << 24) |
				 ((int32_t)data[6] << 16) |
				 ((int32_t)data[5] << 8) |
				 data[4]) * 0.01f;
			if(m_reverseDirection)
				m_multiTurnAngle = 0 - m_multiTurnAngle;
			debug_printf("RMDx6v3[%d] : multi turn angle = %lf\n", m_id, m_multiTurnAngle);
			break;
		case 0x9A: // Read Status1
			m_temperature = data[1];
			m_voltage = (float)(((uint16_t)data[5] << 8) | data[4]) * 0.1f; // 0.1V
			{
				uint16_t err = ((uint16_t)data[7] << 8) | data[6];
				if(err & (1 << 1)) {
					debug_printf("M[%d] Blocking !!!\n", m_id);
				}
				if(err & (1 << 2)) {
					debug_printf("M[%d] Low voltage !!!\n", m_id);
				}
				if(err & (1 << 3)) {
					debug_printf("M[%d] Over voltage !!!\n", m_id);
				}
				if(err & (1 << 4)) {
					debug_printf("M[%d] Over current !!!\n", m_id);
				}
				if(err & (1 << 6)) {
					debug_printf("M[%d] Power overrun !!!\n", m_id);
				}
				if(err & (1 << 8)) {
					debug_printf("M[%d] Speeding !!!\n", m_id);
				}
				if(err & (1 << 12)) {
					debug_printf("M[%d] Over temperature !!!\n", m_id);
				}
				if(err & (1 << 13)) {
					debug_printf("M[%d] Encoder calibration error !!!\n", m_id);
				}
			}
			debug_printf("RMDx6v3[%d] : m_temperature = %d\n", m_id, m_temperature);
			debug_printf("RMDx6v3[%d] : m_voltage = %f\n", m_id, m_voltage); // 0.1V
			break;
		case 0x9C: // Read Status2
		case 0xA1: // Write torque
		case 0xA3: // Write position
		case 0xA4: // Write position
			m_temperature = data[1];
			m_current = static_cast<float>((int16_t)(data[3] << 8) | data[2]) * 0.01; // 0.01A
			//m_current = static_cast<float>(*(int16_t *)&data[2]) * 0.01; // 0.01A
			m_velocity = ((int16_t)(data[5] << 8) | data[4]);
			//m_velocity = *(int16_t *)&data[4];
			// = (((int16_t)data[7] << 8) + data[6]); // +-32767 degree
			
			m_multiTurnAngle = (double)((int16_t)(data[7] << 8) | data[6]);
			//m_multiTurnAngle = (double)(*(int16_t *)&data[6]);
			if(m_reverseDirection)
				m_multiTurnAngle = 0 - m_multiTurnAngle;
			
			debug_printf("RMDx6v3[%d] : m_temperature = %d\n", m_id, m_temperature);
			debug_printf("RMDx6v3[%d] : m_current = %f\n", m_id, m_current);
			debug_printf("RMDx6v3[%d] : m_velocity = %d\n", m_id, m_velocity); // 1 dps
			debug_printf("RMDx6v3[%d] : multi turn angle = %lf\n", m_id, m_multiTurnAngle); // 1 degree / +-32767 degree
			break;
	}	
}

int RMDx6v3::Read(TcpCanFrame & rf)
{
	if(_Read(rf) < 0)
		return -1;

	if(m_id != (rf.id & 0x1f) || // ID 1~32
			rf.FF != 0 || // Handle standard frame ONLY
			rf.RTR != 0 || // Handle data frame ONLY
			(rf.id & 0xffffffe0) != 0x240)
		return 0; // Nothing to do, handle next frame

	OnRead(&rf.data[0]);

	return 0;
}

int RMDx6v3::Read(can_frame & rf) 
{ 
	if(_Read(rf) < 0)
		return -1;

	if(m_id != (rf.can_id & 0x1f) || // ID 1~32
			(rf.can_id & 0xffffffe0) != 0x240)
		return 0; // Nothing to do, handle next frame

	OnRead(&rf.data[0]);

	return 0; 
}

int RMDx6v3::Write(uint8_t data[8])
{
	return _Write(m_id + 0x140, (uint8_t)0x08, &data[0]);
}

int RMDx6v3::Reset()
{
	int r = 0;

	uint8_t data[8] = {0};
	//data[0] = 0x76; // Reset
	//r = Write(data);

	data[0] = 0x20; // Write PID
	data[1] = 0x02; // CAN Filter
	data[2] = 0x00;
	data[3] = 0x00;	
	data[4] = 0x01; // Disable CAN ID filter
	r = Write(data);

	DelayMs(10);

#if 0
	data[0] = 0x20;
	data[1] = 0x04; // Save multi turn angle before power down
	data[2] = 0x00;
	data[3] = 0x00;	
	data[4] = 0x01; // Enable
	r = Write(data);
#endif

	memset(data, 0, 8);
	data[0] = 0x81; // Stop and fix position
	r = Write(data);

	return r;
}

int RMDx6v3::Shutdown()
{
	int r = 0;

	uint8_t data[8] = {0};
	data[0] = 0x80; // Shutdown
	r = Write(data);

	return r;
}

int RMDx6v3::ReadStatus(uint8_t id)
{
	int r = 0;

	uint8_t data[8] = {0};
	data[0] = 0x9A; // Read Status1
	r = Write(data);
	data[0] = 0x9C; // Read Status2
	r = Write(data);

	return r;
}

int RMDx6v3::ReadPID()
{
	int r = 0;

	uint8_t data[8] = {0};
	data[0] = 0x30; // Read PID
	r = Write(data);

	return r;
}

int RMDx6v3::WritePID(uint8_t posKp, uint8_t posKi, uint8_t velKp, uint8_t velKi, uint8_t curKp, uint8_t curKi)
{
	int r = 0;

	uint8_t data[8] = {0};
	data[0] = 0x31; // Write PID
	data[1] = 0x00;
	data[2] = curKp;
	data[3] = curKi;
	data[4] = velKp;
	data[5] = velKi;
	data[6] = posKp;
	data[7] = posKi;
	r = Write(data);

	return r;
}

int RMDx6v3::ReadPosition()
{
	int r = 0;
#if 0
	uint8_t data[8] = {0};
	data[0] = 0x92; // Read Multi Turn Angle
	r = Write(data);
	data[0] = 0x60; // Read multi turn encoder position
	r = Write(data);
#endif
	return r;
}

#if 1
int RMDx6v3::WritePosition(int32_t position, uint16_t max_speed)
{
	if(m_active == false)
		return 0;

	if(m_maxPos > m_minPos) {
		if(position > m_maxPos)
			position = m_maxPos;
		else if(position < m_minPos)
			position = m_minPos;
	}

	if(m_reverseDirection)
		position = 0 - position;
#if 0
	// 0~360 degree <-> 0~1638400 / 1 RPM (Round Per Minute)
	//int64_t _position = (int64_t)position * 288000 / 36000; // encoder position
	uint16_t _max_speed = max_speed * 8; // dps * 8 (reducer)

	if(_max_speed < 8)
		_max_speed = 8;
#else
	uint16_t _max_speed = 400;
#endif
	int r = 0;

	uint8_t data[8] = {0};
	data[0] = 0xA4;
	//data[0] = 0xA5;
	data[1] = 0x00;
	data[2] = _max_speed & 0xFF;
	data[3] = (_max_speed >> 8) & 0xFF;
	data[4] = position & 0xFF;
	data[5] = (position >> 8) & 0xFF;
	data[6] = (position >> 16) & 0xFF;
	data[7] = (position >> 24) & 0xFF;
	r = Write(data);

	return r;
}
#else
int RMDx6v3::WritePosition(int32_t position, uint16_t max_speed)
{
	if(m_active == false)
		return 0;

	if(m_maxPos > m_minPos) {
		if(position > m_maxPos)
			position = m_maxPos;
		else if(position < m_minPos)
			position = m_minPos;
	}

	if(m_reverseDirection)
		position = 0 - position;

	int r = 0;

	uint8_t data[8] = {0};
	data[0] = 0xA3;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = 0x00;
	data[4] = position & 0xFF;
	data[5] = (position >> 8) & 0xFF;
	data[6] = (position >> 16) & 0xFF;
	data[7] = (position >> 24) & 0xFF;
	r = Write(data);

	return r;
}
#endif

int RMDx6v3::WriteTorque(uint16_t torque)
{
	int r = 0;

	uint8_t data[8] = {0};
	data[0] = 0xA1; // Torque
	data[1] = 0x00; 
	data[2] = 0;
	data[3] = 0;
	data[4] = torque & 0xFF;
	data[5] = (torque >> 8) & 0xFF;
	data[6] = 0;
	data[7] = 0;

	return r;
}

int RMDx6v3::WriteBrake(bool onOff)
{
	int r = 0;

	uint8_t data[8] = {0};
	data[0] = onOff ? 0x78 : 0x77;
	r = Write(data);

	return r;
}

int RMDx6v3::WriteAcceleration(uint16_t accelerate) // 1 dps/s / 100 ~ 60000
{
	int r = 0;

	uint8_t data[8] = {0};
	data[0] = 0x43;
	data[1] = 0x00; // Acceleration
	data[2] = 0;
	data[3] = 0;
	data[4] = accelerate & 0xFF;
	data[5] = (accelerate >> 8) & 0xFF;
	data[6] = 0; //(accelerate >> 16) & 0xFF;
	data[7] = 0; //(accelerate >> 24) & 0xFF;

	return r;
}

int RMDx6v3::WriteDeceleration(uint16_t decelerate) // 1 dps/s / 100 ~ 60000
{
	int r = 0;

	uint8_t data[8] = {0};
	data[0] = 0x43;
	data[1] = 0x01; // Deceleration
	data[2] = 0;
	data[3] = 0;
	data[4] = decelerate & 0xFF;
	data[5] = (decelerate >> 8) & 0xFF;
	data[6] = 0; //(decelerate >> 16) & 0xFF;
	data[7] = 0; //(decelerate >> 24) & 0xFF;
	r = Write(data);

	return r;
}

int RMDx6v3::WritePosKpKi(uint16_t kp, uint16_t ki)
{
	return WritePID((uint8_t)(kp & 0xff), (uint8_t)(ki & 0xff), 100, 5, 50, 50);
}

int RMDx6v3::WriteCurrentPositionAsZero()
{
	int r = 0;

	uint8_t data[8] = {0};
	data[0] = 0x64;
	r = Write(data);

	DelayMs(1000);

	data[0] = 0x76;
	r = Write(data);

	return r;
}

int RMDx6v3::WriteMaximumCurrent(uint16_t torque)
{
	return 0;
}

int RMDx6v3::WriteAll(RMDx6v3 & dev, uint8_t data[8])
{
	return dev._Write(0x280, 0x08, data);
}

int RMDx6v3::ResetAll(RMDx6v3 & dev)
{
	int r = 0;

	uint8_t data[8] = {0};
	data[0] = 0x76; // Reset
	r = WriteAll(dev, data);

	data[0] = 0x20; // Write PID
	data[1] = 0x02; // CAN Filter
	data[2] = 0x00;
	data[3] = 0x00;	
	data[4] = 0x01; // Disable CAN ID filter
	r = WriteAll(dev, data);

	return r;
}

/*
*
*/

// ?????????
// ??????   7  6    5      4      3      2      1     0
// ????????? ??? ??? ???????????? ???????????? ???????????? ???????????? ?????? ????????????
// ??????  15 14    13       12    11   10   9  8
// ????????? ??? ??? ??????????????? ??????????????? ??? ???????????? ??? ???

void M8010L::OnRead(uint32_t id, uint8_t data[8])
{
	switch(id & 0xffffff80) { // Strip out device ID
		case 0x180: // TPDO1
		case 0x280: // TPDO2
			m_statusBits.bits = ((int16_t)data[1] << 8) | data[0];
			{
				// TODO : Handle status bits ...
				if(m_statusBits.warning) {
					debug_printf("M8010L[%d] : Warning !!!\n", m_id);
				}
			}

			{
				// 0~360 degree <-> 0~1638400 / 1 RPM (Round Per Minute)
				/*
				int32_t pos32 = (int32_t)((int32_t)data[3] << 24) | 
					((int32_t)data[2] << 16) | 
					((int32_t)data[1] << 8) | 
					data[0];
				*/
				int32_t pos32 = *((int32_t *)&data[0]);
				//debug_printf("\n\n0x%02x 0x%02x 0x%02x 0x%02x\n\n", data[7], data[6], data[5], data[4]);
				m_encoderPosition = pos32;
				debug_printf("M8010L[%d] : m_encoderPosition = %d\n", m_id, m_encoderPosition);
				int64_t pos64 = (int64_t)pos32 * 36000 / 1638400;
				m_multiTurnAngle = static_cast<double>(pos64) * 0.01f; // 0.01 degree
				if(m_reverseDirection)
					m_multiTurnAngle = 0 - m_multiTurnAngle;
				debug_printf("M8010L[%d] : m_multiTurnAngle = %lf\n", m_id, m_multiTurnAngle);
			} break;
		case 0x580: // SDO
			switch(data[0]) {
				case 0x60: // ???????????????
					break;
				case 0x80: // ??????
					printf("M8010L[%d] : Response error !!!\n", m_id);
					break;
				case 0x4f: // ?????????????????????
					break;
				case 0x4b: // ?????????????????????
					break;
				case 0x43: // ?????????????????????
					break;
			}
			if(data[1] == 0x12 && data[2] == 0x26 &&data[3] == 0x00) { // ????????????
				m_temperature = data[4];
				debug_printf("M8010L[%d] : m_temperature = %d\n", m_id, m_temperature);
				//int16_t v = ((int16_t)data[5] << 8) | data[4];
				//printf("M8010L[%d] : m_temperature = %d\n", m_id, v);
			} else if(data[1] == 0x64 && data[2] == 0x60 &&data[3] == 0x00) {
				// 0~360 degree <-> 0~1638400 / 1 RPM (Round Per Minute)
				/*
				int32_t pos32 = (int32_t)((int32_t)data[7] << 24) | 
					((int32_t)data[6] << 16) | 
					((int32_t)data[5] << 8) | 
					data[4];
				*/
				int32_t pos32 = *((int32_t *)&data[4]);
				//debug_printf("\n\n0x%02x 0x%02x 0x%02x 0x%02x\n\n", data[7], data[6], data[5], data[4]);
				m_encoderPosition = pos32;
				debug_printf("M8010L[%d] : m_encoderPosition = %d\n", m_id, m_encoderPosition);
				int64_t pos64 = (int64_t)pos32 * 36000 / 1638400;
				m_multiTurnAngle = static_cast<double>(pos64) * 0.01f; // 0.01 degree
				if(m_reverseDirection)
					m_multiTurnAngle = 0 - m_multiTurnAngle;
				debug_printf("M8010L[%d] : m_multiTurnAngle = %lf\n", m_id, m_multiTurnAngle);
			} else if(data[1] == 0x78 && data[2] == 0x60 &&data[3] == 0x00) { // ????????????
				uint32_t a = ((uint32_t)data[5] << 8) | data[4];
				m_current = ((float)a / 200.0f); // a / 200 A
				debug_printf("M8010L[%d] : m_current = %f\n", m_id, m_current); // 1.0 A
			} else if(data[1] == 0x79 && data[2] == 0x60 &&data[3] == 0x00) { // ????????????
				//uint32_t v = ((int32_t)data[5] << 8) + data[4];
				//m_voltage = (v * 100 / 327) & 0xffff;
				uint16_t v = ((int16_t)data[5] << 8) | data[4];
				m_voltage = (float)v / 327.0f;
				debug_printf("M8010L[%d] : m_voltage = %f\n", m_id, m_voltage); // 1.0 V 
			} else if(data[1] == 0x60 && data[2] == 0x60 &&data[3] == 0x00) {
				// ????????????????????????
			}
			break;
		case 0x700: // Heart beat
			switch(data[0]) {
				case 0x05: // ??????????????????
					debug_printf("M8010L[%d] : heart beat ...\n", m_id);
					break;
				case 0x04: // ????????????
					debug_printf("M8010L[%d] : heart beat alarm !!!\n", m_id);
					break;
			}
			break;
		default: debug_printf("Unknown frame !!!\n");
			break;
	}

}

int M8010L::Read(TcpCanFrame & rf)
{
	if(_Read(rf) < 0)
		return -1;

	//if(m_id != (rf.id & 0x7f)) // ID 1~127
	if(m_id != (rf.id & 0x1f) || // ID 1~32, Limited
			rf.FF != 0 || // Handle standard frame ONLY
			rf.RTR != 0) // Handle data frame ONLY
		return 0; // Nothing to do, handle next frame

	OnRead(rf.id, &rf.data[0]);

	return 0;
}

int M8010L::Read(can_frame & rf) 
{
	if(_Read(rf) < 0)
		return -1;

	//if(m_id != (rf.id & 0x7f)) // ID 1~127
	if(m_id != (rf.can_id & 0x1f))
		return 0; // Nothing to do, handle next frame

	OnRead(rf.can_id, &rf.data[0]);

	return 0;
}

int M8010L::Reset()
{
	int r = 0;

	uint8_t data[8] = {0};
// ?????????????????????
// 0x2F=??????????????????
// 0x2B=??????????????????
// 0x23=??? 4 ????????????
	data[0] = 0x2f; // ???????????????
	data[1] = 0x60; // Index 0x6060
	data[2] = 0x60; 
	data[3] = 0x00; // Subindex 0x00
// ????????????:
// 1???????????????
// 3???????????????
// 6??????????????????
// 7????????? CANopen ???????????????	
	data[4] = 0x01; // ????????????
	data[5] = 0x00;
	data[6] = 0x00;
	data[7] = 0x00;

	r = _Write(0x600 + m_id, // SDO : 0x600 + ID(1~127)
		0x08, data); // Data length is 8 bytes

// ???????????? (??????????????????)

	const uint16_t accelerate = 200;
// ?????????????????????
// 0x2F=??????????????????
// 0x2B=??????????????????
// 0x23=??? 4 ????????????
	data[0] = 0x23; // ???4????????? 
	data[1] = 0x81; // Index 0x6081
	data[2] = 0x60; 
	data[3] = 0x00; // Subindex 0x00
	data[4] = accelerate & 0xff; 
	data[5] = (accelerate >> 8) & 0xff;
	data[6] = 0x00; 
	data[7] = 0x00; 

	r = _Write(0x600 + m_id, // SDO : 0x600 + ID(1~127)
		0x08, data); // Data length is 8 bytes

// ?????????????????????

	const uint16_t velKp = 500; // 0 ~ 10000 ????????????????????????

// ?????????????????????
// 0x2F=??????????????????
// 0x2B=??????????????????
// 0x23=??? 4 ????????????
	data[0] = 0x23; // ???4????????? 
	data[1] = 0xf9; // Index 0x60f9
	data[2] = 0x60; 
	data[3] = 0x01; // Subindex 0x01
	data[4] = velKp & 0xff; 
	data[5] = (velKp >> 8) & 0xff;
	data[6] = 0x00; 
	data[7] = 0x00; 

	r = _Write(0x600 + m_id, // SDO : 0x600 + ID(1~127)
		0x08, data); // Data length is 8 bytes

// ?????????????????????

	const uint16_t velKi = 200; // 2~2000ms ????????????????????????

// ?????????????????????
// 0x2F=??????????????????
// 0x2B=??????????????????
// 0x23=??? 4 ????????????
	data[0] = 0x23; // ???4????????? 
	data[1] = 0xf9; // Index 0x60f9
	data[2] = 0x60; 
	data[3] = 0x02; // Subindex 0x02
	data[4] = velKi & 0xff; 
	data[5] = (velKi >> 8) & 0xff;
	data[6] = 0x00; 
	data[7] = 0x00; 

	r = _Write(0x600 + m_id, // SDO : 0x600 + ID(1~127)
		0x08, data); // Data length is 8 bytes

	return r;
}

int M8010L::Shutdown()
{
	int r = 0;

	uint8_t data[8] = {0};
// ?????????????????????
// 0x2F=??????????????????
// 0x2B=??????????????????
// 0x23=??? 4 ????????????
	data[0] = 0x2B; // ???????????????
	data[1] = 0x40; // Index 0x6040
	data[2] = 0x60; 
	data[3] = 0x00; // Subindex 0x00
// ????????????:
// 1???????????????
// 3???????????????
// 6??????????????????
// 7????????? CANopen ???????????????	
	data[4] = 0x00; // ????????????
	data[5] = 0x00;
	data[6] = 0x00;
	data[7] = 0x00;

	r = _Write(0x600 + m_id, // SDO : 0x600 + ID(1~127)
		0x08, data); // Data length is 8 bytes

	return r;
}

int M8010L::ReadStatus(uint8_t id)
{
	int r = 0;

	uint8_t data[8] = {0};
// ?????????????????????
// 0x40=?????????
	data[0] = 0x40; // ??????
	data[1] = 0x12; // Index 0x2612 
	data[2] = 0x26; 
	data[3] = 0x00; // Subindex 0x00 : ????????????
	data[4] = 0x00; 
	data[5] = 0x00;
	data[6] = 0x00; 
	data[7] = 0x00; 

	r = _Write(0x600 + m_id, // SDO : 0x600 + ID(1~127)
		0x08, data); // Data length is 8 bytes

// ?????????????????????
// 0x40=?????????
	data[0] = 0x40; // ??????
	data[1] = 0x78; // Index 0x6078
	data[2] = 0x60; 
	data[3] = 0x00; // Subindex 0x00 : ????????????
	data[4] = 0x00; 
	data[5] = 0x00;
	data[6] = 0x00; 
	data[7] = 0x00; 

	r = _Write(0x600 + m_id, // SDO : 0x600 + ID(1~127)
		0x08, data); // Data length is 8 bytes

// ?????????????????????
// 0x40=?????????
	data[0] = 0x40; // ??????
	data[1] = 0x79; // Index 0x6079
	data[2] = 0x60; 
	data[3] = 0x00; // Subindex 0x00 : ????????????
	data[4] = 0x00; 
	data[5] = 0x00;
	data[6] = 0x00; 
	data[7] = 0x00; 

	r = _Write(0x600 + m_id, // SDO : 0x600 + ID(1~127)
		0x08, data); // Data length is 8 bytes

	return r;
}

int M8010L::ReadPID()
{
	int r = 0;

	uint8_t data[8] = {0};
// ?????????????????????
// 0x40=?????????
	data[0] = 0x40; // ??????
	data[1] = 0xfb; // Index 0x60fb
	data[2] = 0x60; 
	data[3] = 0x01; // Subindex 0x01 : ?????????????????????
	data[4] = 0x00; 
	data[5] = 0x00;
	data[6] = 0x00; 
	data[7] = 0x00; 

	r = _Write(0x600 + m_id, // SDO : 0x600 + ID(1~127)
		0x08, data); // Data length is 8 bytes

	return r;
}

int M8010L::WritePID(uint8_t posKp, uint8_t posKi, uint8_t velKp, uint8_t velKi, uint8_t curKp, uint8_t curKi)
{
	return 0; // Not implement !!! Call WritePosKpKi instead
}

int M8010L::ReadPosition()
{
	int r = 0;

	uint8_t data[8] = {0};
// ?????????????????????
// 0x40=?????????
	data[0] = 0x40; // ??????
	data[1] = 0x64; // Index 0x6064
	data[2] = 0x60; 
	data[3] = 0x00; // Subindex 0x00 : ????????????
	data[4] = 0x00; 
	data[5] = 0x00;
	data[6] = 0x00; 
	data[7] = 0x00; 

	r = _Write(0x600 + m_id, // SDO : 0x600 + ID(1~127)
		0x08, data); // Data length is 8 bytes

	return r;
}

// ?????????
// ?????? 15:9  8     7        6         5         4         3      2      1     0
// ????????? ???  ?????? ???????????? 0??????????????? ?????????????????? ?????????????????? ???????????? ???????????? ???????????? ??????
//                     1???????????????

// ???????????????????????????
// 0x60=??????????????????
// 0x80=?????????

int M8010L::WritePosition(int32_t position, uint16_t max_speed) // unit : 0.01 degree / 1 dps (Degree Per Second)
{
	if(m_active == false)
		return 0;

	if(m_maxPos > m_minPos) {
		if(position > m_maxPos)
			position = m_maxPos;
		else if(position < m_minPos)
			position = m_minPos;
	}

	if(m_reverseDirection)
		position = 0 - position;

	// 0~360 degree <-> 0~1638400 / 1 RPM (Round Per Minute)
	int64_t _position = (int64_t)position * 1638400 / 36000; // encoder position
	uint16_t _max_speed = (max_speed + 6) / 6 * 200; // dps to rpm (50 reducer)
	if(_max_speed < 200)
		_max_speed = 200;

	int r = 0;
	uint8_t data[8] = {0};
// ????????????+????????????
	data[0] = _position & 0xFF;
	data[1] = (_position >> 8) & 0xFF;
	data[2] = (_position >> 16) & 0xFF;
	data[3] = (_position >> 24) & 0xFF;
	data[4] = _max_speed & 0xFF;
	data[5] = (_max_speed >> 8) & 0xFF;
	data[6] = 0;
	data[7] = 0;

	r = _Write(0x300 + m_id, // RPDO2 : 0x300 + ID(1~127)
		0x08, data); // Data length is 8 bytes

// ?????????+????????????+????????????
	data[0] = 0x2f; // ??????????????????+??????+????????????+????????????+????????????
	data[1] = 0x00;
// ????????????:
// 1???????????????
// 3???????????????
// 6??????????????????
// 7????????? CANopen ???????????????
	data[2] = 0x01; // ????????????
// ???????????? 0 ~ 1638400
	data[3] = _position & 0xFF;
	data[4] = (_position >> 8) & 0xFF;
	data[5] = (_position >> 16) & 0xFF;
	data[6] = (_position >> 24) & 0xFF;
	data[7] = 0x00;

	r = _Write(0x200 + m_id, // RPDO1 : 0x200 + ID(1~127)
		0x07, data); // Data length is 7 bytes

	return r;
}

int M8010L::WriteTorque(uint16_t torque)
{
	return 0;
}

int M8010L::WriteBrake(bool onOff)
{
	int r = 0;
	uint8_t data[8] = {0};
	if(onOff) {
// ?????????????????????
// 0x2F=??????????????????
// 0x2B=??????????????????
// 0x23=??? 4 ????????????
		data[0] = 0x2b; // ??????????????? 
		data[1] = 0x40; // Index 0x6040
		data[2] = 0x60; 
		data[3] = 0x00; // Subindex 0x00
		data[4] = 0x0e; // ????????????+????????????+????????????
		data[5] = 0x01; // ??????
	} else {
		data[0] = 0x2b; // ??????????????? 
		data[1] = 0x40; // Index 0x6040
		data[2] = 0x60; 
		data[3] = 0x00; // Subindex 0x00
		data[4] = 0x0f; // ??????+????????????+????????????+????????????
		data[5] = 0x00;
	}
	data[6] = 0x00;
	data[7] = 0x00;

	r = _Write(0x600 + m_id, // SDO : 0x600 + ID(1~127)
		0x08, data); // Data length is 8 bytes

	return r;
}

int M8010L::WriteAcceleration(uint16_t accelerate)
{
	int r = 0;

	uint8_t data[8] = {0};
// ?????????????????????
// 0x2F=??????????????????
// 0x2B=??????????????????
// 0x23=??? 4 ????????????
	data[0] = 0x23; // ???4????????? 
	data[1] = 0x83; // Index 0x6083
	data[2] = 0x60; 
	data[3] = 0x00; // Subindex 0x00
	data[4] = accelerate & 0xff; 
	data[5] = (accelerate >> 8) & 0xff;
	data[6] = 0x00; 
	data[7] = 0x00; 

	r = _Write(0x600 + m_id, // SDO : 0x600 + ID(1~127)
		0x08, data); // Data length is 8 bytes

	return r;
}

int M8010L::WritePosKpKi(uint16_t kp, uint16_t ki)
{
	if(kp > 30000)
		kp = 30000;
	else if(kp < 60)
		kp = 60;

	int r = 0;

	uint8_t data[8] = {0};
// ?????????????????????
// 0x2F=??????????????????
// 0x2B=??????????????????
// 0x23=??? 4 ????????????
	data[0] = 0x2b; // ??????????????? 
	data[1] = 0xfb; // Index 0x60fb
	data[2] = 0x60;
	data[3] = 0x01; // Subindex 0x01
	data[4] = (kp & 0xff); 
	data[5] = (kp >> 8) & 0xff;
	data[6] = 0x00; 
	data[7] = 0x00; 

	r = _Write(0x600 + m_id, // SDO : 0x600 + ID(1~127)
		0x08, data); // Data length is 8 bytes

	return r;
}

int M8010L::WriteHeartBeatInterval(uint16_t ms)
{
	int r = 0;

	uint8_t data[8] = {0};
// ?????????????????????
// 0x2F=??????????????????
// 0x2B=??????????????????
// 0x23=??? 4 ????????????
	data[0] = 0x2b; // ??? 2 ????????? 
	data[1] = 0x17; // Index 0x1017
	data[2] = 0x10; 
	data[3] = 0x00; // Subindex 0x00
	data[4] = (ms & 0xff); 
	data[5] = (ms >> 8) & 0xff;
	data[6] = 0x00; 
	data[7] = 0x00; 

	r = _Write(0x700 + m_id, // SDO : 0x700 + ID(1~127)
		0x08, data); // Data length is 8 bytes

	return r;
}

int M8010L::WriteCurrentPositionAsZero()
{
	int r = 0;

	uint8_t data[8] = {0};
// ?????????????????????
// 0x2F=??????????????????
// 0x2B=??????????????????
// 0x23=??? 4 ????????????
	data[0] = 0x2B; // ??? 2 ????????? 
	data[1] = 0x0A; // Index 0x1017
	data[2] = 0x26; 
	data[3] = 0x00; // Subindex 0x00
	data[4] = 0x70; 
	data[5] = 0xEA;
	data[6] = 0x00; 
	data[7] = 0x00; 

	r = _Write(0x600 + m_id, // SDO : 0x600 + ID(1~127)
		0x08, data); // Data length is 8 bytes

	DelayMs(1000);

	data[4] = 0x66; 

	r = _Write(0x600 + m_id, // SDO : 0x600 + ID(1~127)
		0x08, data); // Data length is 8 bytes

	return r;
}

int M8010L::WriteMaximumCurrent(uint16_t amp001)
{
	const uint16_t alarm_timeout = 2; // Alarm if reach maximum current over 2 seconds 

	if(amp001 > 1000)
		amp001 = 1000; // 10A
	amp001 = amp001 * 10 + alarm_timeout; // 10000 -> 10.000A

	int r = 0;
	uint8_t data[8] = {0};
// ?????????????????????
// 0x2F=??????????????????
// 0x2B=??????????????????
// 0x23=??? 4 ????????????
	data[0] = 0x2b; // ??????????????? 
	data[1] = 0x1d; // Index 0x261d
	data[2] = 0x26; 
	data[3] = 0x00; // Subindex 0x00
	data[4] = amp001 & 0xFF; 
	data[5] = (amp001 >> 8) & 0xFF;
	data[6] = 0x00; 
	data[7] = 0x00;

	r = _Write(0x600 + m_id, // SDO : 0x600 + ID(1~127)
		0x08, data); // Data length is 8 bytes

	return r;
}
