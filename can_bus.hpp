#ifndef CAN_BUS_HPP_
#define CAN_BUS_HPP_

#include <stdint.h>
#include <vector>

using namespace std;

class CanMotor;

class CanBus {
protected:
	vector<CanMotor *> m_devices;

public:
	CanBus() {}

	inline CanMotor *Device(uint8_t index) const {
		if(index >= m_devices.size())
			return nullptr;
		return m_devices[index];
	}

	inline uint8_t NumberDevices() const { return m_devices.size(); }

	void AddDevice(CanMotor *dev) {
		m_devices.push_back(dev);
	}

	void RemoveAllDevices() {
		m_devices.clear();
	}

	virtual int Write(uint32_t id, uint8_t dlc, uint8_t *data) = 0;
	//virtual int Read(uint8_t id, uint8_t *data) = 0;
};

#endif
