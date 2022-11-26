//#include <opencv2/opencv.hpp>
//#include <opencv2/videoio.hpp>

//#include <opencv2/cudafilters.hpp>
//#include <opencv2/cudabgsegm.hpp>
//#include <opencv2/cudaobjdetect.hpp>
//#include <opencv2/cudaarithm.hpp>
//#include <opencv2/cudaimgproc.hpp>
#include <math.h>

#include <string>
#include <iostream>
#include <chrono>
#include <sstream>
#include <algorithm>
#include <numeric>

#include <thread>
#include <mutex>

#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <termios.h>

//#include "modbus/modbus.h"
#include "csv_parser.h"
//#include "color.h"

#include "socket_can.hpp"
#include "can_node.hpp"

#include <lcm/lcm-cpp.hpp>
#include "servolcm/position_t.hpp"
#include "servolcm/status_t.hpp"
#include "servolcm/pwm_t.hpp"

#include "protolcm/status_t.hpp"
#include "protolcm/command_t.hpp"
#include "protolcm/region_t.hpp"

std::condition_variable s_emergency;

static lcm::LCM s_lcm("udpm://239.255.76.67:7667?ttl=1");

using std::chrono::steady_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::seconds;

//using namespace cv;
using namespace std;

static bool bExit = false;

void sig_handler(int signo)
{
	if(signo == SIGINT) {
		printf("SIGINT\n");
		bExit = true;
	}
}

#define EMERGENCY_GPIO 4

template <class T>
void CanMotorInitialize(CanMotor *cm)
{
	cm->Reset();
	cm->WriteBrake(false);
	cm->WriteMaximumCurrent(100); // 1A
	if(typeid(T) == typeid(RMDx6)) {
// X6 default 
/*
Current Kp = 100 / Ki = 100
Speed Kp = 50 / Ki = 40
Position Kp = 50 / Ki = 50
*/
		cm->WritePID(50, 50, 10, 8, 100, 100); // Kp, Ki from 0 ~ 255
	} else if(typeid(T) == typeid(RMDx6v3)) {
		cm->WriteAcceleration(6000); // 100 ~ 60000 dps/s
		cm->WriteDeceleration(3000); // 100 ~ 60000 dps/s
		cm->WritePosKpKi(100, 5); // Kp, Ki from 0 ~ 255
	} else if(typeid(T) == typeid(M8010L)) {
		cm->ReverseDirection();
		cm->WriteAcceleration(65535); // Disable trapezoidal acceleration pluse
		cm->WritePosKpKi(300, 0); // Kp from 60 ~ 30000
		cm->WriteHeartBeatInterval(0); // Heart beat interval in ms. 0 disabled.
		//cm->PositionLimitation(0, -10000); // 0 to -100 degree
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

#include <future>

#define STATUS_THREAD 

template <class T>
void SocketCanThread(SocketCan & socketCan, uint16_t port, uint32_t bitrate, uint8_t numMotors)
{
	socketCan.Connect(port, bitrate);

	for(uint8_t i=0;i<numMotors;i++) {
		socketCan.AddDevice(new T(socketCan, i+1)); // ID range from 1 to 32
	}

	while(!bExit) {
		//for(uint8_t i=0;i<socketCan.NumberDevices();i++) {
		for(int8_t i=socketCan.NumberDevices()-1;i>=0;i--) {
			CanMotorInitialize<T>(socketCan.Device(i));
			if(typeid(T) == typeid(RMDx6)) {
			} else if(typeid(T) == typeid(RMDx6v3)) {
			} else if(typeid(T) == typeid(M8010L)) {
				if(i < 15)
					socketCan.Device(i)->PositionLimitation(0, -10000); // 0 to -100 degree
				else
					socketCan.Device(i)->PositionLimitation(0, -9500); // 0 to -95 degree
			}
		}

#ifdef STATUS_THREAD // Status thread causes DNET400
		std::promise<void> exitSignal;
		std::future<void> futureObj = exitSignal.get_future();
		
		std::thread t([&socketCan](std::future<void> futureObj) -> void { // Capture local variable 'socketCan' by reference
			const long long _interval = 10; // ms
			while(!bExit) {
				//for(uint8_t i=0;i<socketCan.NumberDevices();i++) {
				for(int8_t i=socketCan.NumberDevices()-1;i>=0;i--) {
					if(socketCan.Device(i) == nullptr)
						continue;
					socketCan.Device(i)->ReadStatus();

					if(futureObj.wait_for(std::chrono::milliseconds(_interval)) == std::future_status::ready)
						break;
				}

				//for(uint8_t i=0;i<socketCan.NumberDevices();i++) {
				for(int8_t i=socketCan.NumberDevices()-1;i>=0;i--) {
					if(socketCan.Device(i) == nullptr)
						continue;
					socketCan.Device(i)->ReadPosition();
		
					if(futureObj.wait_for(std::chrono::milliseconds(_interval)) == std::future_status::ready)
						break;
				}
			}
		}, std::move(futureObj));
#endif
		while(!bExit) {
			vector<can_frame> vf;
			int r = socketCan.Read(vf, 100); // 100ms timeout
			if(r > 0) {
				for(auto it = vf.begin();it != vf.end();++it) {
					uint8_t id = it->can_id & 0x1f; // ID 1~32
					if(id == 0)
						continue;
					uint8_t i = id - 1;
					if(socketCan.Device(i) == nullptr)
						continue;
					
					socketCan.Device(i)->OnRx(*it);
				}
				vf.clear();
			} else if(r < 0) { // Connection closed !!! 
				printf("Connection error !!!\n");
				std::this_thread::sleep_for(std::chrono::seconds(1));
				break;
			} else { // Read timeout ...
				continue; 
			}

			//for(uint8_t i=0;i<socketCan.NumberDevices();i++) {
			for(int8_t i=socketCan.NumberDevices()-1;i>=0;i--) {
				if(socketCan.Device(i) == nullptr)
					continue;
				can_frame f;

				while(socketCan.Device(i)->Read(f) == 0) {} // Handle all packets
			}
		}

#ifdef STATUS_THREAD
		exitSignal.set_value();
		t.join();
#endif
	}

	for(uint8_t i=0;i<socketCan.NumberDevices();i++) {
		delete socketCan.Device(i); // ID range from 1 to 32
	}

	socketCan.RemoveAllDevices();
}

/*
*
*/

#define NUM_DEV_PER_SOCKETCAN 25
#define NUM_SOCKETCAN 2

static SocketCan s_socketCan[NUM_SOCKETCAN];

/*
*
*/

static int select_stdin(long us_timeout)
{
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = us_timeout; // us

	fd_set fds;
	memset(&fds, 0, sizeof(fds));
	FD_SET(fileno(stdin), &fds);
	int r = select(fileno(stdin)+1, &fds, NULL, NULL, &tv);

	return r;	
}

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <poll.h>

static int read_gpio(int gpio)
{
	char buf[64];
	snprintf(buf, 64, "/sys/class/gpio/gpio%d/value", gpio);
	int fd = open(buf, O_RDONLY);
	if(fd < 0)
		return -1;
	
	char valstr[3];
	if(read(fd, valstr, 3) < 0) {
		close(fd);
		return -1;
	}

	close(fd);

	return atoi(valstr);
}

/*
*
*/

#include "atox.h"

typedef struct {
	float pitch, roll;
	float timestamp;
} Petal;

#define NUM_PETAL 25
#define NUM_EXTRA_PETAL 7
static vector<Petal> s_petals[NUM_PETAL + NUM_EXTRA_PETAL];

static bool parse_flower_csv(const char *filename) {
	FILE *fp = fopen(filename, "r");
	if(!fp) {
		printf("Fail open %s => %s\n", filename, strerror(errno));
		return false;
	}

	for(int i=0;i<NUM_PETAL;i++) {
		s_petals[i].clear();
	}

	char line[256];
	while(fgets(line, 256, fp) != NULL) {
		char *p = &line[0];
		size_t sl = strlen(p);
		if(sl == 0)
			continue;
		
		while(sl > 0 && p[sl- 1] <= 0x20) /* Strip space on tail */
			sl--;

		if(sl == 0) { /* space only ... */
			printf("Empty line !!!\n");
			continue;
		} else {
			p[sl] = '\0';
		}

		if(*p == '#' || *p == '\n') /* Comment or new line */
			continue;

		int argc = 0;
		char **argv = parse_csv_line(p, &argc, " \n");

		if(argc < 4) {
			printf("Illegal line !!!\n");
			continue;
		}
#if 0
		printf("argc = %d\n", argc);
		for(int i=0;i<argc;i++) {
			printf("argv[%d] : %s\n", i, argv[i]);
		}
#endif
		if(argv[0][0] != 'A') {
			printf("Illegal line : %s\n", p);
			continue;
		}

		uint8_t id = atou8(&argv[0][1]);
#if 1
		//if(id > 0 && id <= NUM_PETAL) {
		if(id > 0 && id <= (NUM_PETAL + NUM_EXTRA_PETAL)) {
			Petal pl;
			pl.pitch = atof(argv[1]);
			pl.roll = atof(argv[2]);
			pl.timestamp = atof(argv[3]);

			int a = (int)roundf(pl.timestamp * 100) % 5;
			if(a == 0) {
				s_petals[id - 1].push_back(pl);
			}
		}
#else
		if(id == 1) {
			Petal pl;
			pl.pitch = atof(argv[1]);
			pl.roll = atof(argv[2]);
			pl.timestamp = atof(argv[3]);

			int a = (int)roundf(pl.timestamp * 100) % 5;
			if(a == 0) {
				for(int i=0;i<NUM_PETAL;i++) // All petals from ID 1
					s_petals[i].push_back(pl);
			}			
		}
#endif
	}

	fclose(fp);

	return true;
}

static bool emergency_wait(std::chrono::microseconds t) 
{
	std::mutex mtx;
	std::unique_lock<std::mutex> lock(mtx);

	auto end = steady_clock::now() + t;

	auto res = s_emergency.wait_until(lock, end);
	if(res != std::cv_status::timeout) {
		return true; // Emergency Stop !!!
	}

	return false;
}

static bool is_emergency_stopped()
{
	if(read_gpio(EMERGENCY_GPIO) == 0) { // Emergency Stopped !!!
		printf("Emergency Stopped !!!\n");
		return true;
	}

	return false;
}

#ifdef __cplusplus
extern "C" {
#endif

extern void tty_raw_mode(void);
extern char *read_line();

#ifdef __cplusplus
}
#endif

static void play_file_cmd(const char *filename, int32_t replayCount)
{
	printf("\nPlay %s\n", filename);
	if(parse_flower_csv(filename) == false)
		return;
#if 1
	if(is_emergency_stopped())
		return;
#endif
#if 0
	for(uint8_t i=0;i<NUM_PETAL;i++) {
		printf("Petal[%d] : \n", i);
		for_each(s_petals[i].begin(), s_petals[i].end(), [&](Petal const & pl)
		{
			if(i==0)
			printf("\t%.2f, %.2f, %.2f\n", pl.pitch, pl.roll, pl.timestamp);
		});
	}
#endif
	vector<Petal>::iterator ppls[NUM_PETAL + NUM_EXTRA_PETAL];
	for(uint8_t i=0;i<NUM_PETAL + NUM_EXTRA_PETAL;i++) {
		ppls[i] = s_petals[i].begin();
	}

	uint32_t d_pitch[NUM_PETAL] = {0}; // delta movement between two positions
	uint32_t d_roll[NUM_PETAL] = {0}; // delta movement between two positions
	bool bFinished = false;
	bool bPause = false;
	const long long _interval = 50; // 50 ms

	// Set terminal in raw mode
	struct termios orig_attr;
	tcgetattr(0, &orig_attr);
	tty_raw_mode();

	while(!bExit && !bFinished) {
		steady_clock::time_point t1(steady_clock::now()); // T1

		if(!bPause) {
			//for(uint8_t i=0;i<NUM_PETAL;i++) {
			for(int8_t i=NUM_PETAL-1;i>=0;i--) {
/*
	0 : M8010L x 25
	1 : RMD X6V3 x 25
*/
				float p0 = ppls[i]->pitch;
				s_socketCan[0].Device(i)->WritePosition((int32_t)(p0 * 100), d_pitch[i] & 0xffff); // 0.01 degree
				//if(i == 0)
				//	printf("s_socketCan[0].Device(%d) -> angle(%f) deg, speed(%u) dps\n", i, p0, d_pitch[i]);
				
				float p1 = ppls[i]->roll;
				s_socketCan[1].Device(i)->WritePosition((int32_t)(p1 * 100), d_roll[i] & 0xffff); // 0.01 degree
				//if(i == 0)
				//	printf("s_socketCan[1].Device(%d) -> angle(%f) deg, speed(%u) dps\n", i, p1, d_roll[i]);

				++ppls[i];

				if(ppls[i] == s_petals[i].end()) {
					d_pitch[i] = 0;
					d_roll[i] = 0;
					continue;
				}

				d_pitch[i] = abs((int32_t)((ppls[i]->pitch - p0) * (1000 / _interval))); // dps
				d_roll[i] = abs((int32_t)((ppls[i]->roll - p1)  * (1000 / _interval))); // dps

				//std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}

			for(uint8_t i=0;i<NUM_EXTRA_PETAL;i++) {
				uint8_t ei = i + NUM_PETAL;
				if(ppls[ei] == s_petals[ei].end())
					continue;

				servolcm::pwm_t r;
				r.channel = i;
				r.angle = ppls[ei]->pitch;

				s_lcm.publish("PWM", &r);
				++ppls[ei];
			}

			printf("\rpercentage %ld%%", distance(s_petals[0].begin(), ppls[0]) * 100 / s_petals[0].size());
			fflush(stdout);
#if 0
			auto dt_us = duration_cast<microseconds>(steady_clock::now() - t1).count();
			printf("dt_us = %ld\n", dt_us);
#endif
			for(uint8_t i=0;i<NUM_PETAL;i++) {
				if(ppls[i] == s_petals[i].end()) {
					if(replayCount > 0) {
						for(uint8_t j=0;j<NUM_PETAL;j++) {
							ppls[j] = s_petals[j].begin();
						}
						std::this_thread::sleep_for(std::chrono::milliseconds(100));
						replayCount--;
						printf("\nReplay %d ...\n", replayCount);
					} else {
						bFinished = true;
					}
					break; // If any petal is end then leave ...
				}
			}
		}

		if(select_stdin(0) > 0) { // No timeout
			char ch;
			::read(0, &ch, 1); // Read one character in raw mode.
			if(ch == 'q' || ch == 'Q') {
				printf("\nStopped !!!\n");
				break;
			}
			else if(ch == 'p' || ch == 'P') {
				bPause = !bPause;
				if(bPause)
					printf("\nPaused !!!\nPress any key to continue ...\n");
			} else {
				if(bPause)
					bPause = false;
			}
		}

		auto dt_us = duration_cast<microseconds>(steady_clock::now() - t1).count();		
		if(emergency_wait(std::chrono::microseconds(_interval * 1000 - dt_us)))
			break; // Emergency Stop !!!
	}

	tcsetattr(0, TCSANOW, &orig_attr);

	if(bFinished)
		printf("\nFinished !!!\n");
}

static void can_cmd(vector<string> & tokens)
{
	// ID from 1 to N, index from 0 to N-1
	uint8_t busId = stoi(tokens[1]) & 0xff;

	if(busId == 0 || busId > NUM_SOCKETCAN) {
		printf("CAN bus ID out of range !!!\n");
		return;
	}
	uint8_t busIndex = busId - 1;

	uint8_t devId = stoi(tokens[2]) & 0xff;
	if(devId == 0 || devId > s_socketCan[busIndex].NumberDevices()) {
		printf("CAN dev ID out of range !!!\n");
		return;
	}
	uint8_t motorIndex = devId - 1; // Motor index from 0 to maxNumMotors

	if(tokens.size() >= 4) {
		if(tokens[3] == "status") {
			s_socketCan[busIndex].Device(motorIndex)->ReadStatus();
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			printf("CAN%d %s[%d] status\n",
				busIndex, 
				s_socketCan[busIndex].Device(motorIndex)->Name(), 
				motorIndex);
			s_socketCan[busIndex].Device(motorIndex)->PrintStatus();
		} else if(tokens[3] == "reset") {
			s_socketCan[busIndex].Device(motorIndex)->Reset();
			printf("CAN%d %s[%d] reset\n",
				busIndex, 
				s_socketCan[busIndex].Device(motorIndex)->Name(), 
				motorIndex);
		} else if(tokens[3] == "pos") {
			if(tokens.size() <= 4) {
				s_socketCan[busIndex].Device(motorIndex)->ReadPosition();
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				printf("CAN%d %s[%d] position = %f (0.01 degree)\n", 
					busIndex, 
					s_socketCan[busIndex].Device(motorIndex)->Name(), 
					motorIndex, 
					s_socketCan[busIndex].Device(motorIndex)->MultiTurnAngle());
			} else {
				int32_t pos = stoi(tokens[4]);
				if(tokens.size() >= 6) {
					uint16_t speed = stoi(tokens[5]);
					s_socketCan[busIndex].Device(motorIndex)->WritePosition(pos, speed);
				} else
					s_socketCan[busIndex].Device(motorIndex)->WritePosition(pos, 200);
				printf("CAN%d %s[%d] go to position %d (0.01 degree)\n", 
					busIndex, 
					s_socketCan[busIndex].Device(motorIndex)->Name(), 
					motorIndex, 
					pos);
			}
		} else if(tokens[3] == "origin") {
			printf("CAN%d %s[%d] go to zero position whit speed 10 dps\n",
				busIndex, 
				s_socketCan[busIndex].Device(motorIndex)->Name(), 
				motorIndex);
			s_socketCan[busIndex].Device(motorIndex)->WritePosition(0, 10); // Go to zero position whit speed 10 dps
		} else if(tokens[3] == "zero") {
			printf("CAN%d %s[%d] set current position as zero position\n",
				busIndex, 
				s_socketCan[busIndex].Device(motorIndex)->Name(), 
				motorIndex);				
			s_socketCan[busIndex].Device(motorIndex)->WriteCurrentPositionAsZero();
		}
	}
}

#define MAX_NUM_PWM_SERVO 16
#define MAX_SERVO_ANGLE 360

static void pwm_cmd(vector<string> & tokens)
{
	static uint16_t zero_bias[MAX_NUM_PWM_SERVO] = {0};

	if(tokens[1] == "pos") {
		uint8_t channel = stoi(tokens[2]) & 0xff;
		if(channel >= MAX_NUM_PWM_SERVO) {
			printf("PWM%u out of range !!!\n", channel);
			return;
		}

		int16_t angle = stoi(tokens[3]);
		if(angle > MAX_SERVO_ANGLE)
			angle = MAX_SERVO_ANGLE;

		printf("PWM%u set angle %u degree\n", channel, angle);

		servolcm::pwm_t r;
		r.channel = channel;
		r.angle = angle;

		s_lcm.publish("PWM", &r);
	} else if(tokens[1] == "zero_bias") {
		uint8_t channel = stoi(tokens[2]) & 0xff;
		if(channel >= MAX_NUM_PWM_SERVO) {
			printf("PWM%u out of range !!!\n", channel);
			return;
		}

		int16_t bias = stoi(tokens[3]);
		if(bias > MAX_SERVO_ANGLE / 2)
			bias = MAX_SERVO_ANGLE / 2;
		
		zero_bias[channel] = bias;
	}
}

static void status_cmd()
{
#ifndef STATUS_THREAD
	for(uint8_t i=0;i<NUM_SOCKETCAN;i++) {
		for(uint8_t j=0;j<s_socketCan[i].NumberDevices();j++) {
			s_socketCan[i].Device(j)->ReadStatus();
			s_socketCan[i].Device(j)->ReadPosition();
		}
	}
#endif
	printf("Read status now, please wait ...\n");

	std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	for(uint8_t i=0;i<NUM_SOCKETCAN;i++) {
		printf("=========================================================================================\n");
		printf("         CAN %2.2u      |  Current  |  Multi Turn Angle  \n", i);
		printf("=========================================================================================\n");      
		for(uint8_t j=0;j<s_socketCan[i].NumberDevices();j++) {
			if(s_socketCan[i].Device(j)->LastRxDuration() > CanNode::timeout)
				printf("\033[0;31m"); /* Red */
			else
				printf("\033[0;32m"); /* Green */
			printf("Device[%2.2u] %12.12s |   %4.2f    |   %8.2lf\n", j, 
				(s_socketCan[i].Device(j)->LastRxDuration() > CanNode::timeout) ? "timeout !!!" : "online ...",
				s_socketCan[i].Device(j)->Current(),
				s_socketCan[i].Device(j)->MultiTurnAngle());
		}
		printf("\033[0m"); /* Default color */
	}
#if 0
	for(uint8_t i=0;i<NUM_SOCKETCAN;i++) {
		printf("=========================================================================================\n");
		printf("CAN %2.2u - Load %d%%\n", i, s_socketCan[i].BusLoad());
		printf("=========================================================================================\n");
	}
#endif
}

static void reset_cmd()
{
	// 展開外圈舉昇馬達 M8010L 至 90度角
	// 水平外圈自旋馬達 X6V3 至 360度角
	// 展開內圈圈舉昇馬達 M8010L 至 60度角
	// 水平內圈自旋馬達 X6V3 至 360度角 

	printf("Reset socket CAN & reinitial all devices ...\n");

	for(int8_t j=s_socketCan[0].NumberDevices()-1;j>=0;j--) {
		CanMotorInitialize<M8010L>(s_socketCan[0].Device(j));
	}

	for(int8_t j=s_socketCan[1].NumberDevices()-1;j>=0;j--) {
		CanMotorInitialize<RMDx6v3>(s_socketCan[1].Device(j));
	}
}

static bool motors_goto(uint8_t can_id, double angle, uint16_t max_speed, uint8_t dev_begin, uint8_t dev_end)
{
	angle = (angle / 100) * 100;

	// Set terminal in raw mode
	struct termios orig_attr;
	tcgetattr(0, &orig_attr);
	tty_raw_mode();

	int loopCount = 0;
	bool bAllDone = false;
	while(!bAllDone) {
		bAllDone = true;
		for(int dev_id=dev_begin;dev_id>=dev_end;dev_id--) {
			double a = round(s_socketCan[can_id].Device(dev_id)->MultiTurnAngle());
			a *= 100;
			if(abs(angle - a) < 500) {
#if 1
				printf("CAN%u %s[%d] go to %lf degree\n", can_id, s_socketCan[can_id].Device(dev_id)->Name(), dev_id, angle);
#endif
				s_socketCan[can_id].Device(dev_id)->WritePosition(angle, max_speed);
				continue;
			}
#if 0
			printf("Warning !!! Warning !!! Warning !!! \n");
			printf("MOTORS 2,5,8,11,14 NOT INSTALLED !!!\n");

			if(dev_id == 2 || dev_id == 5 || dev_id == 8 || dev_id == 11 || dev_id == 14)
				continue;
#endif
			bAllDone = false;

			double step = angle > a ? 500 : -500; // +/- 1 degree
#if 1
			printf("CAN%u %s[%d] go to %lf degree with step %lf, max speed %u\n", can_id, s_socketCan[can_id].Device(dev_id)->Name(), dev_id, a + step, step, max_speed);
#endif
			s_socketCan[can_id].Device(dev_id)->WritePosition(a + step, max_speed);
		}

		if(loopCount++ > 180) {
			printf("Timeout !!!\n");
			bAllDone = false;
			break;
		}

		if(select_stdin(0) > 0) { // No timeout
			char ch;
			::read(0, &ch, 1); // Read one character in raw mode.
			if(ch == 'q' || ch == 'Q'){
				printf("Stopped !!!\n");
				bAllDone = false;
				break;
			}
		}

		//std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if(emergency_wait(std::chrono::milliseconds(100))) { // 100ms
			bAllDone = false;
			break;
		}

	}

	tcsetattr(0, TCSANOW, &orig_attr);

	return bAllDone;
}

#define NUM_OUTTER_PETAL 15
#define NUM_INNER_PETAL 10

static inline bool outter_motors_goto(uint8_t can_id, double angle, uint16_t max_speed)
{
	return motors_goto(can_id, angle, max_speed, NUM_OUTTER_PETAL-1, 0); // 14 ~ 0
}

static inline bool inner_motors_goto(uint8_t can_id, double angle, uint16_t max_speed)
{
	return motors_goto(can_id, angle, max_speed, NUM_PETAL-1, NUM_OUTTER_PETAL); // 24 ~ 15
}

static bool s_is_home_cmd_success = true;
bool is_home_cmd_success() { return s_is_home_cmd_success; }

static bool home_cmd()
{
	printf("Bring all motors to home position ...\n");

	// CAN0 M8010L[0~14]

	printf("Raising up all outter petals\n");
	if(!outter_motors_goto(0, -9000, 200))
		return false;

	// CAN1 RMDx6v3[0~14]

	printf("Rotate all outter petals\n");
	if(!outter_motors_goto(1, 2400, 200))
		return false;

	// CAN0 M8010L[15~24]

	printf("Raising up all inner petals\n");
	if(!inner_motors_goto(0, -3000, 200))
		return false;

	// CAN1 RMDx6v3[15~24]

	printf("Rotate all inner petals\n");
	if(!inner_motors_goto(1, 2400, 200))
		return false;

	// CAN0 M8010L[15~24]

	printf("Low down all inner petals\n");
	if(!inner_motors_goto(0, 0, 200))
		return false;

	// CAN0 M8010L[0~14]

	printf("Low down all outter petals\n");
	if(!outter_motors_goto(0, 0, 200))
		return false;

	printf("Done ...\n");

	return true;
}

static void shutdown_cmd()
{
	printf("Shutdown all motors ...\n");

	for(uint8_t i=0;i<NUM_SOCKETCAN;i++) {
		for(int8_t j=s_socketCan[0].NumberDevices()-1;j>=0;j--) {
			s_socketCan[i].Device(j)->Shutdown();
		}
	}

	printf("Done ...\n");
}

/*
*
*/

#include <linux/can/error.h>

class MasterServer {
private:
	uint8_t m_scenario;
	typedef enum { e_shutdown, e_reset, e_idle, e_home, e_playing, e_error } State;
	std::atomic<State> m_state;

	std::thread m_actionThread;

public:
	MasterServer() : m_scenario(0), m_state(e_shutdown) {}

	void handleCommand(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
					   const protolcm::command_t *msg)
	{
		printf("  action   = %s\n", msg->action.c_str());
		printf("  index   = %d\n", msg->index);
		printf("  receive time   = %lld\n", (long long) rbuf->recv_utime);
		printf("  timestamp   = %lld\n", (long long) msg->timestamp);

		if(strcmp(msg->action.c_str(), "shutdown") == 0) {
			if(m_state == e_shutdown)
				return;

			std::thread t = ShutdownThread();
			t.detach();
		} else if(strcmp(msg->action.c_str(), "reset") == 0) {
			if(m_state >= e_reset)
				return;

			std::thread t = ResetThread();
			t.detach();
		} else if(strcmp(msg->action.c_str(), "stop") == 0) {
			if(m_state < e_playing)
				return;

			s_emergency.notify_all(); // Not really emergency stop, just use it to stop playing ...

			m_state = e_reset; // Needs home ...
		} else if(strcmp(msg->action.c_str(), "home") == 0) {
			if(m_state < e_reset || m_state >= e_home)
				return;

			std::thread t = HomeThread();
			t.detach();
		} else if(strcmp(msg->action.c_str(), "play") == 0) {
			if(m_state != e_idle)
				return;

			if(m_scenario != 0)
				return;

			char fn[128];
			snprintf(fn, 128, "/usr/local/share/flower_%02d.csv", msg->index);

			if(m_actionThread.joinable())
				m_actionThread.join();

			m_actionThread = PlayThread(fn);
		} else if(strcmp(msg->action.c_str(), "scenario") == 0) {
			m_scenario = msg->index;
		}
	}

	void handleRegion(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
					   const protolcm::region_t *msg)
	{
		printf("  receive time   = %lld\n", (long long) rbuf->recv_utime);
		printf("  timestamp   = %lld\n", (long long) msg->timestamp);
		printf("  human count   = ");
		for(int i=0;i<5;i++) {
			printf("%d ", msg->human_counts[i]);
		}
		printf("\n");

		if(m_state != e_idle)
			return;

		if(m_scenario != 1)
			return;

		uint8_t region = 0;
		uint8_t human_counts = msg->human_counts[0];
		for(int i=1;i<5;i++) {
			if(human_counts < msg->human_counts[i]) {
				human_counts = msg->human_counts[i];
				region = i;
			}
		}

		char fn[128];
		snprintf(fn, 128, "/usr/local/share/flower_%02d_%02d.csv", region, human_counts % 5);

		if(m_actionThread.joinable())
			m_actionThread.join();

		m_actionThread = PlayThread(fn);
	}

	std::thread StatusThread() {
		return std::thread([this] {
			while(!bExit) {
				protolcm::status_t s;
				s.scenario = this->m_scenario;
				
				switch(this->m_state) {
					case e_shutdown: s.state = "shutdown";
						break;
					case e_reset: s.state = "reset";
						break;
					case e_idle: s.state = "idle";
						break;
					case e_home: s.state = "home";
						break;
					case e_playing: s.state = "playing";
						break;
					case e_error: s.state = "error";
						break;
				}

				s.num_warn = 0;
				s.warn_str.resize(0);

				for(int i=0;i<NUM_SOCKETCAN;i++) {
					for(int j=0;j<s_socketCan[i].NumberDevices();j++) {
						CanMotor *cm = s_socketCan[i].Device(j);
						if(cm->Temperature() >= 60) {
							s.num_warn++;
							s.warn_str.resize(s.num_warn);
							std::ostringstream ss;
							ss << "CAN " << i << " Motor " << j+1 << " temperature is " << cm->Temperature() << " degree";
							s.warn_str[s.num_warn-1] = ss.str();
						}

						if(cm->Current() > 1.0f) {
							s.num_warn++;
							s.warn_str.resize(s.num_warn);
							std::ostringstream ss;
							ss << "CAN " << i << " Motor " << j+1 << " current is " << cm->Current() << " amp";
							s.warn_str[s.num_warn-1] = ss.str();
						}
					}
				}

				s.num_err = 0;
				s.err_str.resize(0);

				if(s_socketCan[0].BusErrno() != 0) {
					s.num_err++;
					s.err_str.resize(s.num_err);
					std::ostringstream ss;
					ss << "CAN 0 bus error : " << strerror(s_socketCan[0].BusErrno());
					s.err_str[s.num_err-1] = ss.str();

					this->m_state = e_error;
				}
				if(s_socketCan[1].BusErrno() != 0) {
					s.num_err++;
					s.err_str.resize(s.num_err);
					std::ostringstream ss;
					ss << "CAN 1 bus error : " << strerror(s_socketCan[1].BusErrno());
					s.err_str[s.num_err-1] = ss.str();

					this->m_state = e_error;
				}
				if(s_socketCan[0].Flag() & CAN_ERR_FLAG) {
					s.num_err++;
					s.err_str.resize(s.num_err);
					canid_t flag = s_socketCan[0].Flag();
					std::ostringstream ss;
					ss << "CAN 0 error : ";
					if(flag & CAN_ERR_TX_TIMEOUT)
						ss << "TX timeout (by netdevice driver)";
					if(flag & CAN_ERR_LOSTARB   )
						ss << "lost arbitration";
					if(flag & CAN_ERR_CRTL      )
						ss << "controller problems";
					if(flag & CAN_ERR_PROT      )
						ss << "protocol violations";
					if(flag & CAN_ERR_TRX       )
						ss << "transceiver status";
					if(flag & CAN_ERR_ACK       )
						ss << "received no ACK on transmission";
					if(flag & CAN_ERR_BUSOFF    )
						ss << "bus off";
					if(flag & CAN_ERR_BUSERROR  )
						ss << "bus error (may flood!)";
					if(flag & CAN_ERR_RESTARTED )
						ss << "controller restarted";
					s.err_str[s.num_err-1] = ss.str();

					this->m_state = e_error;
				}
				if(s_socketCan[1].Flag() & CAN_ERR_FLAG) {
					s.num_err++;
					s.err_str.resize(s.num_err);
					canid_t flag = s_socketCan[1].Flag();
					std::ostringstream ss;
					ss << "CAN 1 error : ";
					if(flag & CAN_ERR_TX_TIMEOUT)
						ss << "TX timeout (by netdevice driver)";
					if(flag & CAN_ERR_LOSTARB   )
						ss << "lost arbitration";
					if(flag & CAN_ERR_CRTL      )
						ss << "controller problems";
					if(flag & CAN_ERR_PROT      )
						ss << "protocol violations";
					if(flag & CAN_ERR_TRX       )
						ss << "transceiver status";
					if(flag & CAN_ERR_ACK       )
						ss << "received no ACK on transmission";
					if(flag & CAN_ERR_BUSOFF    )
						ss << "bus off";
					if(flag & CAN_ERR_BUSERROR  )
						ss << "bus error (may flood!)";
					if(flag & CAN_ERR_RESTARTED )
						ss << "controller restarted";
					s.err_str[s.num_err-1] = ss.str();

					this->m_state = e_error;
				}
				if(is_emergency_stopped()) {
					s.num_err++;
					s.err_str.resize(s.num_err);
					s.err_str[s.num_err-1] = "Emergency Stopped !!!";

					this->m_state = e_error;
				}
				if(is_home_cmd_success() == false) {
					s.num_err++;
					s.err_str.resize(s.num_err);
					s.err_str[s.num_err-1] = "Go to home position failed !!!";

					this->m_state = e_error;
				}

				struct timeval tp;
				gettimeofday(&tp, NULL);
				s.timestamp = tp.tv_sec * 1000 + tp.tv_usec / 1000;

				s_lcm.publish("SERVER_STATUS", &s);
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			}
		});
	}

	std::thread PlayThread(const char *filename) {
		m_state = e_playing;
		return std::thread([this] (const char *filename) {
			play_file_cmd(filename, 0);
			if(m_state == e_playing)
				m_state = e_idle;
		}, filename);
	}

	std::thread ShutdownThread() {
		m_state = e_shutdown;
		return std::thread([this] () {
			shutdown_cmd();
		});
	}

	std::thread ResetThread() {
		m_state = e_reset;
		return std::thread([this] () {
			reset_cmd();
			m_state = e_idle;
		});
	}

	std::thread HomeThread() {
		m_state = e_home;
		return std::thread([this] () {
			home_cmd();
			m_state = e_idle;
		});
	}

};

static MasterServer s_server;

/*
*
*/

const char promopt[] = "CAN\\>";
// ID from 1 to N ...
const char help[] = 
"\
Usage: COMMAND [PARAMETERS]\n\
\n\
Management Commands:\n\
  can [bus ID] [dev ID] <reset | status | pos <position speed> | test <position speed> | origin >\n\
  tcp <reconnect | disconnect>\n\
  pwm pos [channel] [angle]\n\
  play [file]\n\
  origin\n\
  zero\n\
  status\n\
  reset\n\
  home\n\
  shutdown\n\
  exit\n\
\n\
";

static int can_main(int argc, char**argv)
{
	std::thread socketCan1Thread(SocketCanThread<M8010L>, std::ref(s_socketCan[0]), 0, 1000000, 
		NUM_DEV_PER_SOCKETCAN); // 25 M8010L motors

	std::thread socketCan2Thread(SocketCanThread<RMDx6v3>, std::ref(s_socketCan[1]), 1, 500000, 
		NUM_DEV_PER_SOCKETCAN); // 25 RMDx6 motors

	while(!bExit) {
		printf(promopt);
		fflush(stdout);
		char *input = read_line();

		if(input[0] < 0x20)
			continue;

		if(input[0] != 0x00) {
			vector<string> tokens;
			char *pch = strtok(input, " "); // Split input commands
			while(pch != NULL) {
				printf("[%lu] '%s'\n", tokens.size(), pch);
				tokens.push_back(pch);
				pch = strtok(NULL, " ");
			}
			printf("\n");

			if(tokens.size() > 0) {
				if(tokens[0] == "exit") {
					bExit = true;
				} else if(tokens[0] == "help") {
					printf(help);					
				} else if(tokens[0] == "play") {
					// If there's CAN bus error then reinitialize all devices
					for(int i=0;i<NUM_SOCKETCAN;i++) { 
						if(s_socketCan[i].Flag() & CAN_ERR_FLAG) {
							reset_cmd();
							break;
						}
					}

					if(tokens.size() == 2)
						play_file_cmd(tokens[1].c_str(), 0);
					else if(tokens.size() >= 3)
						play_file_cmd(tokens[1].c_str(), stoi(tokens[2]));
					else
						printf(help);
				} else if(tokens[0] == "origin") {
					printf("Go to zero position whit speed 10 dps\n");
					for(uint8_t i=0;i<NUM_SOCKETCAN;i++) {
						for(uint8_t j=0;j<s_socketCan[i].NumberDevices();j++) {
							s_socketCan[i].Device(j)->WritePosition(0, 10); // Go to zero position whit speed 10 dps
						}
					}
				} else if(tokens[0] == "can") {
					if(tokens.size() >= 4)
						can_cmd(tokens);
					else
						printf(help);
				} else if(tokens[0] == "pwm") {
					if(tokens.size() >= 4)
						pwm_cmd(tokens);
					else
						printf(help);
				} else if(tokens[0] == "status") {
					status_cmd();
				} else if(tokens[0] == "reset") {
					reset_cmd();
				} else if(tokens[0] == "home") {
					s_is_home_cmd_success = home_cmd();
				} else if(tokens[0] == "shutdown") {
					shutdown_cmd();
				}

				tokens.clear();
			}
		}
	}

	printf("Exit !!! Wait for CAN thread ...\n");

	socketCan1Thread.join();
	socketCan2Thread.join();

	return 0;
}

#if 0
class Handler {
  public:
	~Handler() {}
	void handlePosition(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
					   const servolcm::position_t *msg)
	{
/*
		printf("Received message on channel \"%s\":\n", chan.c_str());
		printf("  position = %d\n", msg->position);
		printf("  speed = %d\n", msg->speed);
*/
		if(msg->bus == 0 || msg->bus > NUM_SOCKETCAN)
			return; 
		if(msg->id == 0 || msg->id > s_socketCan[msg->bus - 1].NumberDevices())
			return;

		CanMotor *cm = s_socketCan[msg->bus - 1].Device(msg->id - 1);

		if(cm == nullptr)
			return;

		cm->WritePosition(msg->position * 100, msg->speed); // 0.01 degree, dps
	}
};
#endif

static void servo_status_publish()
{
	while (!bExit) {
		for(int i=0;i<NUM_SOCKETCAN;i++) {
			for(int j=0;j<s_socketCan[i].NumberDevices();j++) {
				CanMotor *cm = s_socketCan[i].Device(j);
				
				servolcm::status_t r;
				r.bus = i+1;
				r.id = j+1;
				r.active = cm->IsActive();
				r.online = cm->LastRxDuration() <= CanNode::timeout;
				r.encoder_position = cm->EncoderPosition();
				r.multi_turn_angle = cm->MultiTurnAngle();
				r.voltage = cm->Voltage();
				r.current = cm->Current();
				r.temperature = cm->Temperature();

				s_lcm.publish("SERVO_STATUS", &r);
				std::this_thread::sleep_for(std::chrono::milliseconds(20));
			}
		}
		//std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
}

/*
* sudo ifconfig eno1 multicast
* sudo route add -net 239.255.76.67 netmask 255.255.255.255 dev eno1
* route -n
*/

static int lcm_main(int argc, char**argv)
{
	if(!s_lcm.good())
		return 1;

	std::thread servo_status_thread(servo_status_publish);
#if 0
	Handler handlerObject;
	s_lcm.subscribe("POSITION", &Handler::handlePosition, &handlerObject);
#endif
	
	s_lcm.subscribe("SERVER_COMMAND", &MasterServer::handleCommand, &s_server);

	std::thread server_status_thread = s_server.StatusThread();

	while(!bExit) {
		s_lcm.handleTimeout(100);
	}

	server_status_thread.join();
	servo_status_thread.join();

	return 0;
}

/*

echo 26 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio26/direction
echo rising >/sys/class/gpio/gpio26/edge

echo 4 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio4/direction
echo falling >/sys/class/gpio/gpio4/edge

*/

/*
# config.txt

# Make 17 to 21 inputs
      gpio=17-21=ip

# Change the pull on (input) pins 18 and 20
      gpio=18,20=pu
*/

static int emergency_main(int gpio)
{
	printf("Emergency GPIO is %d\n", EMERGENCY_GPIO);

	char buf[64];
	snprintf(buf, 64, "/sys/class/gpio/gpio%d/value", gpio);
	int fd = open(buf, O_RDONLY);
	if(fd < 0)
		return -1;

	struct pollfd pfd;
	pfd.fd = fd;
	pfd.events = POLLPRI;

	lseek(fd, 0, SEEK_SET);

	while(!bExit) {
		int retval = poll(&pfd, 1, -1);
		if (pfd.revents & POLLPRI) {

			lseek(fd, 0, SEEK_SET);

			char valstr[3];
			retval = read(fd, valstr, 3);

			if(retval > 0) {
				if(atoi(valstr) == 1) {
					printf("Emergency Stopped !!!\n");
					shutdown_cmd();
					s_emergency.notify_all();
				}
			}
		}
	}

	return 0;
}

int main(int argc, char**argv)
{
	if(signal(SIGINT, sig_handler) == SIG_ERR)
		printf("\ncan't catch SIGINT\n");

	int r, v = -20;
	r = nice(v);
	if(errno == -1 && errno != 0)
		perror("nice");
	else
		printf("nice value is now %d\n", v);

	std::thread lcm_thread(lcm_main, argc, argv);
	std::thread gpio_thread(emergency_main, EMERGENCY_GPIO);
	
	can_main(argc, argv);

	gpio_thread.detach();
	lcm_thread.join();

	return 0;
}
