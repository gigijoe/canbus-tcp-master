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

//#include "modbus/modbus.h"
#include "csv_parser.h"
//#include "color.h"

#include "socket_can.hpp"
#include "can_node.hpp"

#include <lcm/lcm-cpp.hpp>
#include "servolcm/position_t.hpp"
#include "servolcm/status_t.hpp"
#include "servolcm/pwm_t.hpp"

std::condition_variable s_emergency;

static lcm::LCM s_lcm("udpm://239.255.76.67:7667?ttl=1");

using std::chrono::steady_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::seconds;

//using namespace cv;
using namespace std;

static bool bShutdown = false;

void sig_handler(int signo)
{
	if(signo == SIGINT) {
		printf("SIGINT\n");
		bShutdown = true;
	}
}

#include "can_node.hpp"
#include <future>

#define STATUS_THREAD 

template <class T>
void SocketCanThread(SocketCan & socketCan, uint16_t port, uint32_t bitrate, uint8_t numMotors)
{
	socketCan.Connect(port, bitrate);

	for(uint8_t i=0;i<numMotors;i++) {
		socketCan.AddDevice(new T(socketCan, i+1)); // ID range from 1 to 32
	}

	while(!bShutdown) {
		//for(uint8_t i=0;i<socketCan.NumberDevices();i++) {
		for(int8_t i=socketCan.NumberDevices()-1;i>=0;i--) {
			socketCan.Device(i)->Reset();
			socketCan.Device(i)->WriteBrake(false);
			socketCan.Device(i)->WriteTorque(100); // 1A
			if(typeid(T) == typeid(RMDx6)) {
// X6 default 
/*
Current Kp = 100 / Ki = 100
Speed Kp = 50 / Ki = 40
Position Kp = 50 / Ki = 50
*/
				socketCan.Device(i)->WritePID(50, 50, 10, 8, 100, 100); // Kp, Ki from 0 ~ 255
			} else if(typeid(T) == typeid(RMDx6v3)) {	
				socketCan.Device(i)->WriteAcceleration(6000); // 100 ~ 60000 dps/s
				socketCan.Device(i)->WriteDeceleration(3000); // 100 ~ 60000 dps/s
				socketCan.Device(i)->WritePosKpKi(100, 5); // Kp, Ki from 0 ~ 255
			} else if(typeid(T) == typeid(M8010L)) {
				socketCan.Device(i)->ReverseDirection();
				socketCan.Device(i)->WriteAcceleration(65535); // Disable trapezoidal acceleration pluse
				socketCan.Device(i)->WritePosKpKi(300, 0); // Kp from 60 ~ 30000
				socketCan.Device(i)->WriteHeartBeatInterval(0); // Heart beat interval in ms. 0 disabled.
				//socketCan.Device(i)->PositionLimitation(0, -10000); // 0 to -100 degree
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}

#ifdef STATUS_THREAD // Status thread causes DNET400
		std::promise<void> exitSignal;
		std::future<void> futureObj = exitSignal.get_future();
		
		std::thread t([&socketCan](std::future<void> futureObj) -> void { // Capture local variable 'socketCan' by reference
			const long long _interval = 10; // ms
			while(!bShutdown) {
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
		while(!bShutdown) {
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

#define TEST_GAP 200 // 2 degree

static int test_cmd(SocketCan & socketCan, int index, int32_t pos, uint16_t speed)
{
	printf("Start test ... Press any key to stop !!!\n");
	socketCan.Device(index)->ReadPosition();
	std::this_thread::sleep_for(std::chrono::milliseconds(33));

	int32_t p = socketCan.Device(index)->EncoderPosition(); // 0.01 degree
	int32_t interval = TEST_GAP; // 1 Degree interval 

	p = (p / interval) * interval; // Make sure position aligment to interval
	while(!bShutdown) {
		if(interval > 0) {
			if((p + interval) <= pos) {
				p += interval;
				socketCan.Device(index)->WritePosition(p, speed);
			} else 
				interval = (0 - TEST_GAP);
		} else if(interval < 0) {
			if((p + interval) >= 0) {
				p += interval;
				socketCan.Device(index)->WritePosition(p, speed);
			} else
				interval = TEST_GAP;
		}

		if(select_stdin(100000) > 0) { // 100ms timeout
			break;
		}

		//socketCan.Device(index)->ReadStatus();
	}
	printf("Done ...\n");

	return 0;
}

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
		printf("parse %s fail : %s\n", filename, strerror(errno));
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

void play_file_cmd(const char *filename, int32_t replayCount)
{
	if(parse_flower_csv(filename) == false)
		return;
	
	if(read_gpio(26) == 1) { // Emergency Stopped !!!
		printf("Emergency Stopped !!!\n");
		return;
	}
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
	const long long _interval = 50; // 50 ms
	while(!bShutdown && !bFinished) {
		steady_clock::time_point t1(steady_clock::now()); // T1

		static int32_t pos = 0;
		static uint16_t speed = 0;
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
#if 1
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
					printf("Replay %d ...\n", replayCount);
				} else {
					bFinished = true;
				}
				break; // If any petal is end then leave ...
			}
		}

		if(select_stdin(0) > 0) { // No timeout
			char ch;
			::read(0, &ch, 1); // Read one character in raw mode.
			if(ch == 'q' || ch == 'Q')
				break;
		}

		std::mutex mtx;
		std::unique_lock<std::mutex> lock(mtx);

		dt_us = duration_cast<microseconds>(steady_clock::now() - t1).count();
		//printf("dt_us = %ld\n", dt_us);
		
		auto end = steady_clock::now() + std::chrono::microseconds(_interval * 1000 - dt_us);
		auto res = s_emergency.wait_until(lock, end);
		if(res != std::cv_status::timeout) {
			break; // Emergency Stop !!!
		}
/*
		if(dt_us < _interval * 1000) {
			//std::this_thread::sleep_for(std::chrono::microseconds(_interval * 1000 - dt_us));
			if(select_stdin(_interval * 1000 - dt_us) > 0) { // 100ms timeout
				char ch;
				::read(0, &ch, 1); // Read one character in raw mode.
				if(ch == 'q' || ch == 'Q')
					break;
			}
		}
*/
	}
	printf("Finished !!!\n");
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
		} else if(tokens[3] == "reset") {
			s_socketCan[busIndex].Device(motorIndex)->Reset();
		} else if(tokens[3] == "pos") {
			if(tokens.size() <= 4) {
				s_socketCan[busIndex].Device(motorIndex)->ReadPosition();
			} else {
				int32_t pos = stoi(tokens[4]);
				if(tokens.size() >= 6) {
					uint16_t speed = stoi(tokens[5]);
					s_socketCan[busIndex].Device(motorIndex)->WritePosition(pos, speed);
				} else
					s_socketCan[busIndex].Device(motorIndex)->WritePosition(pos, 360);
			}
		} else if(tokens[3] == "pid") {
			s_socketCan[busIndex].Device(motorIndex)->ReadPID();
		} else if(tokens[3] == "origin") {
			s_socketCan[busIndex].Device(motorIndex)->WritePosition(0, 10); // Go to zero position whit speed 10 dps
		} else if(tokens[3] == "zero") {
			s_socketCan[busIndex].Device(motorIndex)->WriteCurrentPositionAsZero();
		} else if(tokens[3] == "test") {
			if(tokens.size() >= 5) {
				int32_t pos = stoi(tokens[4]);
				uint16_t speed = 360; // dps
				if(tokens.size() >= 6)
					speed = stoi(tokens[5]);
				test_cmd(s_socketCan[busIndex], motorIndex, pos, speed);
			}
		}
	}
}

static void pwm_cmd(vector<string> & tokens)
{
	uint8_t channel = stoi(tokens[1]) & 0xff;
	uint8_t angle = stoi(tokens[2]) & 0xff;
	if(angle > 180)
		angle = 180;

	servolcm::pwm_t r;
	r.channel = channel;
	r.angle = angle;

	s_lcm.publish("PWM", &r);
}

void status_cmd()
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
}

void reset_cmd()
{
	// 展開外圈舉昇馬達 M8010L 至 90度角
	// 水平外圈自旋馬達 X6V3 至 360度角
	// 展開內圈圈舉昇馬達 M8010L 至 60度角
	// 水平內圈自旋馬達 X6V3 至 360度角 
}

#ifdef __cplusplus
extern "C" {
#endif

extern char *read_line();

#ifdef __cplusplus
}
#endif

const char promopt[] = "CAN\\>";
// ID from 1 to N ...
const char help[] = 
"\
Usage: COMMAND [PARAMETERS]\n\
\n\
Management Commands:\n\
  can [bus ID] [dev ID] <reset | status | pos <position speed> | test <position speed> | origin >\n\
  tcp <reconnect | disconnect>\n\
  pwm [channel] [angle]\n\
  play [file]\n\
  origin\n\
  status\n\
  exit\n\
\n\
";

int can_main(int argc, char**argv)
{
	std::thread socketCan1Thread(SocketCanThread<M8010L>, std::ref(s_socketCan[0]), 0, 1000000, 
		NUM_DEV_PER_SOCKETCAN); // 25 M8010L motors

	std::thread socketCan2Thread(SocketCanThread<RMDx6v3>, std::ref(s_socketCan[1]), 1, 1000000, 
		NUM_DEV_PER_SOCKETCAN); // 25 RMDx6 motors

	while(!bShutdown) {
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
					bShutdown = true;
				} else if(tokens[0] == "help") {
					printf(help);					
				} else if(tokens[0] == "play") {
					if(tokens.size() == 2)
						play_file_cmd(tokens[1].c_str(), 0);
					else if(tokens.size() >= 3)
						play_file_cmd(tokens[1].c_str(), stoi(tokens[2]));
					else
						printf(help);
				} else if(tokens[0] == "origin") {
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
					if(tokens.size() >= 3)
						pwm_cmd(tokens);
					else
						printf(help);
				} else if(tokens[0] == "status") {
					status_cmd();
				} else if(tokens[0] == "reset") {
					reset_cmd();
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

void status_publish()
{
	while (!bShutdown) {
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

				s_lcm.publish("STATUS", &r);
				std::this_thread::sleep_for(std::chrono::milliseconds(20));
			}
		}
		//std::this_thread::sleep_for(std::chrono::milliseconds(500));

		for(uint8_t i=0;i<NUM_SOCKETCAN;i++) {
			printf("CAN %2.2u - Load %d%%\n", i, s_socketCan[i].BusLoad());
		}
	}
}

int lcm_main(int argc, char**argv)
{
	if(!s_lcm.good())
		return 1;

	std::thread status_thread(status_publish);

	Handler handlerObject;
	s_lcm.subscribe("POSITION", &Handler::handlePosition, &handlerObject);

	while(!bShutdown) {
		s_lcm.handleTimeout(100);
	}

	status_thread.join();

	return 0;
}

/*

echo 26 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio26/direction
echo rising >/sys/class/gpio/gpio26/edge

*/

int gpio_main(int gpio)
{
    char buf[64];
	snprintf(buf, 64, "/sys/class/gpio/gpio%d/value", gpio);
	int fd = open(buf, O_RDONLY);
	if(fd < 0)
		return -1;

    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLPRI;

    lseek(fd, 0, SEEK_SET);

    while(!bShutdown) {
        int retval = poll(&pfd, 1, -1);
        if (pfd.revents & POLLPRI) {

            lseek(fd, 0, SEEK_SET);

            char valstr[3];
            retval = read(fd, valstr, 3);

            if(retval > 0) {
            	//cout << valstr << endl;
            	if(atoi(valstr) == 1) {
            		printf("Emergency Stopped !!!\n");
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
	std::thread gpio_thread(gpio_main, 26);
    
	can_main(argc, argv);

	gpio_thread.detach();
	lcm_thread.join();

	return 0;
}
