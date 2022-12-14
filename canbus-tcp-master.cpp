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

#include "tcp_can.hpp"
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
void TcpCanThread(TcpCan & tcpCan, const char *addr, uint16_t port, uint8_t numMotors)
{
	for(uint8_t i=0;i<numMotors;i++) {
		tcpCan.AddDevice(new T(tcpCan, i+1)); // ID range from 1 to 32
	}

	while(!bShutdown) {
		int r = tcpCan.Connect(addr, port);

		if(r < 0) {
			std::this_thread::sleep_for(std::chrono::seconds(1));
			continue;
		}

		tcpCan.EnableWriteCache();

		//for(uint8_t i=0;i<tcpCan.NumberDevices();i++) {
		for(int8_t i=tcpCan.NumberDevices()-1;i>=0;i--) {	
			tcpCan.Device(i)->Reset();
			tcpCan.Device(i)->WriteBrake(false);
			tcpCan.Device(i)->WriteMaximumCurrent(100); // 1A
			if(typeid(T) == typeid(RMDx6)) {
// X6 default 
/*
Current Kp = 100 / Ki = 100
Speed Kp = 50 / Ki = 40
Position Kp = 50 / Ki = 50
*/
				tcpCan.Device(i)->WritePID(50, 50, 10, 8, 100, 100); // Kp, Ki from 0 ~ 255
			} else if(typeid(T) == typeid(RMDx6v3)) {	
				tcpCan.Device(i)->WriteAcceleration(6000); // 100 ~ 60000 dps/s
				tcpCan.Device(i)->WriteDeceleration(3000); // 100 ~ 60000 dps/s
				tcpCan.Device(i)->WritePosKpKi(100, 5); // Kp, Ki from 0 ~ 255
			} else if(typeid(T) == typeid(M8010L)) {
				tcpCan.Device(i)->ReverseDirection();
				tcpCan.Device(i)->WriteAcceleration(65535); // Disable trapezoidal acceleration pluse
				tcpCan.Device(i)->WritePosKpKi(300, 0); // Kp from 60 ~ 30000
				tcpCan.Device(i)->WriteHeartBeatInterval(0); // Heart beat interval in ms. 0 disabled.
			}
		}

		tcpCan.FlushWriteCache();
		tcpCan.DisableWriteCache();

#ifdef STATUS_THREAD // Status thread causes DNET400
		std::promise<void> exitSignal;
		std::future<void> futureObj = exitSignal.get_future();
		
		std::thread t([&tcpCan](std::future<void> futureObj) -> void { // Capture local variable 'tcpCan' by reference
			const long long _interval = 500; // ms
			while(!bShutdown) {
				tcpCan.EnableWriteCache();

				//for(uint8_t i=0;i<tcpCan.NumberDevices();i++) {
				for(int8_t i=tcpCan.NumberDevices()-1;i>=0;i--) {
					if(tcpCan.Device(i) == nullptr)
						continue;
					tcpCan.Device(i)->ReadStatus(); 
				}

				tcpCan.FlushWriteCache();
				tcpCan.DisableWriteCache();

				if(futureObj.wait_for(std::chrono::milliseconds(_interval)) == std::future_status::ready)
					break;

				tcpCan.EnableWriteCache();

				//for(uint8_t i=0;i<tcpCan.NumberDevices();i++) {
				for(int8_t i=tcpCan.NumberDevices()-1;i>=0;i--) {
					if(tcpCan.Device(i) == nullptr)
						continue;
					tcpCan.Device(i)->ReadPosition();
				}
				
				tcpCan.FlushWriteCache();
				tcpCan.DisableWriteCache();

				if(futureObj.wait_for(std::chrono::milliseconds(_interval)) == std::future_status::ready)
					break;
			}
		}, std::move(futureObj));
#endif
		while(!bShutdown) {
			vector<TcpCanFrame> vf;
			int r = tcpCan.Read(vf, 100); // 100ms timeout
			if(r > 0) {
				for(auto it = vf.begin();it != vf.end();++it) {
					uint8_t id = it->id & 0x1f; // ID 1~32
					if(id == 0)
						continue;
					uint8_t i = id - 1;
					if(tcpCan.Device(i) == nullptr)
						continue;
					
					tcpCan.Device(i)->OnRx(*it);
				}
				vf.clear();
			} else if(r < 0) { // Connection closed !!! 
				printf("Connection closed !!!\n");
				std::this_thread::sleep_for(std::chrono::seconds(1));
				break;
			} else { // Read timeout ...
				continue; 
			}

			//for(uint8_t i=0;i<tcpCan.NumberDevices();i++) {
			for(int8_t i=tcpCan.NumberDevices()-1;i>=0;i--) {	
				if(tcpCan.Device(i) == nullptr)
					continue;
				TcpCanFrame f;

				while(tcpCan.Device(i)->Read(f) == 0) {} // Handle all packets
			}
		}

#ifdef STATUS_THREAD
		exitSignal.set_value();
		t.join();
#endif
	}

	tcpCan.Disconnect();

	for(uint8_t i=0;i<tcpCan.NumberDevices();i++) {
		delete tcpCan.Device(i); // ID range from 1 to 32
	}

	tcpCan.RemoveAllDevices();
}

/*
*
*/

#undef DNET400
#ifdef DNET400
#define NUM_DEV_PER_TCPCAN 13
#define NUM_TCPCAN 4
#endif

#define ZQWL_CANET
#ifdef ZQWL_CANET
#define NUM_DEV_PER_TCPCAN 25
#define NUM_TCPCAN 2
#endif

static TcpCan s_tcpCan[NUM_TCPCAN];

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

#define TEST_GAP 200 // 2 degree

static int test_cmd(TcpCan & tcpCan, int index, int32_t pos, uint16_t speed)
{
	printf("Start test ... Press any key to stop !!!\n");
	tcpCan.Device(index)->ReadPosition();
	std::this_thread::sleep_for(std::chrono::milliseconds(33));

	int32_t p = tcpCan.Device(index)->EncoderPosition(); // 0.01 degree
	int32_t interval = TEST_GAP; // 1 Degree interval 

	p = (p / interval) * interval; // Make sure position aligment to interval
	while(!bShutdown) {
		if(interval > 0) {
			if((p + interval) <= pos) {
				p += interval;
				tcpCan.Device(index)->WritePosition(p, speed);
			} else 
				interval = (0 - TEST_GAP);
		} else if(interval < 0) {
			if((p + interval) >= 0) {
				p += interval;
				tcpCan.Device(index)->WritePosition(p, speed);
			} else
				interval = TEST_GAP;
		}

		if(select_stdin(100000) > 0) { // 100ms timeout
			break;
		}

		//tcpCan.Device(index)->ReadStatus();
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
static vector<Petal> s_petals[NUM_PETAL];

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
		if(id > 0 && id <= NUM_PETAL) {
			Petal pl;
			pl.pitch = atof(argv[1]);
			pl.roll = atof(argv[2]);
			pl.timestamp = atof(argv[3]);

			int a = (int)roundf(pl.timestamp * 100) % 10;
			if(a == 0) {
				s_petals[id - 1].push_back(pl);
			}
		}
	}
/*
	s_petals[1].clear();
	s_petals[1].assign(s_petals[0].begin(), s_petals[0].end());
*/
	fclose(fp);

	return true;
}

void play_file_cmd(const char *filename, int32_t replayCount)
{
	if(parse_flower_csv(filename) == false)
		return;

	for(uint8_t i=0;i<NUM_PETAL;i++) {
		printf("Petal[%d] : \n", i);
		for_each(s_petals[i].begin(), s_petals[i].end(), [&](Petal const & pl)
		{
			if(i==0)
			printf("\t%.2f, %.2f, %.2f\n", pl.pitch, pl.roll, pl.timestamp);
		});
	}

	vector<Petal>::iterator ppls[NUM_PETAL];
	for(uint8_t i=0;i<NUM_PETAL;i++) {
		ppls[i] = s_petals[i].begin();
	}

	uint32_t d_pitch[NUM_PETAL] = {0};
	uint32_t d_roll[NUM_PETAL] = {0};
	bool bFinished = false;
	const long long _interval = 100; // 100 ms
	while(!bShutdown && !bFinished) {
		steady_clock::time_point t1(steady_clock::now()); // T1

		for(uint8_t i=0;i<NUM_TCPCAN;i++) {
			s_tcpCan[i].EnableWriteCache();
		}
#ifdef DNET400
		for(int8_t i=0;i<NUM_PETAL;i++) {
/*
	0 : M8010L x 13
	1 : M8010L x 12
	2 : RMD X6V3 x 13
	3 : RMD X6V3 x 12
*/
			uint8_t _bus = i / NUM_DEV_PER_TCPCAN; // 0 ~ 13 / 13 ~ 25
			uint8_t _index = i % NUM_DEV_PER_TCPCAN;
			
			float p0 = ppls[i]->pitch;
			s_tcpCan[_bus].Device(_index)->WritePosition((int32_t)(p0 * 100), d_pitch[i] & 0xffff); // 0.01 degree
			if(_bus == 0 && _index == 0)
				printf("s_tcpCan[%d].Device(%d) -> angle(%f) deg, speed(%u) dps\n", _bus, _index, p0, d_pitch[i]);
			
			float p1 = ppls[i]->roll;
//			if(_bus == 0 && _index == 0)
			s_tcpCan[_bus + 2].Device(_index)->WritePosition((int32_t)(p1 * 100), d_roll[i] & 0xffff); // 0.01 degree
			if(_bus == 0 && _index == 0)
				printf("s_tcpCan[%d].Device(%d) -> angle(%f) deg, speed(%u) dps\n", _bus + 2, _index, p1, d_roll[i]);

			++ppls[i];
			
			d_pitch[i] = abs((int32_t)((ppls[i]->pitch - p0) * (1000 / _interval))); // dps
			d_roll[i] = abs((int32_t)((ppls[i]->roll - p1)  * (1000 / _interval))); // dps
		}
#endif
#ifdef ZQWL_CANET
		static int32_t pos = 0;
		static uint16_t speed = 0;
		//for(uint8_t i=0;i<NUM_PETAL;i++) {
		for(int8_t i=NUM_PETAL-1;i>=0;i--) {
/*
	0 : M8010L x 25
	1 : RMD X6V3 x 25
*/
			float p0 = ppls[i]->pitch;
			s_tcpCan[0].Device(i)->WritePosition((int32_t)(p0 * 100), d_pitch[i] & 0xffff); // 0.01 degree
			//if(i == 0)
			//	printf("s_tcpCan[0].Device(%d) -> angle(%f) deg, speed(%u) dps\n", i, p0, d_pitch[i]);
			
			float p1 = ppls[i]->roll;
			s_tcpCan[1].Device(i)->WritePosition((int32_t)(p1 * 100), d_roll[i] & 0xffff); // 0.01 degree
			//if(i == 0)
			//	printf("s_tcpCan[1].Device(%d) -> angle(%f) deg, speed(%u) dps\n", i, p1, d_roll[i]);

			++ppls[i];

			d_pitch[i] = abs((int32_t)((ppls[i]->pitch - p0) * (1000 / _interval))); // dps
			d_roll[i] = abs((int32_t)((ppls[i]->roll - p1)  * (1000 / _interval))); // dps

			//std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
#endif
		for(uint8_t i=0;i<NUM_TCPCAN;i++) {
			s_tcpCan[i].FlushWriteCache();
			s_tcpCan[i].DisableWriteCache();
		}
//printf("%s:%d\n", __PRETTY_FUNCTION__, __LINE__);
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

		for(uint8_t i=0;i<NUM_TCPCAN;i++) {
			if(s_tcpCan[i].IsConnected() == false) {
				bFinished = true;
				break;
			}
		}

		steady_clock::time_point t2(steady_clock::now()); // T2
		auto dt_us = duration_cast<microseconds>(t2 - t1).count();
		printf("dt_us = %ld\n", dt_us);

		std::mutex mtx;
		std::unique_lock<std::mutex> lock(mtx);

		dt_us = duration_cast<microseconds>(steady_clock::now() - t1).count();
		
		auto end = steady_clock::now() + std::chrono::microseconds(_interval * 1000 - dt_us);
		auto res = s_emergency.wait_until(lock, end);
		if(res != std::cv_status::timeout) {
			break; // Emergency Stop !!!
		}

		if(select_stdin(0) > 0) { // No timeout
			char ch;
			::read(0, &ch, 1); // Read one character in raw mode.
			if(ch == 'q' || ch == 'Q')
				break;
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

	if(busId == 0 || busId > NUM_TCPCAN) {
		printf("CAN bus ID out of range !!!\n");
		return;
	}
	uint8_t busIndex = busId - 1;

	uint8_t devId = stoi(tokens[2]) & 0xff;
	if(devId == 0 || devId > s_tcpCan[busIndex].NumberDevices()) {
		printf("CAN dev ID out of range !!!\n");
		return;
	}
	uint8_t motorIndex = devId - 1; // Motor index from 0 to maxNumMotors

	if(tokens.size() >= 4) {
		if(tokens[3] == "status") {
			s_tcpCan[busIndex].Device(motorIndex)->ReadStatus();
		} else if(tokens[3] == "reset") {
			s_tcpCan[busIndex].Device(motorIndex)->Reset();
		} else if(tokens[3] == "pos") {
			if(tokens.size() <= 4) {
				s_tcpCan[busIndex].Device(motorIndex)->ReadPosition();
			} else {
				int32_t pos = stoi(tokens[4]);
				if(tokens.size() >= 6) {
					uint16_t speed = stoi(tokens[5]);
					s_tcpCan[busIndex].Device(motorIndex)->WritePosition(pos, speed);
				} else
					s_tcpCan[busIndex].Device(motorIndex)->WritePosition(pos, 360);
			}
		} else if(tokens[3] == "pid") {
			s_tcpCan[busIndex].Device(motorIndex)->ReadPID();
		} else if(tokens[3] == "origin") {
			s_tcpCan[busIndex].Device(motorIndex)->WritePosition(0, 10); // Go to zero position whit speed 10 dps
		} else if(tokens[3] == "test") {
			if(tokens.size() >= 5) {
				int32_t pos = stoi(tokens[4]);
				uint16_t speed = 360; // dps
				if(tokens.size() >= 6)
					speed = stoi(tokens[5]);
				test_cmd(s_tcpCan[busIndex], motorIndex, pos, speed);
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
	for(uint8_t i=0;i<NUM_TCPCAN;i++) {
		s_tcpCan[i].EnableWriteCache();
		//for(uint8_t j=0;j<s_tcpCan[i].NumberDevices();j++) {
		for(int8_t j=s_tcpCan[i].NumberDevices()-1;j>=0;j--) {
			s_tcpCan[i].Device(j)->ReadStatus();
			s_tcpCan[i].Device(j)->ReadPosition();
		}
		s_tcpCan[i].FlushWriteCache();
		s_tcpCan[i].DisableWriteCache();
	}
#endif
	printf("Read status now, please wait ...\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	for(uint8_t i=0;i<NUM_TCPCAN;i++) {
		printf("=========================================================================================\n");
		printf("         TCPCAN %2.2u      |  Current  |  Multi Turn Angle  \n", i);
		printf("=========================================================================================\n");      
		for(uint8_t j=0;j<s_tcpCan[i].NumberDevices();j++) {
			if(s_tcpCan[i].Device(j)->LastRxDuration() > CanNode::timeout)
				printf("\033[0;31m"); /* Red */
			else
				printf("\033[0;32m"); /* Green */
			printf("Device[%2.2u] %12.12s |   %4.2f    |   %8.2lf\n", j, 
				(s_tcpCan[i].Device(j)->LastRxDuration() > CanNode::timeout) ? "timeout !!!" : "online ...",
				s_tcpCan[i].Device(j)->Current(),
				s_tcpCan[i].Device(j)->MultiTurnAngle());
		}
		printf("\033[0m"); /* Default color */
	}
}

void reset_cmd()
{
	// ???????????????????????? M8010L ??? 90??????
	// ???????????????????????? X6V3 ??? 360??????
	// ??????????????????????????? M8010L ??? 60??????
	// ???????????????????????? X6V3 ??? 360?????? 
}

#ifdef __cplusplus
extern "C" {
#endif

extern char *read_line();

#ifdef __cplusplus
}
#endif

#ifdef DNET400
#define TCPCAN_IP "192.168.1.5"
#define TCPCAN_PORT 5000
#endif


#ifdef ZQWL_CANET
#define TCPCAN_IP "192.168.1.253"
#define TCPCAN_PORT 1030
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
#ifdef DNET400
	std::thread tcpCan1Thread(TcpCanThread<M8010L>, std::ref(s_tcpCan[0]),
		TCPCAN_IP, TCPCAN_PORT, NUM_DEV_PER_TCPCAN); // 13 M8010L motors

	std::thread tcpCan2Thread(TcpCanThread<M8010L>, std::ref(s_tcpCan[1]),
		TCPCAN_IP, TCPCAN_PORT+100, NUM_DEV_PER_TCPCAN-1); // 12 M8010L motors

	std::thread tcpCan3Thread(TcpCanThread<RMDx6v3>, std::ref(s_tcpCan[2]),
		TCPCAN_IP, TCPCAN_PORT+200, NUM_DEV_PER_TCPCAN, 100); // 13 RMDx6 motors

	std::thread tcpCan4Thread(TcpCanThread<RMDx6v3>, std::ref(s_tcpCan[3]),
		TCPCAN_IP, TCPCAN_PORT+300, NUM_DEV_PER_TCPCAN-1); // 12 RMDx6 motors
#endif
#ifdef ZQWL_CANET
	std::thread tcpCan1Thread(TcpCanThread<M8010L>, std::ref(s_tcpCan[0]),
		TCPCAN_IP, TCPCAN_PORT, NUM_DEV_PER_TCPCAN); // 25 M8010L motors

	std::thread tcpCan2Thread(TcpCanThread<RMDx6v3>, std::ref(s_tcpCan[1]),
		TCPCAN_IP, TCPCAN_PORT+1, NUM_DEV_PER_TCPCAN); // 25 RMDx6 motors
#endif
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
					for(uint8_t i=0;i<NUM_TCPCAN;i++) {
						for(uint8_t j=0;j<s_tcpCan[i].NumberDevices();j++) {
							s_tcpCan[i].Device(j)->WritePosition(0, 10); // Go to zero position whit speed 10 dps
						}
					}
				} else if(tokens[0] == "can") {
					if(tokens.size() >= 4)
						can_cmd(tokens);
					else
						printf(help);
				} else if(tokens[0] == "tcp") {
					if(tokens.size() >= 2) {
						if(tokens[1] == "reconnect") {
							for(uint8_t i=0;i<NUM_TCPCAN;i++) {
								s_tcpCan[i].Reconnect();
							}
						} else if(tokens[1] == "disconnect") {
							for(uint8_t i=0;i<NUM_TCPCAN;i++) {
								s_tcpCan[i].Disconnect();
							}
						}
					} else
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

	for(int i=0;i<NUM_TCPCAN;i++) {
		s_tcpCan[i].Disconnect();
	}

#ifdef DNET400
	tcpCan1Thread.join();
	tcpCan2Thread.join();
	tcpCan3Thread.join();
	tcpCan4Thread.join();
#endif
#ifdef ZQWL_CANET
	tcpCan1Thread.join();
	tcpCan2Thread.join();
#endif
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
		if(msg->bus == 0 || msg->bus > NUM_TCPCAN)
			return; 
		if(msg->id == 0 || msg->id > s_tcpCan[msg->bus - 1].NumberDevices())
			return;

		CanMotor *cm = s_tcpCan[msg->bus - 1].Device(msg->id - 1);

		if(cm == nullptr)
			return;

		cm->WritePosition(msg->position * 100, msg->speed); // 0.01 degree, dps
	}
};

void status_publish()
{
	while (!bShutdown) {
		for(int i=0;i<NUM_TCPCAN;i++) {
			for(int j=0;j<s_tcpCan[i].NumberDevices();j++) {
				CanMotor *cm = s_tcpCan[i].Device(j);
				
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

	std::thread lcm_thread(lcm_main, argc, argv);
	
	can_main(argc, argv);

	lcm_thread.join();

	return 0;
}
