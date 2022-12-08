#
#
#

                                  控制主機                系統主機
		                CAN0     +------+               +----+
舉昇馬達    25 MOTORS -------------|      |   Ethernet    |    |
		       |        CAN1     | RPi4 |---------------| PC |
旋轉馬達    25 MOTORS -------------|      |               |    |
		       |                 +------+               +----+
		       |                  |    |
		     RSP-1600 Power-------+    | 16 Channel PWM 
		                               |
		                               +----- 5 SERVOS  花蕊
		                               |
		                               +----- 3 SERVOS  小花

#
# 大花 Status
#

Scenario index : 0 ~ 1
State : shutdown / reset / idle / home / playing / error

RPi4 每秒發送目前 Scenario index / State 給 PC

#
# 大花 Command
#

Action : shutdown / reset / stop / home / play / scenario

#
# 大花 Scenario
#

每個Scenario各自擁有獨立的劇本

#
# 大花 Scenario 0 (10組劇本,亂數播放或指定播放,表情辨識採樣)
#

PC 檢查 Status == idle 
|
|-PC 發送 command_t->action = scenario / index = 0 給 RPi4
   |
   |-PC 發送 command_t->action = play / index = 0 ~ 9 給 RPi4

#
# 大花 Scenario 1 (5 x 5 組劇本,依據人流數據亂數播放)
#

PC 每秒發送目前5個區域中的人數

PC 檢查 Status == idle 
|
|-PC 發送 command_t->action = scenario / index = 1 給 RPi4
   |
   |-PC 發送 region_t->human_counts[5] 給 RPi4

#
# proto_t.lcm
#

package protolcm

struct status_t 
{
	byte scenario;
	string state;
	int32_t num_warn;
	string warn_str[num_warn];
	int32_t num_err;
	string err_str[num_err];
	int64_t timestamp;
}

struct command_t
{
	string action;
	byte index;
	int64_t timestamp;
}

struct region_t
{
	byte human_counts[5];
	int64_t timestamp;
}




