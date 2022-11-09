#
#
#
                CAN0
  25 MOTORS -------------|      |   Ethernet    |    |
                CAN1     | RPi4 |---------------| PC |
  25 MOTORS -------------|      |               |    |

#
# 大花 Status
#

Scenario index : 0 ~ 2
Mode : manual / auto
State : shutdown / reset / idle / home / playing / error

RPi4 每秒發送目前 Scenario index / Mode / State 給 PC

#
# 大花 Command
#

Action : shutdown / reset / stop / home / play 

#
# 大花 Scenario
#

每個Scenario各自擁有獨立的劇本

#
# 大花 Scenario 0 (10組劇本,亂數播放或指定播放,表情辨識採樣)
#

PC 發送 Stop 給 RPi4 (等待)
|
|-PC 檢查 Mode == auto / Status == idle 
  |
  |-PC 發送 Scenario index 0 / Script index (0 ~ 9) 給 RPi4
    |
    |-PC 檢查 Scenario index == 0 / Script index == 0 ~ 9 / Status == playing)

#
# 大花 Scenario 1 (5 x 5 組劇本,依據人流數據亂數播放)
#

PC 每秒發送目前5個區域中的人數

#
# proto_t.lcm
#

package protolcm

struct status_t 
{
	byte scenario;
	string mode;
	string status;
	string errstr;
	int64_t timestamp;
}

struct command_t
{
	string action;
	byte scenario;
	byte script;
	int64_t timestamp;
}

struct region_t
{
	byte human_counts[5];
	int64_t timestamp;
}




