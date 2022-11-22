#!/bin/sh
cp -av build/canbus-master /usr/local/bin
cp -av usr/local/bin/canbus-master.sh /usr/local/bin
cp -av etc/systemd/system/canbus-master.service /etc/systemd/system/

cp -av pwm-servo-rpi4.py /usr/local/bin
cp -av etc/systemd/system/pwm-servo-rpi4.service /etc/systemd/system/

#sudo systemctl enable canbus-master.service
#sudo systemctl enable pwm-servo-rpi4.service