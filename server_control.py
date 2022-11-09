#!/usr/bin/env python
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QMessageBox
from PyQt5.QtWidgets import QTableWidget, QTableWidgetItem, QAbstractItemView
from PyQt5.QtWidgets import QPushButton, QComboBox
from PyQt5 import QtGui, QtCore
from PyQt5.QtCore import QTimer
import time
from datetime import datetime

import lcm
from threading import Thread 

from protolcm import status_t, command_t, region_t

class UI(QWidget):
	def __init__(self):
		super().__init__()
		# Init UI
		self.setWindowTitle('Server Control')
		self.resize(640, 480)

		self.btnShutdown = QPushButton('shutdown', self)
		self.btnShutdown.resize(96, 32)
		self.btnShutdown.move(8, 16)  
		self.btnShutdown.clicked.connect(self.cbShutdown)

		self.btnReset = QPushButton('reset', self)
		self.btnReset.resize(96, 32)
		self.btnReset.move(120, 16)  
		self.btnReset.clicked.connect(self.cbReset)

		self.btnHome = QPushButton('stop', self)
		self.btnHome.resize(96, 32)
		self.btnHome.move(224, 16)  
		self.btnHome.clicked.connect(self.cbStop)

		self.btnHome = QPushButton('home', self)
		self.btnHome.resize(96, 32)
		self.btnHome.move(328, 16)  
		self.btnHome.clicked.connect(self.cbHome)

		self.btnPlay = QPushButton('play', self)
		self.btnPlay.resize(96, 32)
		self.btnPlay.move(432, 16)  
		self.btnPlay.clicked.connect(self.cbPlay)

		self.cboScenario = QComboBox(self)
		self.cboScenario.addItems(['Scenario 0', 'Scenario 1'])
		self.cboScenario.move(8, 64)	

		self.labState = QLabel('Unknown', self)
		self.labState.resize(144, 32)
		self.labState.move(8, 440)

		self.show()

		self.timer = QTimer(self)
		self.timer.timeout.connect(self.timer_cb)
		self.timer.start(1000)  # this will emit every second 

		self.start_time = time.time()

	def timer_cb(self):
		if time.time() - self.start_time > 2:
			print('timeout !!!')
			self.labState.setText('off line')

	def cbShutdown(self):
		msg = command_t()
		msg.action = "shutdown"
		msg.scenario = self.cboScenario.currentIndex()
		msg.script = 0
		now = datetime.now()
		msg.timestamp = int(datetime.timestamp(now))

		lc.publish("SERVER_COMMAND", msg.encode())

	def cbReset(self):
		msg = command_t()
		msg.action = "reset"
		msg.scenario = self.cboScenario.currentIndex()
		msg.script = 0
		now = datetime.now()
		msg.timestamp = int(datetime.timestamp(now))

		lc.publish("SERVER_COMMAND", msg.encode())

	def cbStop(self):
		msg = command_t()
		msg.action = "stop"
		msg.scenario = self.cboScenario.currentIndex()
		msg.script = 0
		now = datetime.now()
		msg.timestamp = int(datetime.timestamp(now))

		lc.publish("SERVER_COMMAND", msg.encode())

	def cbHome(self):
		msg = command_t()
		msg.action = "home"
		msg.scenario = self.cboScenario.currentIndex()
		msg.script = 0
		now = datetime.now()
		msg.timestamp = int(datetime.timestamp(now))

		lc.publish("SERVER_COMMAND", msg.encode())

	def cbPlay(self):
		msg = command_t()
		msg.action = "play"
		msg.scenario = self.cboScenario.currentIndex()
		msg.script = 0
		now = datetime.now()
		msg.timestamp = int(datetime.timestamp(now))

		lc.publish("SERVER_COMMAND", msg.encode())

	def status_handler(self, channel, data):
		msg = status_t.decode(data)
		self.labState.setText(msg.state)

		self.start_time = time.time()

def lcm_task():
    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
	app = QApplication(sys.argv)
	ui = UI()

	lc = lcm.LCM()
	s_status = lc.subscribe("SERVER_STATUS", ui.status_handler)

	lcm_thread = Thread(target=lcm_task)
	lcm_thread.daemon = True
	lcm_thread.start()

	r = app.exec()

	lc.unsubscribe(s_status)

	sys.exit(r)
