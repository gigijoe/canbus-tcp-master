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
		self.btnShutdown.move(16, 16)  
		self.btnShutdown.clicked.connect(self.cbShutdown)

		self.btnReset = QPushButton('reset', self)
		self.btnReset.resize(96, 32)
		self.btnReset.move(120, 16)  
		self.btnReset.clicked.connect(self.cbReset)

		self.btnStop = QPushButton('stop', self)
		self.btnStop.resize(96, 32)
		self.btnStop.move(224, 16)  
		self.btnStop.clicked.connect(self.cbStop)

		self.btnHome = QPushButton('home', self)
		self.btnHome.resize(96, 32)
		self.btnHome.move(328, 16)  
		self.btnHome.clicked.connect(self.cbHome)

		self.btnPlay = QPushButton('play', self)
		self.btnPlay.resize(96, 32)
		self.btnPlay.move(432, 16)  
		self.btnPlay.clicked.connect(self.cbPlay)

		self.btnScenario = QPushButton('scenario', self)
		self.btnScenario.resize(96, 32)
		self.btnScenario.move(536, 16)  
		self.btnScenario.clicked.connect(self.cbScenario)

		self.cboScenario = QComboBox(self)
		self.cboScenario.addItems(['Scenario 0', 'Scenario 1'])
		self.cboScenario.move(16, 64)	

		self.cboPlay = QComboBox(self)
		self.cboPlay.addItems(['0', '1', '2', '3', '4', '5', '6', '7', '8', '9'])
		self.cboPlay.resize(96, 24)
		self.cboPlay.move(120, 64)	

		self.labState = QLabel('Unknown', self)
		self.labState.resize(144, 32)
		self.labState.move(8, 440)

		self.labWarn = QLabel(self)
		self.labWarn.setWordWrap(True)
		self.labWarn.setStyleSheet("border: 1px solid black;")
		self.labWarn.resize(288, 96)
		self.labWarn.move(8, 112)
		self.labWarn.setText('No warning ...')

		self.labErr = QLabel(self)
		self.labErr.setWordWrap(True)
		self.labErr.setStyleSheet("border: 1px solid black;")
		self.labErr.resize(288, 96)
		self.labErr.move(328, 112)
		self.labErr.setText('No error ...')

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
		msg.index = 0
		now = datetime.now()
		msg.timestamp = int(datetime.timestamp(now))

		lc.publish("SERVER_COMMAND", msg.encode())

	def cbReset(self):
		msg = command_t()
		msg.action = "reset"
		msg.index = 0
		now = datetime.now()
		msg.timestamp = int(datetime.timestamp(now))

		lc.publish("SERVER_COMMAND", msg.encode())

	def cbStop(self):
		msg = command_t()
		msg.action = "stop"
		msg.index = 0
		now = datetime.now()
		msg.timestamp = int(datetime.timestamp(now))

		lc.publish("SERVER_COMMAND", msg.encode())

	def cbHome(self):
		msg = command_t()
		msg.action = "home"
		msg.index = 0
		now = datetime.now()
		msg.timestamp = int(datetime.timestamp(now))

		lc.publish("SERVER_COMMAND", msg.encode())

	def cbPlay(self):
		msg = command_t()
		msg.action = "play"
		msg.index = self.cboPlay.currentIndex()
		now = datetime.now()
		msg.timestamp = int(datetime.timestamp(now))

		lc.publish("SERVER_COMMAND", msg.encode())

	def cbScenario(self):
		msg = command_t()
		msg.action = "scenario"
		msg.index = self.cboScenario.currentIndex()
		now = datetime.now()
		msg.timestamp = int(datetime.timestamp(now))

		lc.publish("SERVER_COMMAND", msg.encode())

	def status_handler(self, channel, data):
		msg = status_t.decode(data)
		self.labState.setText(msg.state)

		str = ''
		if msg.num_err > 0:
			for i in range(int(msg.num_err)):
				str += msg.err_str[i]
				str += '\n'
			self.labErr.setText(str)
		else:
			self.labErr.setText('No error ...')

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

	lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
	s_status = lc.subscribe("SERVER_STATUS", ui.status_handler)

	lcm_thread = Thread(target=lcm_task)
	lcm_thread.daemon = True
	lcm_thread.start()

	r = app.exec()

	lc.unsubscribe(s_status)

	sys.exit(r)
