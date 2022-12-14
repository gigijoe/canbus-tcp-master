#!/usr/bin/env python
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QMessageBox
from PyQt5.QtWidgets import QTableWidget, QTableWidgetItem, QAbstractItemView
from PyQt5 import QtGui, QtCore
from PyQt5.QtCore import QTimer
import time

import lcm
from threading import Thread 

from servolcm import status_t

numServo = 50
numTcpCan = 2
numServoPerTcpCan = int(numServo / numTcpCan)

class UI(QWidget):
	def __init__(self):
		super().__init__()
		# Init UI
		self.setWindowTitle('Servo Status')
		self.resize(640, 960)
		self.table = QTableWidget(self)
		self.table.move(12, 12)
		self.table.resize(600, 920)
		self.table.setColumnCount(6)
		self.table.setColumnWidth(0, 24)
		self.table.setColumnWidth(1, 24)
		self.table.setColumnWidth(2, 72)		
		self.table.setColumnWidth(3, 72)
		self.table.setColumnWidth(4, 96)
		self.table.setColumnWidth(5, 144)
		self.tableLabels = ['Bus', 'ID', 'Status', 'Angle', 'Current', 'Temperature']
		self.table.setHorizontalHeaderLabels(self.tableLabels)
		self.table.setRowCount(numServo)
		for bus in range(numTcpCan):
			for id in range(numServoPerTcpCan):
				self.table.setItem(bus*numServoPerTcpCan+id, 0, QTableWidgetItem(str(bus+1)))
				self.table.setItem(bus*numServoPerTcpCan+id, 1, QTableWidgetItem(str(id+1)))
				self.table.setItem(bus*numServoPerTcpCan+id, 2, QTableWidgetItem('Unknown'))
				self.table.item(bus*numServoPerTcpCan+id, 2).setBackground(QtGui.QColor(255,0,0))
		self.table.setSelectionMode(QAbstractItemView.SingleSelection)
		self.show()

		self.timer = QTimer(self)
		self.timer.timeout.connect(self.timer_cb)
		self.timer.start(1000)  # this will emit every second 

		self.start_time = time.time()

	def status_handler(self, channel, data):
		msg = status_t.decode(data)
		if msg.bus == 0 or msg.id == 0:
			return
		bus = msg.bus-1
		id = msg.id-1
		if msg.online:
			if msg.active:
				self.table.setItem(bus*25+id, 2, QTableWidgetItem('On line'))
				self.table.item(bus*25+id, 2).setBackground(QtGui.QColor(0,255,0)) # Green
			else:
				self.table.setItem(bus*25+id, 2, QTableWidgetItem('Deactive'))
				self.table.item(bus*25+id, 2).setBackground(QtGui.QColor(255,255,0)) # Yellow
		else:
			self.table.setItem(bus*25+id, 2, QTableWidgetItem('Off line'))
			self.table.item(bus*25+id, 2).setBackground(QtGui.QColor(255,0,0)) # Red

		self.table.setItem(bus*25+id, 3, QTableWidgetItem(str(round(msg.multi_turn_angle, 2))))
		self.table.setItem(bus*25+id, 4, QTableWidgetItem(str(round(msg.current, 3))))		
		if msg.current > 1.0:
			self.table.item(bus*25+id, 4).setBackground(QtGui.QColor(128,0,128)) # Pupel
		else:
			self.table.item(bus*25+id, 4).setBackground(QtGui.QColor(255,255,255)) # White

		self.table.setItem(bus*25+id, 5, QTableWidgetItem(str(msg.temperature)))
		if msg.temperature > 70:
			self.table.item(bus*25+id, 5).setBackground(QtGui.QColor(255,0,0)) # Red
		elif msg.temperature > 60:
			self.table.item(bus*25+id, 5).setBackground(QtGui.QColor(255,165,0)) # Orange
		else:
			self.table.item(bus*25+id, 5).setBackground(QtGui.QColor(255,255,255)) # White

		self.start_time = time.time()

	def timer_cb(self):
		if time.time() - self.start_time > 2:
			print('timeout !!!')
			for i in range(numServo):
				self.table.setItem(i, 2, QTableWidgetItem('Unknown'))
				self.table.item(i, 2).setBackground(QtGui.QColor(255,0,0))
				self.table.setItem(i, 3, QTableWidgetItem(''))
				self.table.setItem(i, 4, QTableWidgetItem(''))
				self.table.setItem(i, 5, QTableWidgetItem(''))

	def about(self):
		msgBox = QMessageBox(QMessageBox.NoIcon, 'About', 'Display real time status of all servo')
		msgBox.exec()

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
	s_status = lc.subscribe("SERVO_STATUS", ui.status_handler)
	
	lcm_thread = Thread(target=lcm_task)
	lcm_thread.daemon = True
	lcm_thread.start()

	r = app.exec()

	lc.unsubscribe(s_status)

	sys.exit(r)
