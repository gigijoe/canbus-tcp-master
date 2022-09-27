#!/usr/bin/env python
import dearpygui.dearpygui as dpg
import lcm
import time
import _thread
from threading import Thread 

import threading
import numpy as np

from servolcm import position_t
from servolcm import status_t

def send_position():
    msg = position_t()
    msg.bus = int(dpg.get_value(tag_busid))
    msg.id = int(dpg.get_value(tag_canid))
    msg.position = dpg.get_value(tag_angle)
    msg.speed = dpg.get_value(tag_speed)
    lc.publish("POSITION", msg.encode())

def status_handler(channel, data):
    msg = status_t.decode(data)
    print("Received message on channel \"%s\"" % channel)
    if msg.bus == int(dpg.get_value(tag_busid)) and msg.id == int(dpg.get_value(tag_canid)):
        dpg.set_value(tag_encoder_position, msg.encoder_position)
        dpg.set_value(tag_multi_turn_angle, round(msg.multi_turn_angle, 2))
        dpg.set_value(tag_voltage, round(msg.voltage, 3))
        dpg.set_value(tag_current, round(msg.current, 3))
        dpg.set_value(tag_temperature, msg.temperature)

event = threading.Event()

def periodic(period, f, *args):
    def g_tick():
        t = time.time()
        while True:
            t += period
            yield max(t - time.time(), 0)
    g = g_tick()
    while True:
        time.sleep(next(g))
        f(*args)

s_tick = 0
s_angle = 0

def sine_task():
    if event.is_set() == False:
        return
    global s_tick, s_angle
    angle = np.sin(s_tick * 2 * np.pi / 360) * 90
    #av = abs(angle - s_angle) / 0.02 + 6
    av = abs(angle - s_angle) / 0.1 # duration is 0.1 second
    s_angle = angle
    s_tick = s_tick + 1

    msg = position_t()
    msg.bus = int(dpg.get_value(tag_busid))
    msg.id = int(dpg.get_value(tag_canid))
    msg.position = int(s_angle) 
    msg.speed = int(av)
    lc.publish("POSITION", msg.encode())
    print('pos ' + str(msg.position) + ' / speed ' + str(msg.speed))

def lcm_task():
    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:
        pass

tag_busid = 0;
tag_canid = 0;
tag_angle = 0
tag_speed = 0
tag_encoder_position = 0;
tag_voltage = 0
tag_current = 0
tag_temperature = 0
tag_test_button = 0

def execute_callback():
    print("Execute Clicked")
    send_position()

def test_callback():
    if event.is_set():
        event.clear()
        dpg.set_item_label(tag_test_button, "Start test")
    else:
        event.set()
        dpg.set_item_label(tag_test_button, "Stop test")

def exit_callback():
    print('exit_callback')
    event.clear()

if __name__=='__main__':

    dpg.create_context()
    dpg.create_viewport()
    dpg.setup_dearpygui()

    dpg.set_exit_callback(exit_callback)

    with dpg.window(label="Example Window", width=720, height=320):
        dpg.add_text("Servo Control")
        tag_busid = dpg.add_combo(("1", "2", "3", "4"), label="Bus ID", default_value="1")
        tag_canid = dpg.add_combo(("1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14"), label="CAN ID", default_value="1")
        dpg.add_separator()

        tag_angle = dpg.add_slider_int(label="Position Control Angle (Degree)", min_value=0, max_value=359)
        tag_speed = dpg.add_slider_int(label="Speed of Position Control (dps)", min_value=0, max_value=60, default_value=15)
        tag_encoder_position = dpg.add_text(label="Encoder Position", show_label=True)
        tag_multi_turn_angle = dpg.add_text(label="Multi Turn Angle", show_label=True)
        tag_voltage = dpg.add_text(label="Voltage (V)", show_label=True)
        tag_current = dpg.add_text(label="Current (A)", show_label=True)
        tag_temperature = dpg.add_text(label="Temperature (Degree)", show_label=True)
        dpg.add_button(label="Execute", callback=execute_callback)
        tag_test_button = dpg.add_button(label="Start test", callback=test_callback)

    lc = lcm.LCM()

    s_status = lc.subscribe("STATUS", status_handler)

    lcm_thread = Thread(target=lcm_task)
    lcm_thread.daemon = True
    lcm_thread.start()

    event.clear()
    sine_thread = Thread(target=periodic, args=(0.1, sine_task))
    sine_thread.daemon = True
    sine_thread.start()

    dpg.show_viewport()
    dpg.start_dearpygui()
    dpg.destroy_context()

    lc.unsubscribe(s_status)
