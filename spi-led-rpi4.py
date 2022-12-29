import time
import spidev

from servolcm import led_t
import lcm

nLED = 300                  #LED quantitly
GRB_Color = [0,0,0]         #Green/Red/Blue color
breathing = 0.2             #count human each area to breathing LED
LED_update_interval = 0.05  #unit:second
record_time = 0

def display_ws2812(nLED, LED_update_interval, record_time):
    try:
        while True:
            if time.time() > record_time + LED_update_interval:
                
                global GRB_Color
                    
                write2812_numpy4(spi, GRB_Color*nLED)#display all of LED    
                #print("Time.time() =",record_time,", ", GRB_Color =", GRB_Color)
                record_time = time.time()
            
    except KeyboardInterrupt:
            write2812_numpy4(spi, [0,0,0]*nLED)#turn off all of LED

import numpy
from numpy import sin, cos, pi

def write2812_numpy4(spi,data):
    #print spi
    d=numpy.array(data).ravel()
    tx=numpy.zeros(len(d)*4, dtype=numpy.uint8)
    for ibit in range(4):
        #print ibit
        #print ((d>>(2*ibit))&1), ((d>>(2*ibit+1))&1)
        tx[3-ibit::4]=((d>>(2*ibit+1))&1)*0x60 + ((d>>(2*ibit+0))&1)*0x06 +  0x88
        #print [hex(v) for v in tx]
    #print [hex(v) for v in tx]
    spi.xfer(tx.tolist(), int(4/1.25e-6)) #works, on Zero (initially didn't?)
    #spi.xfer(tx.tolist(), int(4/1.20e-6))  #works, no flashes on Zero, Works on Raspberry 3
    #spi.xfer(tx.tolist(), int(4/1.15e-6))  #works, no flashes on Zero
    #spi.xfer(tx.tolist(), int(4/1.05e-6))  #works, no flashes on Zero
    #spi.xfer(tx.tolist(), int(4/.95e-6))  #works, no flashes on Zero
    #spi.xfer(tx.tolist(), int(4/.90e-6))  #works, no flashes on Zero
    #spi.xfer(tx.tolist(), int(4/.85e-6))  #doesn't work (first 4 LEDS work, others have flashing colors)
    #spi.xfer(tx.tolist(), int(4/.65e-6))  #doesn't work on Zero; Works on Raspberry 3
    #spi.xfer(tx.tolist(), int(4/.55e-6))  #doesn't work on Zero; Works on Raspberry 3
    #spi.xfer(tx.tolist(), int(4/.50e-6))  #doesn't work on Zero; Doesn't work on Raspberry 3 (bright colors)
    #spi.xfer(tx.tolist(), int(4/.45e-6))  #doesn't work on Zero; Doesn't work on Raspberry 3
    #spi.xfer(tx.tolist(), int(8e6))

def led_handler(channel, data):
    msg = led_t.decode(data)
    print("Received message on channel \"%s\" %d %d %d (%f)" % (channel, msg.r, msg.g, msg.b, msg.breathing))

    global GRB_Color
    breathing = 0.2
    #GRB_Color = [int(GRB_Color[0] * msg.breathing), int(GRB_Color[1] * msg.breathing), int(GRB_Color[2] * msg.breathing)]
    GRB_Color = [int(msg.g * breathing), int(msg.r * breathing), int(msg.b * breathing)]
    print("GRB_Color =", GRB_Color)

import _thread
from threading import Thread 

def lcm_task():
    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:
        pass

if __name__=="__main__":
    spi = spidev.SpiDev()
    spi.open(0,0)

    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

    s_status = lc.subscribe("LED", led_handler)

    lcm_thread = Thread(target=lcm_task)
    lcm_thread.daemon = True
    lcm_thread.start()

    display_ws2812(nLED, LED_update_interval, record_time)
