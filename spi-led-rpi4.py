import time
import spidev
import ws2812

from servolcm import led_t
import lcm

display_mode = 0            #0=self test mode, 1=normal play mode
nLED = 300                  #LED quantitly
HSV_Color = 0               #HSV color
GRB_Color = [0,0,0]         #Green/Red/Blue color
breathing = 0.2             #count human each area to breathing LED
LED_update_interval = 0.05  #unit:second
record_time = 0

def display_ws2812(nLED, HSV_Color, GRB_Color, LED_update_interval, record_time):
    try:
        while True:
            if time.time() > record_time + LED_update_interval:
                
                if display_mode == 1:
                    GRB_Color =[0,255,0]#program update GRB_Color
                else:
                    GRB_Color = HSV_2_GRB(HSV_Color)
                    
                GRB_Color = [int(GRB_Color[0] * breathing),int(GRB_Color[1] * breathing),int(GRB_Color[2] * breathing)]
                ws2812.write2812(spi, GRB_Color*nLED)#display all of LED    
                print("Time.time() =",record_time,", HSV_Color =", HSV_Color,", GRB_Color =", GRB_Color)
                HSV_Color = HSV_Color + 1
                record_time = time.time()
            
    except KeyboardInterrupt:
            ws2812.write2812(spi, [0,0,0]*nLED)#turn off all of LED

def HSV_2_GRB(HSV_Color):
    HSV_index = HSV_Color % 360
    if HSV_index < 60:
        GRB_Color = [int(HSV_index*255/60),255,0]
    elif HSV_index >= 60 and HSV_index < 120:
        GRB_Color = [255,int((120-HSV_index)*255/60),0]
    elif HSV_index >= 120 and HSV_index < 180:
        GRB_Color = [255,0,int((HSV_index-120)*255/60)]
    elif HSV_index >= 180 and HSV_index < 240:
        GRB_Color = [int((240-HSV_index)*255/60),0,255]
    elif HSV_index >= 240 and HSV_index < 300:
        GRB_Color = [0,int((HSV_index-240)*255/60),255] 
    elif HSV_index >= 300 and HSV_index < 360:
        GRB_Color = [0,255,int((360-HSV_index)*255/60)]
    return GRB_Color

def led_handler(channel, data):
    msg = led_t.decode(data)
    print("Received message on channel \"%s\" %d %d %d (%f)" % (channel, msg.r, msg.g, msg.b, msg.breathing))

    GRB_Color = [int(msg.r * msg.breathing),int(msg.g * msg.breathing),int(msg.b * msg.breathing)]
    ws2812.write2812(spi, GRB_Color * nLED)#display all of LED    

if __name__=="__main__":
    spi = spidev.SpiDev()
    spi.open(1,0)

    lc = lcm.LCM()

    s_status = lc.subscribe("LED", led_handler)

    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:
        pass


    #display_ws2812(nLED, HSV_Color, GRB_Color, LED_update_interval, record_time)