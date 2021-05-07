
#!/usr/bin/python
#https://github.com/micropython/micropython/tree/master/drivers/display

#There are two hardware I2C peripherals with identifiers 0 and 1. Any available output-capable pins can be used for SCL and SDA but the defaults are given below.
#https://docs.micropython.org/en/latest/esp32/quickref.html#hardware-i2c-bus

from machine import I2C, Pin
from time import sleep

print("start i2c and scan")

scl = 22
sda = 21

i2c = I2C(scl=Pin(scl), sda=Pin(sda))
print(i2c, i2c.__class__)
#i2c = I2C(1,scl=Pin(scl), sda=Pin(sda), freq=4000000)

#if i2c == None or i2c.__class__ != SoftI2C: # instead of I2C
if i2c == None: 
    raise ValueError('I2C create failed')

else:
    print('i2C CREATED, scan bus. scl=%d, sda=%d' %(scl, sda))


devices = i2c.scan()
if len(devices) == 0:
    print("no i2c devices")
else:
    print ("devices list: ", devices) 
    print ("devices in hexa")
    for x in devices:
        print (hex(x))
"""
devices:  [54, 60]
list devices in hexa
0x36
0x3c
"""