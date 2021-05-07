"""
>>> upip.install("tsl2561")
That method of installation only works if the module is on PyPi. Further, upip requires a special format in order to work on the ESP8266 so only modules with names like "micropython-xyz" are likely to be compatible.
"""
#https://github.com/adafruit/micropython-adafruit-tsl2561
#https://docs.micropython.org/en/latest/library/machine.I2C.html

print('unit test tsl2561')
from machine import Pin, I2C, SoftI2C
import tsl2561

import esp
esp.osdebug(None)

import gc
gc.collect()


# I2C or SoftI2C

i2c = I2C(scl=Pin(22), sda=Pin(21))
print(i2c, i2c.__class__)

if i2c == None:
    raise ValueError('I2C create failed')

try:
    print('create lux sensor')
    tsl2561 = tsl2561.TSL2561(i2c=i2c)

    print('read lux sensor')
    lux = tsl2561.read()
    print(lux, type(lux))

except Exception as e:
    print('Failed to read lux sensor.', str(e))
