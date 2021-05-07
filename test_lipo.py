
from pb_max17043 import max17043
from utime import sleep_ms
from machine import I2C, Pin

# ESP32 - Pin assignement
i2c = I2C(scl=Pin(22), sda=Pin(21), freq=10000)
# ESP8266 - Pin assignement
#i2c = I2C(scl=Pin(5), sda=Pin(4), freq=10000)
if i2c == None:
    raise ValueError('I2C create failed')


try:
    m = max17043(i2c)
    print(m)
    for i in range(5): # multiple read to stabilize ?
        vcell = round(m.getVCell(), 1)
        soc = round(m.getSoc(), 0)
        print(vcell, soc)
        sleep_ms(500)
    print('vcell %0.1f, soc %0.0f: '% (vcell, soc))

except Exception as e:
    print('exception reading Lipo gauge ' , str(e))

