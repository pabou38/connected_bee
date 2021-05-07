print('unit test BME280')

from machine import Pin, I2C

import esp
esp.osdebug(None)

import gc
gc.collect()

# ESP32 - Pin assignement
i2c = I2C(scl=Pin(22), sda=Pin(21), freq=10000)
# ESP8266 - Pin assignement
#i2c = I2C(scl=Pin(5), sda=Pin(4), freq=10000)
if i2c == None:
    raise ValueError('I2C create failed')

# Complete project details at https://RandomNerdTutorials.com
#https://randomnerdtutorials.com/micropython-bme280-esp32-esp8266/
print('library 1')
import pb_BME280 # ruiz
try:
    bme = pb_BME280.BME280(i2c=i2c)
    temp = bme.temperature # float
    hum = bme.humidity
    pres = bme.pressure
    print('temp: %0.2f, pressure: %0.2f, humid: %0.2f' %(temp, pres, hum))

except Exception as e:
    print('Failed to read sensor.', str(e))

#https://github.com/robert-hh/BME280
print('library 2')
import bme280_float as bme280


bme280 = bme280.BME280(i2c=i2c)
print(bme280.values) # ('18.19C', '907.19hPa', '0.00%')

a = [0.0, 0.0, 0.0]
values = bme280.read_compensated_data(result=a)
print(a) # [18.58942, 90717.55, 0.0]

