
# sensors library
from pb_max17043 import max17043
import bme280_float as bme280
#import pb_BME280
import tsl2561
from machine import freq
freq(160000000) # hx711
from hx711 import HX711

from utime import sleep_ms, sleep, sleep_us, localtime, mktime



###################################################
# read SENSORS
# returns value or None
###################################################

####################################################
# read lipo gauge
# use modified library. create i2c object in main, and pass to library
#    can use same i2c for multiple sensors
#    original library had hardcoded i2c pins for pyboard
#####################################################
def read_lipo_gauge(i2c):
  vcell = -1 # 
  soc = -1
  try:
    m = max17043(i2c)
    for i in range(5): # multiple read to stabilize ?
      vcell = round(m.getVCell(), 1)
      soc = round(m.getSoc(), 0)
      #print(vcell, soc)
      sleep_ms(500)
    #print('vcell %0.1f, soc %0.0f: '% (vcell, soc))

    # print everything about the battery and the max17043 module
    # call the __str__ method
    #print(m)
    # restart the module
    #m.quickStart()
    # close the connection to the module
    #m.deinit()
    return(vcell, soc)
  except Exception as e:
    print('exception reading Lipo gauge ' , str(e))
    return(None,None)


def read_bme280(i2c):
  try:
    """
    # alternate library
    bme = pb_BME280.BME280(i2c=i2c)
    temp = bme.temperature
    hum = bme.humidity
    pres = bme.pressure
    return(temp, pres, hum)
    """
    bme = bme280.BME280(i2c=i2c)
    #a = [0.0, 0.0, 0.0]  buffer allocated from heap
    for i in range(5): # WTF. first read is always 23 and 883
      values = bme.read_compensated_data(result=None) # [18.58942, 90717.55, 0.0]
      sleep_ms(10)
    print('BME compensated ', values)

  
    temp = round(values[0],1)
    pres = round(values[1],0) /100.0 # hecto pascal
    hum = round(values[2],0) # in %
    return(temp, pres, hum)

  except Exception as e:
    print('exception reading bme280 ', str(e))
    return(None,None,None)

def read_tsl2561(i2c):
  try:
    tsl = tsl2561.TSL2561(i2c=i2c)
    lux = tsl.read()
    return lux
  except Exception as e:
    print('exception reading tsl2561: ', str(e))
    return None


def read_hx711(nb, driver):
  #print('HX711 %d read' %nb)
  # use different averaging method
  try:
    driver.power_on()
    sleep_ms(100)
    # stabilization
    values = []
    for _ in range(nb):
        values.append(driver.read())
        sleep_ms(10)

    weights = []
    for prev in values:
        weights.append(sum([1 for current in values if abs(prev - current) / (prev / 100) <= 10]))
    w = sorted(zip(values, weights), key=lambda x: x[1]).pop()[0]
    #print('HX711: stabilized: ', w)

    # simple average
    a=0
    for i in range(nb):
        a = a + driver.read()
        sleep_ms(10)
    a = a/nb
    #print('HX711: average: ', w)

    a = (w + a) / 2.0 # average of average
    #print('HX711: new average: ', a)
    l = a - offset
    #print('HX711: without offset %0.1f , in percent %0.1f'%(l, l/a))
    if l < 0:
      l = 0 - l # depends on polarity od E and A

    grams = l / scale
    print('HX711: in grams: %0.0f' %grams)

    driver.power_off()
    return(grams)

  except Exception as e:
    print('exception hx711 read: ', str(e))
    driver.power_off()
    return(None)