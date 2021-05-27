#https://phixed.co/blog/micropython-workflow/
#https://randomnerdtutorials.com/esp32-pinout-reference-gpios
#https://www.youtube.com/watch?v=yZjpYmWVLh8

"""
Pascal Boudalier
pboudalier@gmail.com
"""

# long pulse OK 500ms:  4 wifi, 4 blynk
# short pulse error 100ms : 4 wifi, 4 blynk, 8 watchdog
# red led: any error


version = "2.0"
log_file = 'log.txt'

# add file logging and button to send log content to terminal
# log file deleted when sent to terminal, avoid growing unbound
# all majors exception catched and written to persistent log in file system. # main protected by try 
# add entry in log file at hard boot


from utime import ticks_ms
start_time=ticks_ms() # note t=time() is in seconds. to measure execution time

print('\n\n================== bee ESP32 boot starting. version %s =======================\n\n' %version)

import gc
import os
import sys

"""
import upip
#upip.install('picoweb')
#upip.install('utemplate')
upip.install('micropython-logging')
"""

from machine import deepsleep, idle, reset_cause, DEEPSLEEP_RESET, DEEPSLEEP
from utime import sleep_ms, sleep, sleep_us, localtime, mktime
from machine import Pin, RTC, I2C, ADC, reset
from micropython import mem_info, const, stack_use

import _thread 
import blynklib_mp as blynklib # no .mpy
import ntptime

import network

import mynet

import urandom # build in, in help('modules'), ie list of module that can be imported
# u means micropython ified. subset of Cpython
# correspond to micropython-random in pypi.org/projects/micropython-random
# note almost empty library. there to avoid import errors

#import uos # for random
# uos.random is also available uos.urandom(2) returns 2 bytes b'\xf4\xe8'

# sensors library
from pb_max17043 import max17043
import bme280_float as bme280
#import pb_BME280
import tsl2561
from machine import freq
freq(160000000) # hx711
from hx711 import HX711

"""
# https://towardsdatascience.com/rich-generate-rich-and-beautiful-text-in-the-terminal-with-python-541f39abf32e
from rich import print, pretty
pretty.install()
from rich.console import console
console.log(init, log_locals=True) # #use Console object to print both the output and all the variables in the function.
"""

sensor_ok = const(1) # state of sensor, stored in RTC
sensor_failed = const(0)

sleep_sec_measure = 60*15 # deep sleep in sec between measure
sleep_sec_error = 30 # deep sleep in sec if error

first_connect = True # blynk connect call back
sensor_done = False # global. set when sensor value have been sent to blynk.  app thread wait for it
sync_repl_done = False # global. set to True when repl button is synched. app thread wait for it
sync_log_done = False # global. set to True when log button is synched. app thread wait for it
repl_value = -1 # will be set to repl button value, 0 or 1,  in sync call back. all thread wait for it
repl = False # global. set by end_app to, prevent watchdog to go to deep sleep

# used in log time stamp. make sure it is defined in any case
mday =0
hour = 0 
minute = 0

# pin definition
test_gpio = 15 # connect to gnd to disable deep slep
led_gpio = 5 # built in blue led  b.off() to lit
lux_power_gpio = 16 # power vcc for lux
bme_power_gpio = 17
load_power_gpio = 18 # not used unstable reading
red_led_gpio = 14 # for error

hx711_clk = 32
hx_data = 34

# The ESP32 has two I2C channels and any pin can be set as SDA or SCL
scl = 22
sda = 21

# blynk virtual pin
vpin_lux = 50
vpin_temp = 51
vpin_pres = 52
vpin_soc = 53
vpin_load = 54
vpin_terminal = 55

vpin_repl = 56 # set to ON to start webrepl and not deep sleep
vpin_led_repl = 57

vpin_led_ping = 58
vpin_vbat = 59
vpin_log = 60

########################################
# create log file
# a button can send log file content to terminal and delete it
# capture all exceptions. post mortem analysis
########################################
f = open(log_file, 'a') # append, otherwize file delete at each deep sleep

########################################
# set GPIO
########################################

# Lolin D32 non Pro
led = Pin(led_gpio, Pin.OUT, value=0)
# to signal we are running. will be set to off just before going to deep sleep
led.off() # connect to ground to lit off() will lit.

red_led = Pin(red_led_gpio, Pin.OUT, value=1) # inverse logic. gpio connected to minus

red_led.off() # to signal script has started
sleep_ms(1000)
red_led.on()

# test GPIO , input. prevent deep sleep if grounded
test_pin = Pin(test_gpio, Pin.IN, Pin.PULL_UP)

# to power lux and bme sensors from gpio
lux_power_pin = Pin(lux_power_gpio, Pin.OUT, value=0)
bme_power_pin = Pin(bme_power_gpio, Pin.OUT, value=0)
load_power_pin = Pin(load_power_gpio, Pin.OUT, value=0) # not used

# will be set to low at deep sleep
pin_to_power_down = [lux_power_pin, bme_power_pin, load_power_pin]


########################################
# LED functions
########################################
def pulse_led(n, msec): # on board led
  for i in range(n):
    led.on() # off
    sleep_ms(msec)
    led.off() # on
    sleep_ms(msec)

def error_led(sec): # error led
  red_led.off() # on
  sleep(sec) 
  red_led.on # off


#rand = urandom.randint(1,100)

# vbat 100K divisor. default input for ADC is 0 to 1V. use attenuation to extend input range 
# O to 4095 #https://www.youtube.com/watch?v=mzlq65fr3uI
# ADC vref 1.1 V
adc = ADC(Pin(35)) #https://www.youtube.com/watch?v=yZjpYmWVLh8
adc.atten(ADC.ATTN_11DB)    # set 11dB input attenuation (voltage range roughly 0.0v - 3.6v)
#ADC.ATTN_6DB: 6dB attenuation, gives a maximum input voltage of approximately 2.00v
adc_read = adc.read()

vbat = (adc_read *4.2)/ 2605.0
"""
vadc = 3.3 * ADC / 4095
ADC = 2605   vadc = 2.1v  vbat = 4.2 V
vbat - 4.2 / 4.2  = adc - 2695 / 2605
vbat = 4.2/2605 * ADC
"""
#vbat = round((2*adc_read/4096.0) * 3.3, 1) # adc / 4096.0 * 7.445 ?  vbat = 4.2  gpio = 2.1
print('adc: %d , vbat: %0.1f' %(adc_read, vbat))

# write to log file for hard boot. No time stamp yet
# check if the device woke from a deep sleep
if reset_cause() == DEEPSLEEP_RESET:
  print('ESP32: woke from a deep sleep')
else:
  print('ESP32: hard boot')
  s = 'Bee hard boot\n'
  x = f.write(s)
  print('wrote %d bytes: %s' %(x,s))

"""
  f.close()
  f = open(log_file)
  s = f.read()
  print('reading file: %d %s ' %(len(s),s) )

  s = 'test\n'
  x = f.write(s)
  print('wrote %d bytes: %s' %(x,s))

  f.close()
  f = open(log_file)
  s = f.read()
  print('reading file: %d %s ' %(len(s),s) )
"""


####################################
#  RTC
####################################

def init_RTC(init):
  global r
  # pass list of bytes eg  [1, 1, 1, 1, 0, 0, 0, 0, 0]

  ###########################################
  # RTC 16KB of SRAM
  ###########################################

  # index into RTC memory, 2 bytes
  # r.memory()[0] is lux error state
  # r.memory()[1] is bme error state
  # r.memory()[2] is lipo error state
  # r.memory()[3] is load error state
  # 4 number of lux read error since last reboot
  # 5 same for bme
  # 6 same for lipo
  # 7 same for load
  # 8 number of watchdog popped

  # 9 previous load 8 bit unsigned integer

  r = RTC()
  mem = r.memory()  # content survives deep sleep
  print('RTC memory: ', mem)

  if (mem == b''):
    print('RTC memory empty. initializing..')
    #r.memory(bytearray(0b00000000)) # need object with buffer protocol

    # store x bytes in RTC  
    # layout
    # 1 , 1 : reset error condition for sensor error. send alarm only first error and recovered
    # 0, 0, 0 : number of temp error, lipo error, watchdog reset , SINCE LAST POWER UP

    r.memory(bytes(init)) # need object with buffer protocol
    mem = r.memory() # type bytes, immutable
    print('RTC memory: ', mem) #  b'\x01\x01\x00\x00\x00'

  # to test a value  r.memory() [i]

  # to set a value 
  # x=r.memory()
  # x=bytearray(x) make mutable
  # x[i]=1
  # r.memory(x)

  # bit operation possible as well
  # r.memory and wifi_error == 0 , test if bit not set
  # r.memory (r.memory() or wifi_error), set bit
  # r.memory (r.memory() and not wifi_error) reset bit

  else:
    pass
  
# return RTC at index i
def read_RTC(i):
  global r
  return(r.memory() [i])

# set RTC at index i with value, byte, ie 8 bit integer
def set_RTC(i, value):
  global r
  x=r.memory()
  x=bytearray(x) #make mutable
  x[i]=value
  r.memory(x)

def inc_RTC(i): # increment RTC counter
  v = read_RTC(i)
  v = v + 1
  set_RTC(i, v)

##########################################
# in case sensor is OK. manage RTC and notifications
##########################################
def RTC_sensor_ok (index, string_recovered, sensor_value):

  sensor_value = round(sensor_value,1)

  if r.memory()[index] == sensor_failed: # we were in a sensor error state so change state and send notif
      print("RTC: error recovered: ", string_recovered, sensor_value)
      s= str(mday) + ' ' + str(hour) +':' + str(minute)
      s1 = '%s: ' + string_recovered
      s = (s1  %s) # use time stamp vs simple random number
      blynk.virtual_write(vpin_terminal, s)
      
      blynk.notify(string_recovered + ': %s ' %(str(sensor_value)))
      #blynk.email('pboudalier@gmail.com', string_recovered)
      # reset error condition in RTC memory
      set_RTC(index, sensor_ok)
      print ('error status set to OK. RTC: ', r.memory())
  else:
    pass # status is OK and sensor read is OK

##########################################
# in case sensor failed. manage RTC and notifications
##########################################

def RTC_sensor_failed (index, string_failed, counter , exception):
  global r
  # increment RTC error counter
  inc_RTC(counter)

  # write error to blynk terminal at each failed read
  # write content of RTC error counters
  s= str(mday) + ' ' + str(hour) +':' + str(minute)
  s1 = '%s: ' + string_failed
  s = (s1  %s) # use time stamp vs simple random number
  print('RTC: sensor failed : ', s , exception)
  s = s + ' ' + exception
  blynk.virtual_write(vpin_terminal, s)
  
    
  # send notification for first fail only. no flooding
  # update RTC flag
  if r.memory()[index] == sensor_ok: # we were in a sensor ok state, so need to change state and send notif.
    blynk.notify(string_failed)
    #blynk.email('pboudalier@gmail.com', string_failed)
    set_RTC(index, sensor_failed)
    print ("RTC: sensor failed: ", string_failed, r.memory())
  else: # already in failed state. do not send notif
    print('sensor still failing')

# RTC content send to blynk together with time stamp

print('content of /: ', os.listdir())
try:
  print('content of /lib: ', os.listdir('/lib'))
except Exception as e:
  print('content of /lib ', str(e))

##################################################
# repl
##################################################
def start_repl():
  # need to import once webrepl_setup from a usb/ttl connection to set password
  # creates webrepl_cfg.py (not visible in uPyCraft, visible w: os.listdir()
  # cannot just browse to IP, need client http://micropython.org/webrepl/
  import webrepl 
  print('import webrepl_setup once to set password')
  print('use http://micropython.org/webrepl/ to connect and use ws://192.168.1.181:8266/') 
  print('or use local webrepl.html, file:///C:/Users/pboud/micropython/webrepl-master/webrepl.html')
  # cannot use ws://192.168.1.9:8266/ directly in browser
  webrepl.start()


#########################################################
# deep sleep ESP32
# turns off led and power gpio
#########################################################
def go_to_deepsleep(sec):

  sleeptime_msec = sec * 1000
  print('ESP32 will deep sleep for %d sec. close log file, turn off led and sensor power' %sec)
  f.close()
  led.on() # off
  red_led.on() # off
  
  for p in pin_to_power_down:
    p.off()

  deepsleep(sleeptime_msec)

#############################################
# watchdog. prevent hang
# forces deep sleep
#############################################
def watchdog(sec): # tuple
  global repl
  print('==== watchdog thread started for %d sec' %sec)
  sleep(sec)
  print('!!!!! watchdog popped. repl: ', repl)

  # prevent deep sleep if in REPL
  if repl:
    print('watchdog: repl is true, do not deep sleep')
    return()

  # increment RTC error counter
  inc_RTC(8) # number of watchdog popped

  try:
    blynk.notify('Bee: watchdog popped')
    s = ('%d %d %2d: watchdog popped' %(mday, hour, minute)) # use time stamp vs simple random number
    blynk.virtual_write(vpin_terminal, s)
    sleep(3) # blynk to complete before deep sleep ?
    pulse_led(8,100) # 8 pulse of 100 ms
    error_led(5) # red led
    sleep(5)

  except Exception as e:
    s = '%d %d %2d: exception watchdog: %s' %(mday, hour, minute, str(e))
    print(s)
    f.write(s)

  finally:
    wifi.disconnect()
    blynk.disconnect()
    go_to_deepsleep(sleep_sec_error)  


#############################################
# automatic reconnect to blynk
# does not seem to be reliable, so the ping thread is used instead
#############################################
def blynk_reconnect(sec):
  global blynk
  print('==== blynk reconnect thread started for %d sec' %sec)
  while True:
    sleep(sec)
    print('   !!!!! blynk reconnect popped.')
    if blynk.connected():
      print('   blynk reconnect: already connected')
    else:
      print('   blynk reconnect: not connected')
      if (blynk.connect(timeout=30)): # boolean
        print('   blynk reconnect ***** blynk is REconnected *****')
      else:
        print('   blynk reconnect  !!!! cannot REconnect !!!!')

#############################################
# aka ping 
# force the connected state, so that pushing button will be captured while in repl, and reset executed
#############################################
def blynk_ping(sec):
  global repl
  print('==== blynk ping thread started for %d sec' %sec)

  flip = True

  while True: # flip flop led to keep blynk connected. ONLY if in REPL mode
    sleep(sec)
    if repl:
      if flip:
        blynk.virtual_write(vpin_led_ping,255)
        flip = False
      else:
        blynk.virtual_write(vpin_led_ping,0)
        flip = True 
    else:
      pass
  # while True


#####################################################
# try to connect to a given wifi
# https://docs.micropython.org/en/latest/library/network.html
# https://docs.micropython.org/en/latest/library/network.WLAN.html
#####################################################

def wifi_connect(ssid, psk, sta_if):
    i =0
    ok = True
    sta_if.connect(ssid, psk)

    while not sta_if.isconnected():
      print('F' , end='') # marker we scan for wifi
      sleep_ms(1000)
      i = i + 1
      if i >=10:
        ok=False
        break
         
    if ok == True: 
      print('\n\nconnected. network config:', sta_if.ifconfig())
      print ('status: ', sta_if.status()) # no param, link status
      print('ssid: ', ssid)
      print('rssi ', sta_if.status('rssi')) # no []
      return (sta_if) 
    else:
      print('cannot connect to %s' %(ssid))
      error_led(3)  # signal could not connect to this wifi
      sta_if.disconnect() # wifi:sta is connecting, return error 2nd attempt
      return(None)
# return None or sta_id


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
    s = '%d %d %2d: exception reading Lipo gauge: %s' %(mday, hour, minute, str(e))
    print(s)
    f.write(s)
    return(None,None)

###################################################
# read BME

def read_bme280(i2c):
  try:
    
    # alternate library
    #bme = pb_BME280.BME280(i2c=i2c)
    #temp = bme.temperature
    #hum = bme.humidity
    #pres = bme.pressure
    #return(temp, pres, hum)
    
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
    s = '%d %d %2d: exception reading bme280: %s' %(mday, hour, minute, str(e))
    print(s)
    f.write(s)
    return(None,None,None)

###################################################
# read tsl
# return exception
def read_tsl2561(i2c):
  try:
    tsl = tsl2561.TSL2561(i2c=i2c) 
    lux = tsl.read()
    return (lux, "OK")
  except Exception as e:
    # sensor saturated
    s = '%d %d %2d: exception reading tsl2561: %s' %(mday, hour, minute, str(e))
    print(s)
    f.write(s)
    return (None, str(e))


###################################################
# read load
def read_hx711(nb, driver):
  #print('HX711 %d read' %nb)
  # use different averaging method
  try:
    driver.power_on()
    sleep_ms(100)

    """
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
    """
    
    # simple average 
    # experience shows first 3 reads 
    ignore = 3
    a=0
    for i in range(nb):
        b = driver.read()
        print('HX711: ', b)
        if i > ignore -1:
          a = a + b
        else:
          print('i', sep='') # ignore those read
          
        sleep_ms(100)
    a = a/(nb-ignore)
    print('HX711: average: ', a)

    #a = (w + a) / 2.0 # average of average
    #print('HX711: overall average: ', a)
   
    l = a - offset
    #print('HX711: without offset %0.1f , in percent %0.1f'%(l, l/a))
    if l < 0:
      l = 0 - l # depends on polarity of E and A

    grams = l / scale
    print('HX711: in grams: %0.0f' %grams)

    driver.power_off()
    return(grams)

  except Exception as e:
    s = '%d %d %2d: exception reading hx711: %s' %(mday, hour, minute, str(e))
    print(s)
    f.write(s)
    driver.power_off()
    return(None)


###################################### 
# read all sensors
# cut power after reads
# publish all sensor values to blynk
# update RTC
# return 1 (sensor_done)
# assumes blynk is connected
######################################
def process_sensors(blynk, r):

  warm = 2
  retry = 5
  print('processing sensors ..')
  blynk.virtual_write(vpin_led_ping, 0)
  blynk.virtual_write(vpin_vbat, vbat)
  
  print('read all sensors')

  # do not go to deep sleep is a sensor fails
  # send notification and email for sensor first fail, or first recovered only, based on status stored in RTC memory
  # ie avoid flooding 

  # WARNING: do NOT call blynk.run(). it will block until inbound message. can cause deadlock or delays (wait for disconnect)

  #######################################
  # LIPO
  #######################################
  # powered by Lipo itself. could add a mosfet
  for i in range(warm): # some dummy read to warm up. WTF. why am I writting that
    (vcell, soc) = read_lipo_gauge(i2c)
    sleep_ms(10)

  for i in range(retry): # read until success 
    (vcell, soc) = read_lipo_gauge(i2c)
    if vcell == None:
      print('   BLYNK: !! error lipo sensor ' , (vcell, soc))
      sleep_ms(100)
    else:
      print('   BLYNK: OK vcell %0.1f, soc %0.1f'  %(vcell, soc))
      break
    # see also vbat from onboard ADC
    # vcell == None for error

  if vcell != None: # sensor OK
    blynk.virtual_write(vpin_soc, int(soc)) 
    RTC_sensor_ok(2, "Bee: Lipo recovered", soc) # manage RTC state, error counters and notifications

  else: # error , blynk counter not updated. will stay at last value. will deep sleep the normal way
    RTC_sensor_failed(2, 'Bee: Lipo failed', 6) # index of state and error counter
    error_led(2)

  #######################################
  # LUX
  #######################################
  
  lux = 10000.0 
  for i in range(retry):
    (lux,s)  = read_tsl2561(i2c) # s is "OK" or exception eg sensor saturated

    try:
      error = s.split(' ') [1] # in case of exception
    except:
      error = None
    
    if lux == None and error != 'saturated':   # real error, saturated is not considered as error per se
      print('   BLYNK: !! error lux sensor' , s)
      sleep_ms(1000)
    else:
      print('   BLYNK: OK or saturated lux %0.1f'  %(lux))
      break # we got a good reading, possibly saturated

  if lux != None or error == 'saturated': # sensor OK or saturated (lux = 10000)
    blynk.virtual_write(vpin_lux, int(lux)) # 
    RTC_sensor_ok(0, "Bee: Lux recovered", lux)

  else: # error sensor
    RTC_sensor_failed(0, 'Bee: Lux failed', 4 , s) 
    error_led(2)


  #######################################
  # Load
  #######################################

  prev_load = read_RTC(9)
  # prev_load = 0 at reset
  print('previous load %d' %prev_load)

  for i in range (warm): # some dummy read
    load = read_hx711(5, driver) # uses average of multiple readc
    sleep_ms(10)

  for i in range (retry): # exit as soon as read is not None
    load = read_hx711(10, driver) # input nb of read to average, returns in grams
    if load == None:
      print('   BLYNK: !! error load sensor, retry')
      sleep_ms(100) # retry
    else:
      print('   BLYNK: OK load  in grams %0.1f'  %(load))
      break

  # test for spurious read
  if load != None: # sensor OK
    load = load / 1000.0 # in kg
    load_int = int(load)

    # test for spurious read
    max_delta = 1 # ignore this read if two sequential read are more than 1 kg appart
    max_load = 60.0 # max load to be considered valid
    min_load = 1.0
  
    if abs(load_int-prev_load) > max_delta and prev_load != 0: # prev_load = 0 at reset
      s= 'load: spurious read. now %d, previous %d. do NOT update blynk and RTC' %(load_int, prev_load)
      print(s)
      blynk.virtualwrite(vpin_terminal, s)

    elif load > max_load or load < min_load:
      s= ('load: suspect read. %0.1f. do NOT update blynk and RTC' %load)
      print(s)
      blynk.virtualwrite(vpin_terminal, s)

    else:
      
      load = round(load,1)
      print('load is ok %0.1f' %load)
      blynk.virtual_write(vpin_load, load) # 
      set_RTC(9,load_int) # store as previous load in RTC memory

    RTC_sensor_ok(3, "Bee: Load recovered", load)

  else: # error 
    RTC_sensor_failed(3, 'Bee: Load failed', 7)
    error_led(2)
  
  #######################################
  # BME
  #######################################
  for i in range(warm):
    (temp, pres, hum) = read_bme280(i2c) # do not take first couple of read
    sleep_ms(10)

  for i in range(retry):
    (temp, pres, hum) = read_bme280(i2c)
    if temp == None:
      print('   BLYNK: !! error BME sensor')
      sleep_ms(100)
    else:
      print('   BLYNK: OK temp: %0.1f, pressure: %0.1f, humid: %0.1f' %(temp, pres, hum))
      break

  if temp != None: # temp sensor OK
    blynk.virtual_write(vpin_temp, round(temp,1))
    blynk.virtual_write(vpin_pres, int(pres))
    # write to blynk terminal, incl get time stamp(TZ) and flags
    s = ('%d/%d:%2d temp %0.0f pres %0.0f' %(mday, hour, minute, temp, pres)) # use time stamp vs simple random number
    # write flag as well
    s1=r.memory() #  will print b'\x01\x01\x00\x00\x00'
    s2= ''
    for i in range(9):
      s2 = s2 + str(s1[i]) + ' '
    blynk.virtual_write(vpin_terminal, '%s %s ' %(s,s2)) 
    RTC_sensor_ok (1, "Bee: BME recovered", temp)
  else:
    RTC_sensor_failed(1, 'Bee: BME failed', 5) 
    error_led(2)

  
  print('power down sensors')
  lux_power_pin.off()
  load_power_pin.off()
  bme_power_pin.off()

  print('   BLYNK: sensor update done', r.memory())
  return(True) # always return True sensor done

#sensor_done = True # to signal to main thread. main waiting on this global var
# could have had sensor error
# end_app thread wait on both sensor and button synch done



########################################
# blynk thread 
########################################

def blynk_thread(a):
  global sensor_done
  global devices, missing_sensor

  print('\nBLYNK: starting ..', a)
  
  #########################
  # connect call back
  # sync button
  #########################
  @blynk.handle_event("connect")
  def connect_handler():
    global first_connect
    global repl
    
    print("   BLYNK: in connect call back")
    
    """
    if repl:
      print('   Blynk: repl true, this is a reconnect. do not read sensors')
    """

    if first_connect: # avoid connect, disconnect
      print('   BLYNK: first connect')
      first_connect = False 
      print('   BLYNK: sync button')
      blynk.virtual_sync(vpin_repl) # repl button
      # WTF for some reason I could not synch the 2 button at once? only the first call back got called
      # 2nd sync is done in first button call back
      
      # could do sensor processing here
    else:
      print('    BLYNK: not first connect')
    
  #########################
  # disconnect call back
  #########################
  @blynk.handle_event("disconnect")
  def disconnect_handler():
    print("   BLYNK: in disconnect call back")

    """
    # done by separate threads (reconnect and ping)
    if (blynk.connect(timeout=30)): # boolean
      print(' BLYNK: disconnec: ***** blynk is REconnected *****')
    else:
      print('   BLYNK: disconnect: !!!! cannot REconnect !!!!')
    """

  #########################
  # REPL button call back 
  # update led
  # set repl_value global
  # if button = OFF while in repl, reset
  #########################
  # read repl button 56. if pushed, will run apps until pushed again, if not pushed, will go to deep sleep immediatly
  # communicate with two globals, bolean and int for value of button
  # called by sync or if button is pushed and app running, ie a way to stop the app and go to deep sleep
  # set sync_done = True. main wait for sync_done == True

  @blynk.handle_event('write V56')
  def write_virtual_pin_handler_v56(pin, value): # value is a list of str
    global sync_repl_done # to communicate to main thread that this button has synched
    global repl_value # value of button, int, 1 if button is ON
    global repl # bolean. set by end_app when REPL is running
    print('   v56 call back start button: vpin %d start button: %s.' %(pin, value[0]))
    repl_value = int(value[0]) # value of button
    
    if repl_value == 1: # button is ON
      print('   v56: repl button in ON')
      blynk.virtual_write(vpin_led_repl,255)
    else:
      print('   v56: repl button in OFF')
      blynk.virtual_write(vpin_led_repl,0)
      if repl:
        print('   v56: REPL button set to off while running. reset')
        blynk.virtual_write(vpin_terminal, "RESET")
        sleep(2)
        reset()

    sync_repl_done = True # button value synched
    # repl_value and sync_repl_done are set 

    # for some reason I could not synch the 2 button at once? only the first call back got called
    blynk.virtual_sync(vpin_log) 

  
  #########################
  # log file button call back 
  # if button = ON send log file to terminal and purge it 
  # if button = OFF, pass
  #########################

  @blynk.handle_event('write V60')
  def write_virtual_pin_handler_v60(pin, value): # value is a list of str
    global f
    global sync_log_done # to communicate to main thread that this button has synched
    print('   v60 call back start button: vpin %d start button: %s.' %(pin, value[0]))
    log_value = int(value[0]) # value of button
    if log_value == 1: # button is ON
      print('   v60: log button in ON. send log file content to terminal and delete file')
      try: # ceinture et bretelles
        f.close()
        f = open(log_file)
        s = f.read()
        print('log file content: %d %s'  %(len(s),s))
        blynk.virtual_write(vpin_terminal, s)
        blynk.virtual_write(vpin_terminal, 'deleting log file\n')
        f.close()
        # delete log file to make sure it does not grow
        # log information is therefore only available once on terminal
        os.remove(log_file)
        f = open(log_file, 'r')
      except Exception as e:
        pass

    else:
      print('   v60: log button in OFF., pass')
      # do nothing

    sync_log_done = True #button value synched


  ################################### 
  # END of callbacks
  # BLYNK connect and run
  ###################################

  # connect default timeout 30sec
  # try to connect to blynk with retry

  try:

    retry = 0
    while retry < 5:
      print('try to connect to blynk %d retry' %retry)
      if (blynk.connect(timeout=30)): # boolean
        print('   BLYNK: connect() returned. blynk is connected') # all processing was done in connect call back. 
        break
      else: 
        print('   BLYNK: !!!! cannot connect !!!! . retry')
        retry = retry + 1
        sleep(10)
    
    if blynk.connected() == False: # in case all connect attemps failed
      print('   BLYNK: could not connect after %d retry, go to deep sleep on error' %retry)
      pulse_led(4,100) # short pulse for error
      error_led(5) # red led
      wifi.disconnect()
      go_to_deepsleep(sleep_sec_error)
    else:
      print('   BLYNK: all is good')
      pulse_led(4,500) # long pulse for OK

      try:
        if missing_sensor:
          print('some I2C sensor not detected ', str(devices))
          blynk.virtualwrite(vpin_terminal, str(devices))
        else:
          print('all I2C sensors detected ', str(devices))
      except Exception as e:
        print(str(e))

  except Exception as e:
    print('exception in blynk connect ', str(e))
    wifi.disconnect()
    go_to_deepsleep(sleep_sec_error)


  ##############################
  try:
    sensor_done = process_sensors(blynk, r) # need to be global to be seen by other threads
  ##############################
  except Exception as e: # in case of any sensor error, do not crash, but go tpo deep sleep (or REPL) to retry next time
    print('exception in process_sensor ', str(e))
    s = '%d %d %2d: exception sensor: %s' %(mday, hour, minute, str(e))
    print(s)
    f.write(s)
    blynk.virtualwrite(vpin_terminal, s)
  finally:
    sensor_done = True

  print('   BLYNK: call blynk.run in endless loop')
  # blynk.run will make call back happen. need to call Blynk.run to have callback read start button value
  ######### run() will block for incomming event. DO NOT CALL if expecting none, or in seperate thread
  try:
    
    while True:
      blynk.run() # This function should be called frequently to process incoming commands and perform housekeeping of Blynk connection.
      sleep(1)
      print('run', end=' ')
      
  except Exception as e: # notify error and deep sleep to retry
    s = '%d %d %2d: exception in BLYNK run: %s' %(mday, hour, minute, str(e))
    print(s)
    f.write(s)

    blynk.notify('micropython blynk.run exception: %s ' %(str(e)))
    blynk.email('pboudalier@gmail.com', 'bee micropython blynk run exception', str(e))
    print('blynk run exception , deep sleep on error: ', sleep_sec_error)
    error_led(5) # red led
    wifi.disconnect()
    go_to_deepsleep(sleep_sec_error)

  print('end blynk thread')

# end Blynk thread

##############################################
# go to deep sleep or idle with repl 
# blynk.run as main
# end_app runs as a separate thread
# wait for global value sensor_done !=0 and sync_done == True 
###############################################
def end_app(a):

  try:

    print('==== end app thread started')

    global repl_value, sync_repl_done, sync_log_done # shared with blynk sync callback
    global repl # shared with watchdog, set here based on repl_value and pin status
    global sensor_done # shared with blynk connect call back 

    # wait until all button have been synched, and sensor have been read (trigerred in bly)

    while sensor_done == False or sync_repl_done == False or sync_log_done == False: 
      #print(sensor_done, sync_repl_done, sync_log_done)
      sleep(2)
      print('W', end='')

    pulse_led(6,500)

    print('sensor updated, button synched: ', sensor_done, sync_repl_done, sync_log_done)

    # connect GPIO 15 (pullup) to gnd to avoid any deep sleep. allow to reflash in peace
    if test_pin.value() == 0 or repl_value == 1:

      repl = True # to prevent watchdog to kill it all
      # signal REPL is ON
      print('repl is ON. keeps running, write to terminal, notify and email') 
      blynk.notify('micropython BEE not going to deep sleep, REPL is ON')
      blynk.email('pboudalier@gmail.com', 'micropython BEE', 'REPL is ON') 
      s = ('%d/%d:%d REPL ON' %(mday, hour, minute)) # use time stamp vs simple random number
      blynk.virtual_write(vpin_terminal, s) 
      start_repl()
      while True:
        sleep(10)  # idle forever. can update with webrepl

    else:

      print('REPL is OFF, will sleep shortly') # button is OFF
      sleep(5) # time for blynk.run to process virtual write ??
      
      print('disconnect from blynk and wifi ')
      blynk.disconnect()
      sleep(1)
      wifi.disconnect()
      sleep(1)

      print ('script execution time(ms): ', ticks_ms()-start_time)

      print('go to deep sleep until next measure ', sleep_sec_measure)
      go_to_deepsleep(sleep_sec_measure)

  except Exception as e:
    s = '%d %d %2d: exception in end app: %s' %(mday, hour, minute, str(e))
    print(s)
    f.write(s)

    print('disconnect from blynk and wifi ')
    blynk.disconnect()
    sleep(1)
    wifi.disconnect()
    sleep(1)
    go_to_deepsleep(sleep_sec_measure)


# end app

############################
# MAIN
# init RTC
# create Blynk object
# power sensors
# i2c init
# create hx711 driver
# start wifi and update time with ntp
# start watchdog, end_app, blynk_reconnect, blynk_ping thread
# run Blynk (call backs, connect and run)
# entiere main is protected with try
############################

# project meaudre, cloud server
# public blynk server

# protect ALL main
try:

  # define as global if in try: and not in straigh main
  global wifi
  global offset, scale, driver

  init_RTC([1, 1, 1, 1, 0, 0, 0, 0, 0, 0])

  blynk = blynklib.Blynk(mynet.token) # auth token configured in mynet.py token = 'xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx'
  print ('BLYNK: blynk on public server created: ' , type(blynk))

  #There are two hardware I2C peripherals with identifiers 0 and 1. Any available output-capable pins can be used for SCL and SDA but the defaults are given below.
  #https://docs.micropython.org/en/latest/esp32/quickref.html#hardware-i2c-bus


  print('power sensors with gpio')
  lux_power_pin.on()
  bme_power_pin.on()
  load_power_pin.on()
  sleep_ms(100)

  print("start i2c and scan")
  i2c = I2C(scl=Pin(scl), sda=Pin(sda))
  #i2c = I2C(1,scl=Pin(scl), sda=Pin(sda), freq=4000000)
  if i2c == None:
      #raise ValueError('I2C create failed. deep sleep')
      go_to_deepsleep(sleep_sec_error)
  else:
    print('i2C CREATED, scan bus', i2c)

  devices = i2c.scan()
  if len(devices) == 0:
    print("no i2c devices. deep sleep")
    go_to_deepsleep(sleep_sec_error)
  else:
    print ("devices list: ", devices) 
    print ("devices in hexa")
    for x in devices:
      print (hex(x), end= ' ')
    print(' ')
  """
  devices:  [54, 60]
  list devices in hexa
  0x36
  0x3c
  """
  if len(devices) != 3:
    missing_sensor = True
  else:
    missing_sensor = False

  print('create hx711 driver')
  driver = HX711(d_out=34, pd_sck=32)
  driver.channel= HX711.CHANNEL_A_64 # retrieving data from the channel 'A' with gain 128 and 64, 
  # offset and scale done with gain 64
  if driver == None:
      raise ValueError('load driver create failed. deep sleep')
      go_to_deepsleep(sleep_sec_error)
  else:
    print('HX711 load driver created ', driver)

  offset = 254000 # avec plateau
  # 41xxx for 4700 grams , 4 cells
  scale = 41500/4700


  ###############################################
  # start wifi
  # credential for wifi stored in mynet.py
  ################################################
  """
  net = [
  ['ssid1', 'pass1'] , \
  ['ssid2', 'pass2'] \
  ]
  """

  import mynet
  print('start wifi ', mynet.net)

  sta_if = network.WLAN(network.STA_IF)
  sta_if.active(True)
  sta_if.ifconfig(('192.168.1.181', '255.255.255.0','192.168.1.1', '8.8.8.8'))

  #wifi_ok = False

  # try to connect to all configured wifi
  for i in range(len(mynet.net)):
    print("\ntrying to connect to wifi %s ..." %(mynet.net[i][0]))

    wifi = wifi_connect(mynet.net[i][0], mynet.net[i][1], sta_if)
    
    if wifi != None:
      (ip, _,_,_) = wifi.ifconfig()
      print('\n************** wifi connected **************\n')
      #wifi_ok = True
      pulse_led(4,500) # long pulse for OK
      break
  else: # no break
  #if (wifi_ok == False):
    print('could not connect to any wifi')
    # blynk on board led
    pulse_led(4,100) # short pulse for error
    error_led(5) # red led
    
    print('deep sleep sec: ', sleep_sec_error)
    go_to_deepsleep(sleep_sec_error)

  # set local time from ntp server. 
  print('local time before ntp: ', localtime())
  for i in range(5): # retry in case of TIMEOUT
    try: # protect from timeout in ntp
      print('set time with ntp')
      ntptime.settime()
      (year, month, mday, hour, minute, second, weekday, yearday) = localtime()
      print('UTC time after ntp: ', localtime()) # tuple (year, month, mday, hour, minute, second, weekday, yearday)
      # adjust in code for TZ. RTC can only run in UTC
      t=mktime(localtime())
      t = t + 2*3600
      (year, month, mday, hour, minute, second, weekday, yearday) = localtime(t)
      print('local time after TZ: ', localtime(t))
      print('day: %d hour: %d mn: %d'%(mday, hour, minute ))
      break

    except Exception as e:
      print('exception in ntp ', str(e))
      # default value for time stamp, otherwize undefined
      (year, month, mday, hour, minute, second, weekday, yearday) = localtime()

      s = '%d %d %2d: exception in NTP: %s' %(mday, hour, minute, str(e))
      print(s)
      f.write(s)

    finally:

      #######################################
      # write another boot time stamp to log file, now we got NTP
      #######################################
      if reset_cause() == DEEPSLEEP_RESET:
        pass
      else: # timestamp in log 
        print("write time stamp to log file ", s)
        s = '%d %d %2d: ESP32 hard boot' %(mday, hour, minute)
        print(s)
        f.write(s)
        
  ######################################
  # start threads
  # end_app
  # watchdog
  # blynk_ping
  ######################################

  _thread.start_new_thread(end_app, ('pabou',))  # param need to be a tuple
  # will wait for blynk to run and read sensors, button to sync and call deep sleep or webrepl

  # force deep sleep
  _thread.start_new_thread(watchdog, (120,))  # param need to be a tuple

  #_thread.start_new_thread(blynk_reconnect, (90,))  # param need to be a tuple

  # keep blynk opened while in repl
  _thread.start_new_thread(blynk_ping, (10,))  # param need to be a tuple

  # run blynk in main
  blynk_thread(('pabou',)) # block at blynk.run() waiting for inbound event

except Exception as e:
  s = '%d %d %2d: exception in main: %s' %(mday, hour, minute, str(e))
  print(s)
  f.write(s)
  print('exception in running main, ', str(e))
  go_to_deepsleep(sleep_sec_error)
  


