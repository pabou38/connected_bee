from hx711 import HX711
from utime import sleep_us

print('\n\nunit test HX711')
#https://circuitjournal.com/50kg-load-cells-with-HX711
#https://github.com/j-dohnalek/hx711py/blob/master/calibration.py
#https://learn.sparkfun.com/tutorials/load-cell-amplifier-hx711-breakout-hookup-guide/all#installing-the-hx711-arduino-library-and-examples
# https://youtu.be/LIuf2egMioA 
# https://www.youtube.com/watch?v=Gheo1ptZnXE

#https://github.com/j-dohnalek/hx711py  PYTHON 

#https://www.hbm.com/en/7163/wheatstone-bridge-circuit/

"""
calibration: need to set offset and scale

tare: read value at zero load. this is offset

read value at known load  measured weigth = value -offset
scale = measure weigth / known weight

gram = read - offset / scale
"""

class Scales(HX711):
    def __init__(self, d_out, pd_sck):
        super(Scales, self).__init__(d_out, pd_sck)
        self.offset = 0

    def reset(self):
        self.power_off()
        self.power_on()

    def tare(self):
        self.offset = self.read()

    def raw_value(self):
        return self.read() - self.offset

    def stable_value(self, reads=10, delay_us=500):
        values = []
        for _ in range(reads):
            values.append(self.raw_value())
            sleep_us(delay_us)
        return self._stabilizer(values)

    @staticmethod
    def _stabilizer(values, deviation=10):
        weights = []
        for prev in values:
            weights.append(sum([1 for current in values if abs(prev - current) / (prev / 100) <= deviation]))
        return sorted(zip(values, weights), key=lambda x: x[1]).pop()[0]

"""
if __name__ == "__main__":
    scales = Scales(d_out=5, pd_sck=4)
    scales.tare()
    val = scales.stable_value()
    print(val)
    scales.power_off()
"""

from machine import freq
freq(160000000)
from hx711 import HX711

offset = 254000 # avec plateau
# 41xxx for 4700 grams
scale = 41500/4700

print('create driver')
driver = HX711(d_out=34, pd_sck=32)
print(driver)

driver.power_on()

print('hx711: ', driver.read())

driver.channel= HX711.CHANNEL_A_64 # retrieving data from the channel 'A' with gain 128 and 64,
print('hx711 gain 64: ', driver.read())
driver.power_off()

driver.power_on()

nb = 25
values = []
for _ in range(nb):
    values.append(driver.read())
    sleep_us(500)

weights = []
for prev in values:
    weights.append(sum([1 for current in values if abs(prev - current) / (prev / 100) <= 10]))
w = sorted(zip(values, weights), key=lambda x: x[1]).pop()[0]
print("hx711 stabilized:", w)


# simple average
a=0
for i in range(nb):
    a = a + driver.read()
a = a/nb
print('HX711 average: ', a)

a = (w + a) / 2.0

l = a - offset

print('without offset %0.1f , in percent %0.1f'%(l, l/a))

l = l / scale

print('in grams: %0.0f' %l)












