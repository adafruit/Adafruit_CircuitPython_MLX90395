# SPDX-FileCopyrightText: 2020 by Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
from time import sleep
import board
import busio
from adafruit_mlx90395 import MLX90395, OSR
i2c = busio.I2C(board.SCL, board.SDA)
from adafruit_debug_i2c import DebugI2C
i2c = DebugI2C(i2c)

sensor = MLX90395(i2c)
# from adafruit_lis3mdl import LIS3MDL as li
# s2=li(i2c)
# sensor.gain = 1
sensor.oversample_rate = OSR.RATE_8X
# sensor._filter = 3

print("{:#018b}".format(sensor._reg0))
print("{:#018b}".format(sensor._reg2))
print("DONE")
while True:
    mag_x, mag_y, mag_z = sensor.magnetic
    print("looking for")
    print(" X: -13.57       Y: -29.99       Z: 5.00 uTesla")
    print("X1:{0:10.2f}, Y:{1:10.2f}, Z:{2:10.2f} uT".format(mag_x, mag_y, mag_z))
    # print("X2:{0:10.2f}, Y:{1:10.2f}, Z:{2:10.2f} uT".format(*s2.magnetic))
    print("")
    
    sleep(0.5)
