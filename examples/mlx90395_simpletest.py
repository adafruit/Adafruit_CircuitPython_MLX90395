# SPDX-FileCopyrightText: 2020 by Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
from time import sleep
import board
import busio
from adafruit_mlx90395 import MLX90395, OSR

i2c = busio.I2C(board.SCL, board.SDA)
sensor = MLX90395(i2c)
while True:
    mag_x, mag_y, mag_z = sensor.magnetic

    print("X1:{0:10.2f}, Y:{1:10.2f}, Z:{2:10.2f} uT".format(mag_x, mag_y, mag_z))
    print("")

    sleep(0.5)
