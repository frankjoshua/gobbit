#!/usr/bin/env python
import smbus
import time

bus = smbus.SMBus(1)
address = 0x20

bus.write_byte_data(address, 0xF2, 40)
bus.write_byte_data(address, 0xF1, 0)
bus.write_byte_data(address, 0xF0, 0)
time.sleep(1)

while(True):
    print("Checking motor: ")
    motor = bus.read_i2c_block_data(address, 0xFA, 1)
    print str(motor[0])
    time.sleep(1)