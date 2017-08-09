#!/usr/bin/env python
import smbus
import time

bus = smbus.SMBus(1)
address = 0x20
servo = 0xF7
pos = 18000
servo_range = 1000

bus.write_byte_data(address, servo, pos)

time.sleep(1)

while(True):
    # print("Checking motor: ")
    # motor = bus.read_i2c_block_data(address, servo, 2)
    # print str(motor[0] + motor[1])
    bus.write_byte_data(address, servo, pos)
    pos += 1
    print str(pos)
    time.sleep(1)