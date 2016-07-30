#!/usr/bin/env python

import time
import L298NHBridge

HBridge = L298NHBridge.L298NHBridge()
HBridge.Init()

HBridge.SetMotorRight(0.3)
time.sleep(5)
HBridge.SetMotorRight(0)
