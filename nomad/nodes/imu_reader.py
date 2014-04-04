#!/usr/bin/env python

import sys, serial

try:
    imu = serial.Serial(
        port = '/dev/ttyUSB0',
        baudrate = 38400,
        timeout = 3
        )
    imu.setRTS(level = True)
    imu.setDTR(level = True)
except:
    print 'Failed to open serial port'
    sys.exit(1)

while True:

    characterRead = ''
    imuData = ''
    inMessage = False
    while True:
        characterRead = imu.read(size = 1)
        if len(characterRead) < 1:
            continue

        if characterRead == '!':
            inMessage = True
            continue

        if characterRead == '\n':
            inMessage = False
            break

        if inMessage:
            imuData = imuData + characterRead
            continue

    imuFields = imuData[:-1].split(',')
    if len(imuFields) < 11:
        print 'Incomplete IMU message'
        print imuData
        continue

    print imuData

