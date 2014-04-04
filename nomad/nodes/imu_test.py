#!/usr/bin/env python

import sys, serial, time

serialRates = [ 2400, 4800, 9600, 19200, 38400, 57600, 115200 ]

for baudRate in serialRates:

    print "Trying ", baudRate

    try:
        imu = serial.Serial(
            port = '/dev/ttyUSB0',
            baudrate = baudRate,
            timeout = 5
            )
        imu.setRTS(level = True)
        imu.setDTR(level = True)

        imu.flushInput()
        imu.flushOutput()

        time.sleep(1.0)

    except:
        print 'Failed to open serial port'
        time.sleep(3.0)
        continue

    characterRead = ''
    while True:
        characterRead = imu.read(size = 1)
        if len(characterRead) < 1:
            print "Nothing to read"
            break

        if characterRead == '\n':
            print characterRead
            imu.close()
            break

        print characterRead

sys.exit(0)
