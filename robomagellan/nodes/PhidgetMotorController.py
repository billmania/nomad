"""Phidgets HC Motor Control ROS class

"""

__author__ = 'Bill Mania <bill@manailabs.us>'
__version__ = '1'

import time
import math
from ctypes import *
from Phidgets.Devices.MotorControl import MotorControl
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException


class PhidgetMotorController:

    def __init__(self):
        self.leftWheels = 0
        self.rightWheels = 1

        self.motorControl = MotorControl()

        self.motorControl.setOnAttachHandler(self.mcAttached)
        self.motorControl.setOnDetachHandler(self.mcDetached)
        self.motorControl.setOnErrorhandler(self.mcError)
        self.motorControl.setOnCurrentChangeHandler(self.mcCurrentChanged)
        self.motorControl.setOnInputChangeHandler(self.mcInputChanged)
        self.motorControl.setOnVelocityChangeHandler(self.mcVelocityChanged)

        try:
            self.motorControl.openPhidget()

        except PhidgetException, e:
            print "openPhidget() failed"
            print "code: %d" % e.code
            print "message", e.message

            raise

        try:
            self.motorControl.waitForAttach(10000)

        except PhidgetException, e:
            print "waitForAttach() failed"
            print "code: %d" % e.code
            print "message", e.message
    
            raise

        if self.motorControl.isAttached():
            print("Device: %s, Serial: %d, Version: %d" % (
                    self.motorControl.getDeviceName(),
                    self.motorControl.getSerialNum(),
                    self.motorControl.getDeviceVersion()
                    ))
        else:
            print ("Failed")
    
        self.minAcceleration = self.motorControl.getAccelerationMin(self.leftWheels)
        self.maxAcceleration = self.motorControl.getAccelerationMax(self.leftWheels)
        

    def move(self, translationX, rotationZ):
        """Move the rover base
    
        rotationZ specifies the amount to first rotate the rover base
        and then translationX is the amount to translate the rover base.
        the rotation argument is given in radians and the translation
        argument is given in meters per second.
    
        """
    
        if (abs(rotationZ) > 0.1):
            self.rotate(rotationZ)
        else:
            self.translate(translationX)
    
        return
    
    def rotate(self, rotationZ):
        leftSpeed = -100.0 * rotationZ
        rightSpeed = 100.0 * rotationZ
    
        self.motorControl.setVelocity(self.leftWheels, leftSpeed);
        self.motorControl.setVelocity(self.rightWheels, rightSpeed);
    
        return
    
    def translate(self, translationX):
        leftSpeed = 100.0 * translationX
        rightSpeed = 100.0 * translationX
    
        self.motorControl.setVelocity(self.leftWheels, leftSpeed);
        self.motorControl.setVelocity(self.rightWheels, rightSpeed);
    
        return
    
    def mcAttached(self, e):
        return
    
    def mcDetached(self, e):
        return
    
    def mcError(self, e):
        return
    
    def mcCurrentChanged(self, e):
        return
    
    def mcInputChanged(self, e):
        return
    
    def mcVelocityChanged(self, e):
        return

if __name__ == "__main__":
    motorControl = PhidgetMotorController()

    while True:
        motorControl.move(0.0, 1.0)
        time.sleep(0.1)
