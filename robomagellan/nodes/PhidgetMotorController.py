"""Phidgets HC Motor Control ROS class

"""

__author__ = 'Bill Mania <bill@manailabs.us>'
__version__ = '1'

import rospy
import time
from ctypes import *
from Phidgets.Devices.MotorControl import MotorControl
from Phidgets.PhidgetException import PhidgetException


class PhidgetMotorController:

    def __init__(self):
        self.leftWheels = 0
        self.rightWheels = 1
        self.defaultMotorSpeed = 100.0
        self.motorMaxSpeed = 100
        self.motorMinSpeed = 20
        self.leftAdjustment = 1.0
        self.rightAdjustment = 1.0

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
            rospy.loginfo("Device: %s, Serial: %d, Version: %d" % (
                    self.motorControl.getDeviceName(),
                    self.motorControl.getSerialNum(),
                    self.motorControl.getDeviceVersion()
                    ))
        else:
            print ("Failed")
    
        self.minAcceleration = self.motorControl.getAccelerationMin(self.leftWheels)
        self.maxAcceleration = self.motorControl.getAccelerationMax(self.leftWheels)

        self.currentAcceleration = int((self.maxAcceleration - self.minAcceleration) / 2 + self.minAcceleration)
        self.motorControl.setAcceleration(self.leftWheels, self.currentAcceleration)
        self.motorControl.setAcceleration(self.rightWheels, self.currentAcceleration)
        rospy.loginfo('Current acceleration: %d' % (self.currentAcceleration))

        rospy.loginfo('PhidgetMotorController initialized')

        return
        
    def setDefaultSpeed(self, defaultSpeed):
        self.defaultMotorSpeed = defaultSpeed

        return

    def getDefaultSpeed(self):
        return self.defaultMotorSpeed

    def move(self, translationX, rotationZ):
        """Move the rover base
    
        rotationZ specifies the amount to first rotate the rover base
        and then translationX is the amount to translate the rover base.
        the rotation argument is interpreted as radians per second and the
        translation argument is interpreted as meters per second.
    
        this method assumes that the publisher of the Twist message
        will not request a speed which is greater than zero but less
        than self.motorMinSpeed, but this assumption has not been validated.
        """

        if (rotationZ == 0 and translationX == 0):
            # FULL STOP
            self.motorControl.setVelocity(self.leftWheels, 0);
            self.motorControl.setVelocity(self.rightWheels, 0);
            return

        leftSpeed = -19.230 * rotationZ
        rightSpeed = 19.230 * rotationZ
 
        #
        # these magic numbers are derived from the motor speed and
        # wheel radius. they're based on a maximum translation
        # speed of 0.66 meters per second.
        #
        wheelSpeed = translationX / 0.0054

        leftSpeed += wheelSpeed
        rightSpeed += wheelSpeed

        if leftSpeed > self.motorMaxSpeed:
            leftSpeed = self.motorMaxSpeed
        if rightSpeed > self.motorMaxSpeed:
            rightSpeed = self.motorMaxSpeed
        if leftSpeed < -self.motorMaxSpeed:
            leftSpeed = -self.motorMaxSpeed
        if rightSpeed < -self.motorMaxSpeed:
            rightSpeed = self.motorMaxSpeed

        # adjust for difference in motor speeds
        leftSpeed *= self.leftAdjustment
        rightSpeed *= self.rightAdjustment

        try:
            self.motorControl.setVelocity(self.leftWheels, leftSpeed);
            self.motorControl.setVelocity(self.rightWheels, rightSpeed);

        except PhidgetException, e:
            rospy.logwarn('setVelocity() failed, left: %d, right: %d' % (
                leftSpeed,
                rightSpeed
                ))
            rospy.logwarn(" code: %d" % e.code)
            rospy.logwarn("message: %s" % e.message)
       
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
