#!/usr/bin/env python

"""
PhidgetsEncoders() - Read the encoder values from a
	Phidgets 1047, filter the data a bit and then
	publish the Odometry message.

    Looking at the top of the encoder board, with the
    digital input terminals on the right and the
    encoder inputs on the top, encoder port number 0
    is at the top right and port 3 is at the top left.

"""

import roslib; roslib.load_manifest('nomad')
import rospy
from std_msgs.msg import Int16

from Phidgets.Devices.Encoder import Encoder
from Phidgets.PhidgetException import PhidgetException

class PhidgetEncoders:

    def __init__(
        self,
        leftEncoder,
        rightEncoder,
        leftSignAdjust,
        rightSignAdjust,
        rollover
        ):

        self.leftEncoderId = leftEncoder
        self.rightEncoderId = rightEncoder
        self.leftSignAdjust = leftSignAdjust
        self.rightSignAdjust = rightSignAdjust
        self.rollover = rollover

        self.leftEncoder = Int16(0)
        self.rightEncoder = Int16(0)

        self.encoder = Encoder()

        self.encoder.setOnAttachHandler(self.encoderAttached)
        self.encoder.setOnDetachHandler(self.encoderDetached)
        self.encoder.setOnErrorhandler(self.encoderError)
        self.encoder.setOnInputChangeHandler(self.encoderInputChanged)
        self.encoder.setOnPositionChangeHandler(self.encoderPositionChange)

        try:
            rospy.logdebug('openPhidget()')
            self.encoder.openPhidget()

        except PhidgetException, e:
            rospy.logerror("openPhidget() failed")
            rospy.logerror("code: %d" % e.code)
            rospy.logerror("message", e.message)

            raise

        try:
            rospy.logdebug('waitForAttach()')
            self.encoder.waitForAttach(10000)

        except PhidgetException, e:
            rospy.logerror("waitForAttach() failed")
            rospy.logerror("code: %d" % e.code)
            rospy.logerror("message", e.message)
    
            raise

        self.leftEncoderPublisher = rospy.Publisher('lwheel', Int16, queue_size = 10)
        self.rightEncoderPublisher = rospy.Publisher('rwheel', Int16, queue_size = 10)

        return

    def encoderPositionChange(self, e):
        """Called each time the encoder reports a change to its position."""

        return

    def updatePulseValues(self):

        leftPulses = (self.leftSignAdjust * self.encoder.getPosition(self.leftEncoderId))
        rightPulses = (self.rightSignAdjust * self.encoder.getPosition(self.rightEncoderId))


        rospy.logdebug('leftEncoder: %d' % (
            leftPulses
            ))
        rospy.logdebug('rightEncoder: %d' % (
            rightPulses
            ))

        #
        # actively manage the encoder value rollover. The encoder value,
        # when rolling forward, will start at 0 and move toward self.rollover.
        # When it exceeds self.rollover, it is reset to -self.rollover plus
        # the amount by which it exceeded self.rollover.
        #
        if leftPulses > self.rollover:
            rospy.logdebug('left rolled over forward')
            leftPulses = leftPulses - self.rollover
            self.encoder.setPosition(
                self.leftEncoderId,
                leftPulses
                )
        elif leftPulses < -(self.rollover):
            rospy.logdebug('left rolled over backward')
            leftPulses = leftPulses + self.rollover
            self.encoder.setPosition(
                self.leftEncoderId,
                leftPulses
                )
        if rightPulses > self.rollover:
            rospy.logdebug('right rolled over forward')
            rightPulses = rightPulses - self.rollover
            self.encoder.setPosition(
                self.rightEncoderId,
                rightPulses
                )
        elif rightPulses < -(self.rollover):
            rospy.logdebug('right rolled over backward')
            rightPulses = rightPulses + self.rollover
            self.encoder.setPosition(
                self.rightEncoderId,
                rightPulses
                )

        self.leftEncoder.data = leftPulses
        self.rightEncoder.data = rightPulses

        return

    def encoderAttached(self, e):
        rospy.loginfo('encoderAttached() called')

        self.encoder.setPosition(
                self.leftEncoderId,
                0
                )
        self.encoder.setPosition(
                self.rightEncoderId,
                0
                )
        self.encoder.setEnabled(
                self.leftEncoderId,
                True
                )
        self.encoder.setEnabled(
                self.rightEncoderId,
                True
                )

        return
    
    def encoderDetached(self, e):
        rospy.loginfo('encoderDetached() called')
        return
    
    def encoderError(self, e):
        rospy.loginfo('encoderError() called')
        return
    
    def encoderInputChanged(self, e):
        rospy.loginfo('encoderInputChanged() called')
        return
    
if __name__ == "__main__":
    rospy.init_node(
        name = 'diff_wheelEncoders',
        log_level = rospy.DEBUG
        )
    rospy.sleep(3.0)
    rospy.loginfo("Initializing diff_wheelEncoders.py")

    rightMotorDirectionSign = rospy.get_param("rightMotorDirectionSign", 1)
    leftMotorDirectionSign  = rospy.get_param("leftMotorDirectionSign", -1)
    leftMotorId             = rospy.get_param("leftMotorId", 0)
    rightMotorId            = rospy.get_param("rightMotorId", 1)
    encoderUpdateHz         = rospy.get_param("encoderUpdateHz", 10.0)
    encoderMax              = rospy.get_param("encoder_max", 65535)

    encoder = PhidgetEncoders(
        leftMotorId,
        rightMotorId,
        leftMotorDirectionSign,
        rightMotorDirectionSign,
        encoderMax
        )

    encoderUpdate = rospy.Rate(encoderUpdateHz)
    while not rospy.is_shutdown():
        encoder.updatePulseValues()

        encoder.leftEncoderPublisher.publish(encoder.leftEncoder)
        encoder.rightEncoderPublisher.publish(encoder.rightEncoder)

        encoderUpdate.sleep()

