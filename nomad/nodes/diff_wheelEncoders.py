#!/usr/bin/env python

"""
diff_wheelEncoders.py
Bill Mania <bill@manialabs.us>

Read encoder values from the Arduino encoder controller
and publish them.

"""

import roslib; roslib.load_manifest('nomad')
import rospy
import serial, sys, exceptions
from std_msgs.msg import Int32

class Encoders:
    """
    Encoders: read the incoming encoder data and publish to separate
    topics

    open the serial port
    read the values as fast as they arrive and
     publish the updates

    """

    def __init__(
        self,
        encoderPort,
        encoderDataRate,
        leftSignAdjust,
        rightSignAdjust,
        rollover
        ):

        self.device = encoderPort
        self.dataRate = encoderDataRate
        self.leftSignAdjust = leftSignAdjust
        self.rightSignAdjust = rightSignAdjust
        self.rollover = rollover

        self.leftEncoder = Int32(0)
        self.rightEncoder = Int32(0)

        self.encoderDevice = None

        try:
            self.encoderDevice = serial.Serial(
                port = self.device,
                baudrate = self.dataRate,
                timeout = 1
                )
            self.encoderDevice.setRTS(level = True)
            self.encoderDevice.setDTR(level = True)

        except serial.SerialException, e:
            self.encoderDevice = None
            rospy.logerr("Encoder initialization failed")
            rospy.logerr("code: %d" % e.code)
            rospy.logerr("message: ", e.message)

            rospy.signal_shutdown('Failed to initialize encoders')
            return

        #
        # Align at the beginning of a complete message
        #
        characterRead = ''
        while True:
            characterRead = self.encoderDevice.read(size = 1)
            if characterRead == '\n':
                break
        rospy.logdebug('Messages aligned')

        self.leftEncoderPublisher = rospy.Publisher('lwheel', Int32, queue_size = 1)
        self.rightEncoderPublisher = rospy.Publisher('rwheel', Int32, queue_size = 1)

        return

    def publishPulseValues(self):

        try:
            self.leftEncoderPublisher.publish(self.leftEncoder)
            self.rightEncoderPublisher.publish(self.rightEncoder)

        except exceptions.NameError, e:
            rospy.logerr("NameError while publishing, %s" % (
                e
                ))
            rospy.logerr("leftEncoder: %d" % self.leftEncoder.data)
            rospy.logerr("righttEncoder: %d" % self.rightEncoder.data)

        return

    def updatePulseValues(self):
        """updatePulseValues(): read the most recent encoder values

        """

        if not self.encoderDevice:
            return

        characterRead = ''
        encoderData = ''
        while True:
            characterRead = self.encoderDevice.read(size = 1)
            if len(characterRead) < 1:
                return
    
            if characterRead == '\n':
                break
    
            encoderData = encoderData + characterRead
            continue
    
        rospy.logdebug('Raw encoder message %s ' % encoderData)
        encoderFields = encoderData.split(',')
        if len(encoderFields) < 4:
            rospy.logwarn('Failed to parse raw Encoder message')
            return

        timestamp = int(encoderFields[0])
        leftPulses = (self.leftSignAdjust * int(encoderFields[1]))
        rightPulses = (self.rightSignAdjust * int(encoderFields[2]))
        encoderVersion = encoderFields[3]
        self.leftEncoder.data = leftPulses
        self.rightEncoder.data = rightPulses

        rospy.logdebug('updatePulseValues - time: %d, left: %d, right: %d, version: %s' % (
            timestamp,
            self.leftEncoder.data,
            self.rightEncoder.data,
            encoderVersion
            ))

        return

if __name__ == "__main__":
    rospy.init_node(
        name = 'diff_wheelEncoders',
        log_level = rospy.DEBUG
        )
    rospy.sleep(3.0)
    rospy.loginfo("Starting diff_wheelEncoders.py")

    encoderPort             = rospy.get_param("encoderPort", '/dev/ttyACM0')
    encoderDataRate         = rospy.get_param("encoderDataRate", 9600)
    rightMotorDirectionSign = rospy.get_param("rightMotorDirectionSign", 1)
    leftMotorDirectionSign  = rospy.get_param("leftMotorDirectionSign", 1)
    encoderMax              = rospy.get_param("encoder_max", 65535)
    rospy.logdebug('Params: %s, %d, %d, %d, %d' % (
        encoderPort,
        encoderDataRate,
        rightMotorDirectionSign,
        leftMotorDirectionSign,
        encoderMax
        ))

    encoder = Encoders(
        encoderPort,
        encoderDataRate,
        leftMotorDirectionSign,
        rightMotorDirectionSign,
        encoderMax
        )

    rospy.loginfo('diff_wheelEncoders initialized')

    while not rospy.is_shutdown():
        encoder.updatePulseValues()

        encoder.publishPulseValues()
