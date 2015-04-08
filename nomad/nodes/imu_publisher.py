#!/usr/bin/env python

import roslib ; roslib.load_manifest('nomad')
import rospy
from tf import transformations
import sys, serial
from sensor_msgs.msg import Imu
#from calc_covariance import calcCovariance
#roslib.load_manifest('rover')


imuMessage = Imu()

#
# the quaternion
#
imuMessage.orientation.x = 0.0
imuMessage.orientation.y = 0.0
imuMessage.orientation.z = 0.0
imuMessage.orientation.w = 0.0
#
# A covariance matrix containing all zeroes
# means that the covariance is unknown. If
# the first element is -1, that means this
# bit of the IMU data isn't available.
#
imuMessage.orientation_covariance = [
    0.15905861, -0.00651995, 0.02264925,
    -0.00651995, 0.13767703, 0.02274308,
    0.02264925, 0.02274308, 0.23587952
    ]

#
# radians per second
#
imuMessage.angular_velocity.x = 0.0
imuMessage.angular_velocity.y = 0.0
imuMessage.angular_velocity.z = 0.0
imuMessage.angular_velocity_covariance = [
    0.09708758, -0.0038554, -0.00666636,
    -0.0038554, 0.04836122, 0.00239955,
    -0.00666636, 0.00239955, 0.19192923
    ]

#
# meters per second squared
#
imuMessage.linear_acceleration.x = 0.0
imuMessage.linear_acceleration.y = 0.0
imuMessage.linear_acceleration.z = 0.0
imuMessage.linear_acceleration_covariance = [
    0.75524656, -0.26598849, 0.06740979,
    -0.26598849, 3.32580205, -0.75313099,
    0.06740979, -0.75313099, 0.50450472
    ]

imuMessage.header.frame_id = 'imu'

try:
    rospy.has_param('run_id')
    rospy.init_node('imu_publisher', log_level = rospy.INFO)

except:
    print 'Failed to initialize ROS node'
    sys.exit(1)

imuUpdateHz   = rospy.get_param('imuUpdateHz', 10.0);
imu_port      = rospy.get_param('imu_port', '/dev/ttyUSB0');
imu_baud_rate = rospy.get_param('imu_baud_rate', 38400);

publisher = rospy.Publisher('imu_data', Imu, queue_size = 10)

try:
    imu = serial.Serial(
        port = imu_port,
        baudrate = imu_baud_rate,
        timeout = 3
        )
    imu.setRTS(level = True)
    imu.setDTR(level = True)
except:
    print 'Failed to open serial port'
    sys.exit(1)

accelerometerUnitsPerG = 255
metersPerSecondSquaredPerG = 9.80665
radiansPerDegree = 0.0174533

#qCovariance, qSampleList, qSamples = calcCovariance(
#    None,
#    [0.0,
#     0.0,
#     0.0],
#    0,
#    1000
#    )
#aCovariance, aSampleList, aSamples = calcCovariance(
#    None,
#    [0.0,
#     0.0,
#     0.0],
#    0,
#    1000
#    )
#rCovariance, rSampleList, rSamples = calcCovariance(
#    None,
#    [0.0,
#     0.0,
#     0.0],
#    0,
#    1000
#    )
# counter = 0

rate = rospy.Rate(imuUpdateHz)
while not rospy.is_shutdown():
    #
    # Read the next row from the IMU
    #
    imuMessage.header.stamp = rospy.Time.now()

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

    #
    # !ANG:,21.19,2.80,-142.96,SEN:,-13.00,101.00,268.00,1.00,1.00,0.50,-173,130,-15,3.78,313,58175
    #
    # roll, pitch, yaw
    # accelerometer x, y, z
    # gyro x, y, z
    # magnetometer x, y, z
    # heading
    # barometer temperature, pressure
    #

    # Euler angles in degrees
    try:
        eulerRoll = float(imuFields[1])
        eulerPitch = float(imuFields[2])
        eulerYaw = float(imuFields[3])

    except:
        print 'Euler angle exception'
        print imuData
        print sys.exc_info()[0]
        print sys.exc_info()[1]
        print "<%s> <%s> <%s>" % (imuFields[1], imuFields[2], imuFields[3])

    # 
    try:
        xAccelerationRaw = float(imuFields[5])
        yAccelerationRaw = float(imuFields[6])
        zAccelerationRaw = float(imuFields[7])

    except:
        print 'Acceleration exception'
        print imuData
        print sys.exc_info()[0]
        print sys.exc_info()[1]
        print "<%s> <%s> <%s>" % (imuFields[5], imuFields[6], imuFields[7])

    try:
        xRotationRaw = float(imuFields[8])
        yRotationRaw = float(imuFields[9])
        zRotationRaw = float(imuFields[10])

    except:
        print 'Rotation exception'
        print imuData
        print sys.exc_info()[0]
        print sys.exc_info()[1]
        print "<%s> <%s> <%s>" % (imuFields[8], imuFields[9], imuFields[10])

    imuMessage.linear_acceleration.x = xAccelerationRaw / accelerometerUnitsPerG * metersPerSecondSquaredPerG
    imuMessage.linear_acceleration.y = yAccelerationRaw / accelerometerUnitsPerG * metersPerSecondSquaredPerG
    imuMessage.linear_acceleration.z = zAccelerationRaw / accelerometerUnitsPerG * metersPerSecondSquaredPerG

    imuMessage.angular_velocity.x = xRotationRaw * radiansPerDegree
    imuMessage.angular_velocity.y = yRotationRaw * radiansPerDegree
    imuMessage.angular_velocity.z = zRotationRaw * radiansPerDegree

    #
    # Calculate a quaternion from the Euler angles
    #

    imuMessage.orientation.x, imuMessage.orientation.y, imuMessage.orientation.z, imuMessage.orientation.w = transformations.quaternion_from_euler(eulerRoll, eulerPitch, eulerYaw)

#    qCovariance, qSampleList, qSamples = calcCovariance(
#        qSampleList,
#        [imuMessage.orientation.x,
#         imuMessage.orientation.y,
#         imuMessage.orientation.z],
#        qSamples,
#        1000
#        )
#    aCovariance, aSampleList, aSamples = calcCovariance(
#        aSampleList,
#        [imuMessage.linear_acceleration.x,
#         imuMessage.linear_acceleration.y,
#         imuMessage.linear_acceleration.z],
#        aSamples,
#        1000
#        )
#    rCovariance, rSampleList, rSamples = calcCovariance(
#        rSampleList,
#        [imuMessage.angular_velocity.x,
#         imuMessage.angular_velocity.y,
#         imuMessage.angular_velocity.z],
#        rSamples,
#        1000
#        )
#
#    counter = counter + 1
#    if (counter % 1000) == 0:
#        counter = 0
#        print "Covariance matrices"
#        print "Quaternion"
#        print qCovariance
#        print "Acceleration"
#        print aCovariance
#        print "Rotation"
#        print rCovariance

#    print imuMessage.orientation.x, imuMessage.orientation.y, imuMessage.orientation.z, imuMessage.orientation.w
    publisher.publish(imuMessage)

    rate.sleep()
