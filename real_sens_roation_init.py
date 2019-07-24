#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from tf.transformations import rotation_matrix, euler_from_quaternion, quaternion_matrix, quaternion_from_euler, \
    quaternion_inverse, quaternion_multiply
import numpy as np
import tf
from pyquaternion import Quaternion

first_run = True
tag45_quaternion = None
tag45_position = None

calibration_realsens_position = None
calibration_realsens_rotation = None
calibration_quaternion = None
calibration_position = None
rospy.init_node('listener', anonymous=True)
br = tf.TransformBroadcaster()
rate = rospy.Rate(10.0)


def real_sense_callback(data):
    global tag45_quaternion, first_run, tag45_position, calibration_position, calibration_quaternion, calibration_realsens_rotation, calibration_realsens_position
    if first_run:
        # calibrier_realsense
        # print(quaternion_tag45)
        # print(position_tag45)
        # quaternion_matrix(tag45_quaternion)
        calibration_realsens_rotation = Quaternion(data.pose.pose.orientation.w, data.pose.pose.orientation.x,
                                                   data.pose.pose.orientation.y, data.pose.pose.orientation.z)
        calibration_realsens_position = np.asarray([data.pose.pose.position.x, data.pose.pose.position.y,
                                                    data.pose.pose.position.z])
        first_run = False

    realsens_current_position = np.asarray([data.pose.pose.position.x, data.pose.pose.position.y,
                                            data.pose.pose.position.z])
    realsens_current_rotation = Quaternion(data.pose.pose.orientation.w, data.pose.pose.orientation.x,
                                           data.pose.pose.orientation.y, data.pose.pose.orientation.z)

    qx_180 = Quaternion(axis=[1, 0, 0], angle=np.pi)
    qx_90n = Quaternion(axis=[1, 0, 0], angle=-np.pi / 2)
    qx_90p = Quaternion(axis=[1, 0, 0], angle=np.pi / 2)
    qz_90p = Quaternion(axis=[0, 0, 1], angle=np.pi / 2)
    qy_90n = Quaternion(axis=[0, 1, 0], angle=-np.pi / 2)
    qy_90p = Quaternion(axis=[0, 1, 0], angle=np.pi / 2)
    qy_180 = Quaternion(axis=[0, 1, 0], angle=np.pi)
    S_k1_k2 = np.asarray([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
    fish_to_pose = np.asarray([-0, 0, 0])
    # realsens_position = calibration_position + calibration_quaternion.rotation_matrix.dot(
    #     fish_to_pose + S_k1_k2.dot(realsens_current_position - calibration_realsens_position))
    realsens_position = realsens_current_position - calibration_realsens_position
    realsens_orientation = (1
                            * calibration_realsens_rotation.inverse
                            * realsens_current_rotation
                            * qx_90n
                            * qy_90p
                            )

    br.sendTransform(realsens_position,
                     (realsens_orientation.x, realsens_orientation.y, realsens_orientation.z, realsens_orientation.w),
                     rospy.Time.now(),
                     "realsense",  # me
                     "camera_odom_frame")  # parent
    # rate.sleep()
    return


def listener():

    rospy.Subscriber('/camera/odom/sample', Odometry, real_sense_callback)

    rospy.spin()
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()


if __name__ == '__main__':
    listener()
