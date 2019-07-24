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
pub = rospy.Publisher('realsens_position', Odometry, queue_size=1)

def april_tag_callback(data):
    global tag45_quaternion, tag45_position
    number_of_tags_detected = len(data.detections)
    if number_of_tags_detected > 0:
        for i in range(number_of_tags_detected):
            if data.detections[i].id[0] == 45:
                odometry_tag = data.detections[i].pose
                # print(odometry_tag.pose.pose.orientation)
                tag45_position_tmp = np.asarray(
                    [odometry_tag.pose.pose.position.x, odometry_tag.pose.pose.position.y,
                     odometry_tag.pose.pose.position.z])

                tag45_quaternion_tmp = Quaternion(odometry_tag.pose.pose.orientation.w,
                                                  odometry_tag.pose.pose.orientation.x,
                                                  odometry_tag.pose.pose.orientation.y,
                                                  odometry_tag.pose.pose.orientation.z)
                # print(np.asarray(angles)/2/np.pi*360)
                # print(odometry_tag.pose.pose.position)
                trans = tag45_quaternion_tmp.transformation_matrix

                trans[0:3, 3] = tag45_position_tmp
                # print(trans)
                # trans = np.linalg.inv(trans)
                tag45_quaternion = Quaternion(matrix=np.asarray(trans[0:3, 0:3]))
                tag45_position = trans[0:3, 3]
                trans = np.linalg.inv(trans)
                tag45_quaternion_tmp = Quaternion(matrix=np.asarray(trans[0:3, 0:3]))
                tag45_position_tmp = trans[0:3, 3]
                br.sendTransform(tag45_position_tmp,
                                 (tag45_quaternion_tmp.x, tag45_quaternion_tmp.y, tag45_quaternion_tmp.z,
                                  tag45_quaternion_tmp.w),
                                 rospy.Time.now(),
                                 "camera_fish1",  # me
                                 "tag_45")  # parent
                rate.sleep()
    else:
        return
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def real_sense_callback(data):
    global tag45_quaternion, first_run, tag45_position, calibration_position, calibration_quaternion, calibration_realsens_rotation, calibration_realsens_position,pub
    # print("X:", data.pose.pose.position.x)
    # print("Y:", data.pose.pose.position.y)
    # print("Z:", data.pose.pose.position.z)
    if first_run:
        # calibrier_realsense
        if tag45_quaternion is None:
            print("not_initialized")
            return
        else:
            # print(quaternion_tag45)
            # print(position_tag45)
            # quaternion_matrix(tag45_quaternion)
            # print(tag45_quaternion)
            trans = tag45_quaternion.transformation_matrix

            trans[0:3, 3] = tag45_position
            trans = np.linalg.inv(trans)
            calibration_position = trans[0:3, 3]
            calibration_quaternion = Quaternion(matrix=np.asarray(trans[0:3, 0:3]))
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
    fish_to_pose = np.asarray([0.00, 0, 0])
    realsens_position = calibration_position + calibration_quaternion.rotation_matrix.dot(
        fish_to_pose + S_k1_k2.dot(realsens_current_position - calibration_realsens_position))
    # print(realsens_position.shape)
    # realsens_orientation = (qy_90n * qx_90n * realsens_current_rotation * qx_90n * qy_90n)
    # print("real",np.linalg.det( realsens_current_rotation.rotation_matrix))
    # print("sk1",np.linalg.det(S_k1_k2))
    # print("calibr",np.linalg.det(calibration_quaternion.rotation_matrix))
    # print(np.linalg.det(np.matmul(np.matmul(calibration_quaternion.rotation_matrix,S_k1_k2),realsens_current_rotation.rotation_matrix)))
    # realsens_orientation = Quaternion(matrix=np.matmul(np.matmul(calibration_quaternion.rotation_matrix,S_k1_k2),(calibration_realsens_rotation.inverse*realsens_current_rotation).rotation_matrix))
    realsens_orientation = (calibration_quaternion
                            * qx_90p
                            * qz_90p
                            * calibration_realsens_rotation.inverse
                            * realsens_current_rotation
                            * qx_90n
                            * qy_90p
                            # * calibration_quaternion.inverse
                            )

    br.sendTransform(realsens_position,
                     (realsens_orientation.x, realsens_orientation.y, realsens_orientation.z, realsens_orientation.w),
                     rospy.Time.now(),
                     "realsense",  # me
                     "tag_45")  # parent
    message = Odometry()
    message.header.stamp = rospy.Time.now()
    message.pose.pose.orientation.x = realsens_orientation.x
    message.pose.pose.orientation.y = realsens_orientation.y
    message.pose.pose.orientation.z = realsens_orientation.z
    message.pose.pose.orientation.w = realsens_orientation.w
    message.pose.pose.position.x = realsens_position[0]
    message.pose.pose.position.y = realsens_position[1]
    message.pose.pose.position.z = realsens_position[2]
    pub.publish(message)
    # rate.sleep()
    return


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, april_tag_callback)
    rospy.Subscriber('/camera/odom/sample', Odometry, real_sense_callback)

    rospy.spin()
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()


if __name__ == '__main__':
    listener()
