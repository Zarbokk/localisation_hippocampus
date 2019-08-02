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

rospy.init_node('listener', anonymous=True)

rate = rospy.Rate(10.0)
pub = rospy.Publisher('realsens_position', Odometry, queue_size=1)

def april_tag_callback(data):
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

    else:
        return
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, april_tag_callback)

    rospy.spin()
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()


if __name__ == '__main__':
    listener()
