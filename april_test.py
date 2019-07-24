import rospy
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
import numpy as np
import tf
from pyquaternion import Quaternion


rospy.init_node('listener', anonymous=True)
br = tf.TransformBroadcaster()
rate = rospy.Rate(10.0)


def april_tag_callback(data):
    global tag45_quaternion, tag45_position
    number_of_tags_detected = len(data.detections)
    if number_of_tags_detected > 0:
        for i in range(number_of_tags_detected):
            if data.detections[i].id[0] == 45:
                print(data.detections[i].id[0])
                odometry_tag = data.detections[i].pose
                tag45_position_tmp = np.asarray(
                    [odometry_tag.pose.pose.position.x, odometry_tag.pose.pose.position.y,
                     odometry_tag.pose.pose.position.z])
                print(tag45_position_tmp)
                tag45_quaternion_tmp = Quaternion(odometry_tag.pose.pose.orientation.w,
                                                  odometry_tag.pose.pose.orientation.x,
                                                  odometry_tag.pose.pose.orientation.y,
                                                  odometry_tag.pose.pose.orientation.z)
                trans = tag45_quaternion_tmp.transformation_matrix

                trans[0:3, 3] = tag45_position_tmp
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
