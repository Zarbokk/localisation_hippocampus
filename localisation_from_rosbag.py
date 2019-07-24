#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

# print(cv2.__version__)
import numpy as np
import time
# from std_msgs.msg import String
# from nav_msgs.msg import Odometry
# from apriltag_ros.msg import AprilTagDetectionArray,AprilTagDetection
x_list = list()
y_list = list()
z_list = list()


def callback_method(data):
    global x_list,y_list,z_list
    x_list.append(data.pose.pose.position.x)
    y_list.append(data.pose.pose.position.y)
    z_list.append(data.pose.pose.position.z)

def myhook():
    print("saving")
    np.savetxt("house_data.csv", [np.asarray(x_list, dtype=np.float32), np.asarray(y_list, dtype=np.float32),
                                  np.asarray(z_list, dtype=np.float32)], delimiter=",")
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("realsens_position", Odometry, callback_method)
    rospy.on_shutdown(myhook)
    rospy.spin()
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()


if __name__ == '__main__':
    listener()
