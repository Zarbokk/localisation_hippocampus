import numpy as np

import rospy
import tf
from gantry_msgs.msg import Gantry
from geometry_msgs.msg import PoseStamped


def callback(msg, publisher):
    """"""

    publish_pose = PoseStamped()
    publish_pose.header = msg.header
    publish_pose.pose.position.x = float(msg.position.x)
    publish_pose.pose.position.y = float(msg.position.y)
    publish_pose.pose.position.z = float(msg.position.z)
    publisher.publish(publish_pose)
    # add spheres to rviz

    # print pose_array


def main():
    rospy.init_node('gantry_to_rviz')
    publisher_pose = rospy.Publisher('gantry_point_stamped', PoseStamped, queue_size=1)
    rospy.Subscriber("/gantry_position", Gantry, callback, publisher_pose, queue_size=1)

    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    main()
