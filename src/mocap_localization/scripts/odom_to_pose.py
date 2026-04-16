#!/usr/bin/env python3
"""Relay: nav_msgs/Odometry -> geometry_msgs/PoseStamped."""
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

pub = None


def cb(msg):
    ps = PoseStamped()
    ps.header = msg.header
    ps.pose = msg.pose.pose
    pub.publish(ps)


def main():
    global pub
    rospy.init_node('odom_to_pose')
    pub = rospy.Publisher('~pose_out', PoseStamped, queue_size=1)
    rospy.Subscriber('~odom_in', Odometry, cb)
    rospy.spin()


if __name__ == '__main__':
    main()
