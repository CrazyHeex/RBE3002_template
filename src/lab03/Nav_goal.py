#!/usr/bin/env python

import rospy, tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped


class NGServer:
    def __init__(self):
        rospy.init_node('ZYang2_3002_nav_goal')
        self.rate = rospy.Rate(1000)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.nav_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.nav_to_pose)
        self.goal = None
        while not rospy.is_shutdown():
            self.loop()

    def nav_to_pose(self, nav_goal):
        self.goal = nav_goal

    def loop(self):
        try:
            trans = TransformStamped()
            trans.child_frame_id = 'nav_goal'
            trans.header.frame_id = 'odom'
            trans.header.stamp = rospy.Time.now()
            trans.transform.translation = self.goal.pose.position
            trans.transform.rotation = self.goal.pose.orientation

            self.tf_broadcaster.sendTransform(trans)
        except Exception as e:
            pass


if __name__ == '__main__':
    ng = NGServer()
