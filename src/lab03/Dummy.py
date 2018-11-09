#!/usr/bin/env python

import rospy, tf2_ros
from geometry_msgs.msg  import TransformStamped, Pose


class Dummy:
    def __init__(self):
        rospy.init_node('ZYang2_3002_dummy')
        self.rate = rospy.Rate(100)

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.nav_subscriber = rospy.Subscriber('/zyang/next_goal_dummy', TransformStamped, self.dummy)
        self.dummy_trans = None

        while not rospy.is_shutdown(): # start loop
            self.loop()
            self.rate.sleep()

    def dummy(self, msg):
        self.dummy_trans = msg

    def loop(self):
        try:
            self.dummy_trans.child_frame_id = 'dummy'
            self.dummy_trans.header.frame_id = 'odom'
            self.dummy_trans.header.stamp = rospy.Time.now()
            self.tf_broadcaster.sendTransform(self.dummy_trans)

        except Exception as e:
            print e
            self.rate.sleep()


if __name__ == '__main__':
    dummy = Dummy()
