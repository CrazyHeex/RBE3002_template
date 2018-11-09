#!/usr/bin/env python

import rospy
import tf2_ros
import math

import numpy as np
from geometry_msgs.msg import TransformStamped, Point
from nav_msgs.msg import GridCells, OccupancyGrid


class MapHandler:
    def __init__(self):
        rospy.init_node('ZYang2_3002_map')
        self.rate = rospy.Rate(10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.nav_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.mod_map = rospy.Publisher('/mm', GridCells, queue_size=10)

        self.map = None
        self.map_dummy = GridCells()
        self.map_dummy.header.frame_id = 'odom'
        self.map_dummy.cell_height = 0.0
        self.map_dummy.cell_width = 0.0
        self.map_dummy.cells = []
        self.map_trans = TransformStamped()
        self.map_trans.transform.rotation.w = 1.0
        self.now_grid = [0, 0]
        self.goal_grid = [0, 0]
        self.h_matrix = None
        self.g_matrix = None

        while not rospy.is_shutdown():
            if self.map is not None:
                try:
                    self.map_trans.child_frame_id = 'odom'
                    self.map_trans.header.frame_id = 'map'
                    self.map_trans.header.stamp = rospy.Time.now()
                    self.tf_broadcaster.sendTransform(self.map_trans)

                    self.mod_map.publish(self.map_dummy)

                except Exception as e:
                    print e
                    self.rate.sleep()
                    continue

            try:
                self.update_now_grid()
                self.update_goal_grid()
            except Exception as e:
                print e

            self.rate.sleep()

    def map_callback(self, msg):
        self.map = msg
        self.map_trans.transform.translation = msg.info.origin.position
        self.map_trans.transform.rotation = msg.info.origin.orientation

        m = np.array(msg.data)
        m = np.reshape(m, (int(math.sqrt(len(msg.data))), int(math.sqrt(len(msg.data)))))

        self.map_dummy.header.frame_id = 'odom'
        self.map_dummy.cell_height = msg.info.resolution
        self.map_dummy.cell_width = msg.info.resolution

        self.map_dummy.cells = []

        for h in range(msg.info.height):
            for w in range(msg.info.width):
                if m[h][w] == 0:
                    p = Point()
                    p.x = w * msg.info.resolution + msg.info.resolution / 2
                    p.y = h * msg.info.resolution + msg.info.resolution / 2
                    p.z = 0.0
                    self.map_dummy.cells.append(p)

    def update_now_grid(self):
        trans = self.tf_buffer.lookup_transform('odom', 'base_footprint', rospy.Time())
        self.now_grid = [int(trans.transform.translation.x / self.map.info.resolution),
                         int(trans.transform.translation.y / self.map.info.resolution)]

    def update_goal_grid(self):
        trans = self.tf_buffer.lookup_transform('odom', 'nav_goal', rospy.Time())
        self.goal_grid = [int(trans.transform.translation.x / self.map.info.resolution),
                          int(trans.transform.translation.y / self.map.info.resolution)]
        print self.goal_grid


if __name__ == '__main__':
    dummy = MapHandler()
