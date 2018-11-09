#!/usr/bin/env python

import rospy, tf2_ros, math

import numpy as np

from geometry_msgs.msg  import TransformStamped, PoseStamped,Point
from  nav_msgs.msg import GridCells, OccupancyGrid




class Map_handler:
    def __init__(self):
        rospy.init_node('ZYang2_3002_map')
        self.rate = rospy.Rate(100)

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.nav_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map)
        self.nav_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.nav_goal)
        self.mod_map = rospy.Publisher('/mm', GridCells, queue_size=10)

        self.map = None
        self.map_dummy = GridCells()
        self.map_dummy.header.frame_id = 'odom'
        self.map_dummy.cell_height = 0.0
        self.map_dummy.cell_width = 0.0
        self.map_dummy.cells = []
        self.map_trans = TransformStamped()
        self.map_trans.transform.rotation.w = 1.0

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

            self.rate.sleep()






    def map(self, msg):
        self.map = msg
        self.map_trans.transform.translation = msg.info.origin.position
        self.map_trans.transform.rotation = msg.info.origin.orientation

        m = np.array(msg.data)
        m = np.reshape(m,(int(math.sqrt(len(msg.data))), int(math.sqrt(len(msg.data)))))
        print m

        self.map_dummy.header.frame_id = 'odom'
        self.map_dummy.cell_height = msg.info.resolution
        self.map_dummy.cell_width = msg.info.resolution

        self.map_dummy.cells = []

        for h in range(msg.info.height):
            for w in range(msg.info.width):
                if  m[h][w] == 0:
                    p = Point()
                    p.x = w * msg.info.resolution + msg.info.resolution/2
                    p.y = h * msg.info.resolution + msg.info.resolution/2
                    p.z = 0.0
                    self.map_dummy.cells.append(p)




    def nav_goal(self, msg):
        # cells = []
        # self.map_dummy.header.stamp = rospy.Time.now()
        # try:
        #     # trans = self.tf_buffer.lookup_transform('nav_goal', 'map', rospy.Time.now())
        #
        #     p = Point()
        #     p.x = msg.pose.position.x
        #     p.y = msg.pose.position.y
        #     p.z = 0.0
        #     cells.append(p)
        #     print cells,'<<<<<<<<<<<<<<<<'
        #     self.map_dummy.cells = cells
        #
        # except Exception as e:
        #     print 'nav_goal: ', e
        # # self.dummy_trans = msg
        pass


    def update_heuristic(self):
        pass

    def update_cost(self):
        pass


if __name__ == '__main__':
    dummy = Map_handler()
