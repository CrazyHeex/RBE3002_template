#!/usr/bin/env python
import sys, math
import rospy
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose

import numpy as np

def get_neighbors(loc, my_map):
    """
        returns the legal neighbors of loc
        :param loc: tuple of location
        :return: list of tuples
    """
  

def is_valid_loc(loc, my_map):
    """
        Gets if a point is a legal location
        :param loc: tuple of location
        :return: boolean is a legal point
    """
    max_x = my_map.info.width*my_map.info.resolution + my_map.info.origin.position.x
    min_x = my_map.info.origin.position.x
    max_y = my_map.info.height*my_map.info.resolution + my_map.info.origin.position.y
    min_y = my_map.info.origin.position.y

    return (max_x >= loc.x >= min_x) and (max_y >= loc.y >= min_y)

def convert_location(loc, my_map):
    """converts points to the grid"""

    loc = Point()
    my_map = OccupancyGrid()

    np_map = np.reshape(my_map.data,(int(math.sqrt(len(my_map.data))), int(math.sqrt(len(my_map.data)))))
   

def world_to_map(x, y, my_map):
    """
        converts a point from the world to the map
        :param x: float of x position
        :param y: float of y position
        :return: tuple of converted point
    """

    return x - my_map.info.origin.position.x, y - my_map.info.origin.position.y

def map_to_world(x, y, my_map):
    """
        converts a point from the map to the world
        :param x: float of x position
        :param y: float of y position
        :return: tuple of converted point
    """

    return x + my_map.info.origin.position.x, y + my_map.info.origin.position.y


def to_cells(points, my_map):
    """
        Creates a GridCell() for Rviz distplay
        :param points: list of tuples
        :return: GridCell()
    """


def to_poses(points, my_map):
    """
        Creates a GridCell() for Rviz distplay
        :param points: list of tuples
        :return: GridCell()
    """


def index_to_point(point, my_map):
    """convert a point to a index"""

def point_to_index(location, my_map):
    """convert a index to a point"""
