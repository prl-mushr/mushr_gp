#!/usr/bin/env python

"""
Global Planner for Mushr Navigation.

Author: Doerr, Schiffer
"""

import numpy as np
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from pysbpl import planner


class GlobalPlanner:
    """ROS Wrapper for pysbpl Planner."""

    def __init__(self, name):
        """
        Initialize global planner.

        Attributes:
            name (string) rosnode name

        """
        rospy.init_node(name, anonymous=True)

        self.start = None
        self.path = None
        self.map = Map()

        # initialize subscribers
        self.goal_sub = rospy.Subscriber(
            rospy.get_param("~goal_topic"), PoseStamped, self.goal_cb, queue_size=1
        )
        self.start_sub = rospy.Subscriber(
            rospy.get_param("~start_topic"),
            PoseStamped,
            self.set_start_cb,
            queue_size=1,
        )
        self.map_sub = rospy.Subscriber(
            rospy.get_param("~map"), OccupancyGrid, self.set_map_cb, queue_size=1
        )

        # initialize publisher
        self.path_publisher = rospy.Publisher(
            rospy.get_param("~path_topic"), Path, queue_size=1
        )

        config_file = open(rospy.get_param("~config_path"))
        self.params = yaml.load(config_file)

        rate = rospy.Rate(3)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.path is not None:
                self.path_publisher.publish(self.path)

    def goal_cb(self, msg):
        """
        Plan a path from recorded start to new goal.

        Attributes:
            msg (geometry_msgs/PoseStamped): goal position

        """
        self.params["map"] = self.map
        self.params["perimeter"] = [
            [-0.2, -0.235],
            [0.2, -0.235],
            [0.2, 0.235],
            [-0.2, 0.235],
        ]

        mprim_path = rospy.get_param("~mprim_path")
        self.params["mprim_path"] = mprim_path.encode("utf-8")
        goal = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

        print("Planning with pysbpl")
        print("Start: " + str(self.start))
        print("Goal: " + str(goal))

        sbpl_planner = planner.Planner(**self.params)

        path = sbpl_planner.plan(self.start, goal, allocated_time=10)

        print("Path created: " + path)

    def set_start_cb(self, msg):
        """
        Record the new car position as the start.

        Attributes:
            msg (geometry_msgs/PoseStamped): goal position

        """
        self.start = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def set_map_cb(self, msg):
        """
        Load and save the map.

        Attributes:
            msg (nav_msgs/OccupancyGrid): map representation

        """
        self.map.load_map(msg)


class Map:
    """Costmap Representation."""

    def load_map(self, occ_grid):
        """
        Transform occuppancy grid into costmap.

        Attributes:
            occ_grid (nav_msgs/OccupancyGrid): occupancy grid to convert to costmap

        """
        self.width = occ_grid.info.width
        self.height = occ_grid.info.height

        # TODO: make a way not to hardcode this
        self.resolution = 0.050000  # occ_grid.info.resolution
        self.origin = [
            occ_grid.info.origin.position.x,
            occ_grid.info.origin.position.y,
            occ_grid.info.origin.position.z,
        ]
        self.occupied_thresh = 0.65
        self.free_thresh = 0.196

        # reformat map into 2D array with values [0, 1]
        self.img = np.reshape(occ_grid.data, (self.height, self.width))
        self.img /= 100
        # uncertain values are set to -1, map that to 1
        self.img[self.img == -1] = 1


if __name__ == "__main__":
    global_planner = GlobalPlanner("global_planner")
