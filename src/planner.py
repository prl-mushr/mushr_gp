#!/usr/bin/env python
import rospy
import pysbpl
import yaml

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class GlobalPlanner:

    def __init__(self, name):
        rospy.init_node(name, anonymous=True, disable_signals=True)

        start_topic = rospy.get_param("~start_topic")
        goal_topic = rospy.get_param("~goal_topic")
        config_path = rospy.get_param("~config_path")
        path_topic = rospy.get_param("~path_topic")

        self.start = None
        self.plan = None

        self.goal_subscriber = rospy.Subscriber(goal_topic, PoseStamped, self.plan, queue_size=1)
        self.start_subscriber = rospy.Subscriber(start_topic, PoseStamped, self.start, queue_size=1)
        self.path_publisher = rospy.Publisher(path_topic, Path, queue_size=1)


        config_file = open(config_path)
        self.params = yaml.load(config_file)

    def plan(self, msg):

        # add planning code

        self.publish_plan()

    def publish_plan(self):
        if self.plan != None:
            self.path_publisher.publish(plan)


if __name__ == "__main__":

    planner = GlobalPlanner("global_planner")
    # 3 Hz
    rate = rospy.Rate(3)

    while not rospy.is_shutdown():
        planner.publish_plan()
        rate.sleep()

    



