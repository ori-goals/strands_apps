#!/usr/bin/env python
""" Watchdog action to kill fleet of robots.

Kills fleet by publishing to e_stop topics for each robot.

Author: Charlie Street
Owner: Charlie Street
"""

from watchdog_node.fleet_watchdog_utils import get_robot_ids
from . import ActionType
from std_msgs.msg import Bool
import rospy

class KillFleet(ActionType):
    name = "KillFleet"
    description = "Publishes a message to e_stop topic for all robots."
    config_keys = []

    def _get_e_stop_topics(self):
        """ Gets all topics with e_stop in the name.

        Gets all robot_ids and then creates the topic names from that.

        Returns:
            e_stop_topics: A list of e_stop topics.
        """
        
        e_stop_topics = []

        robot_ids = get_robot_ids()

        for robot in robot_ids:
            e_stop_topics.append('/'+str(robot)+'/e_stop')

        return e_stop_topics


    def execute(self):
        """ Searches through topic list and publishes to all e_stop topics. """

        rospy.logwarn("Watchdog killing fleet")

        e_stop_topics = self._get_e_stop_topics()

        for topic in e_stop_topics:
            # Publish to activate e_stop (value doesn't matter)
            pub = rospy.Publisher(topic, Bool, queue_size=5, latch=True)
            pub.publish(True)
