#!/usr/bin/env python
""" Watchdog action to kill fleet of robots.

Kills fleet by publishing to e_stop topics for each robot.

Author: Charlie Street
Owner: Charlie Street
"""

from . import ActionType
from std_msgs.msg import Bool
import rospy

class KillFleet(ActionType):
    name = "KillFleet"
    description = "Publishes a message to e_stop topic for all robots."
    config_keys = []

    def execute(self):
        """ Searches through topic list and publishes to all e_stop topics. """

        rospy.logwarn("Watchdog killing fleet")

        # TODO: Fill this in 
        e_stop_topics = []

        for topic in e_stop_topics:

            pub = rospy.Publisher(topic, Bool, latch=True)
            pub.publish(True)
