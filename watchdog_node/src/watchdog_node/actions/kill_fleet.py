#!/usr/bin/env python
""" Watchdog action to kill fleet of robots.

Kills fleet by publishing to e_stop topics for each robot.

Author: Charlie Street
Owner: Charlie Street
"""

from . import ActionType
from std_msgs.msg import Bool
import rosgraph
import rospy

class KillFleet(ActionType):
    name = "KillFleet"
    description = "Publishes a message to e_stop topic for all robots."
    config_keys = []

    def _get_e_stop_topics(self):
        """ Gets all topics with e_stop in the name.

        The problem is that rospy.get_published_topics() only publishes
        the topics that have been published to. The /robot_i/e_stop topics 
        are subscribed to by /robot_i/twist_mux. Therefore, this code
        connects to the ros master and does what `rostopic list` does.

        Returns:
            e_stop_topics: A list of e_stop topics.
        """
        master = rosgraph.Master('/rostopic')
        state = master.getSystemState()

        pubs, subs, _ = state

        e_stop_topics = []

        # Topic is [topic_name, [pubs/subs]]
        for topic in pubs + subs:
            if 'e_stop' in topic[0] and topic[0] not in e_stop_topics:
                e_stop_topics.append(topic[0])

        return e_stop_topics


    def execute(self):
        """ Searches through topic list and publishes to all e_stop topics. """

        rospy.logwarn("Watchdog killing fleet")

        e_stop_topics = self._get_e_stop_topics()

        for topic in e_stop_topics:
            # Publish to activate e_stop (value doesn't matter)
            pub = rospy.Publisher(topic, Bool, queue_size=5, latch=True)
            pub.publish(True)
