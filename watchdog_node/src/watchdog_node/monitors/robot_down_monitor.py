#!/usr/bin/env python
""" Monitors for detecting down time on topics across all robots.

E.g. monitor triggers if one of the /robot_i/front/scan topics doesn't publish
for a second.

Author: Charlie Street
Owner: Charlie Street
"""

from watchdog_node.fleet_watchdog_utils import get_robot_ids
from . import MonitorType
from threading import Timer
import rostopic
import signal
import rospy
import sys

class RobotDown(MonitorType):
    name = "RobotDown"
    description = ("This monitor detects if there are no messages on "
                   " a topic for at least one robot for a given period")
    config_keys = [('topic', "The topic to monitor, will be prepended with"
                             "/robot_i/ namespace"),
                   ('max_duration', "The maximum number of seconds to accept "
                                    "not receiving a message")]
                    

    def __init__(self, monitor_config, invalid_cb):
        super(RobotDown, self).__init__(monitor_config, invalid_cb)
        self.max_duration = rospy.Duration(self.max_duration)

        signal.signal(signal.SIGINT, self._signal_handler)

        self.robot_ids = []
        while len(self.robot_ids) != self.num_robots:
            self.robot_ids = get_robot_ids()
            rospy.loginfo('Detected ' + str(len(self.robot_ids)) + 
                          ' robots: ' + str(self.robot_ids) + '...')
            rospy.sleep(1)

        self.topics = {}
        for robot in self.robot_ids:
            self.topics[robot] = '/' + robot
            self.topics[robot] += '/' if self.topic[0] != '/' else ''
            self.topics[robot] += self.topic
        
        rospy.loginfo("Listening to each robot's " + str(self.topic) + 
                      " topic...")

    def _signal_handler(self, sig, frame):
        if sig == signal.SIGINT:
            rospy.logwarn("SIGINT received, shutting down...")
            sys.exit(0)

    def _init_robot(self, robot):
        """ Initialises single robot. """
        self._validity_timers[robot] = Timer(self.max_duration.to_sec(), 
                                             lambda: self.timeout_cb(robot))
        self._last_times[robot] = None
        self._topic_subs[robot] = None
        self._retry_timers[robot] = None

        MsgClass, topic, _ = rostopic.get_topic_class(self.topics[robot])

        if topic is not None:
            self._topic_subs[robot] = rospy.Subscriber(topic, 
                                                       MsgClass, 
                                          lambda m: self.topic_cb(robot, m))
            self._last_times[robot] = rospy.Time.now()
            self._validity_timers[robot].start()
        else:
            rospy.logwarn("Could not determine type of '{}' topic. Retry in"
                          " 10s.".format(self.topics[robot]))
            self._retry_timers[robot] = Timer(10, 
                                              lambda: self._init_robot(robot))
            self._retry_timers[robot].start()


    def start(self):
        self._validity_timers = {}
        self._last_times = {}
        self._topic_subs = {}
        self._retry_timers = {}

        for robot in self.robot_ids:
            self._init_robot(robot)
        
        rospy.loginfo('Starting RobotDown monitor on ' + str(self.topic) + 
                      ' topic for each robot...')
        

    def stop(self):
        for robot in self.robot_ids:
            if (robot in self._topic_subs and 
                self._topic_subs[robot] is not None):
                self._topic_subs[robot].unregister()
            if (robot in self._retry_timers and 
                self._retry_timers[robot] is not None):
                self._retry_timers[robot].cancel()
            if (robot in self._validity_timers and 
                self._validity_timers[robot] is not None):
                self._validity_timers[robot].cancel()


    def topic_cb(self, robot_id, msg):
        self._last_times[robot_id] = rospy.Time.now()

    
    def timeout_cb(self, robot):
        duration = rospy.Time.now() - self._last_times[robot]
        if duration > self.max_duration:
            rospy.logwarn("No message received on " + str(self.topics[robot]) +
                          " for " + str(duration.to_sec()) + "s.")
            self.set_invalid()
        else:
            time_left = (self.max_duration - duration).to_sec()
            self._validity_timers[robot] = Timer(time_left, 
                                                 lambda: self.timeout_cb(robot))
            self._validity_timers[robot].start()