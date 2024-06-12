#!/usr/bin/env python

import time
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Float64, Bool
import dynamic_reconfigure.server
import roslib
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest
from actionlib_msgs.msg import GoalStatusArray
import os

roslib.load_manifest('mirte_clean_robot')

from mirte_clean_robot.cfg import StopForObstaclesConfig


class ObstacleDetector:
    def __init__(self, left_topic, right_topic, min_range, max_range, weight, publish_topic, distance_stop):
        self.min_range = min_range
        self.max_range = max_range
        self.weight = weight
        self.left_avg = 0
        self.right_avg = 0
        self.distance_stop = distance_stop
        self.i = 0
        self.stop_time = None

        self.is_paused = False

        rospy.init_node("stop_for_obstacles")
        rospy.loginfo("Initialized stop_for_obstacles")
        rospy.Subscriber(left_topic, Range, self.range_left_callback)
        rospy.Subscriber(right_topic, Range, self.range_right_callback)
        self.publisher = rospy.Publisher(publish_topic, Bool, queue_size=10)
        self.publisher.publish(False)

        rospy.wait_for_message("/sound_play/status", GoalStatusArray)
        rospy.sleep(5)

        
        self.server = dynamic_reconfigure.server.Server(StopForObstaclesConfig, self.config_callback)

        self.soundclient = SoundClient()
        self.soundclient.play(SoundRequest.NEEDS_UNPLUGGING_BADLY, blocking=False)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def config_callback(self, config, level):
        self.distance_stop = config.distance_stop
        rospy.loginfo(f"Sonar new stop distance: {self.distance_stop}")
        return config

    def range_left_callback(self, data: Range):
        clamped_range = max(self.min_range, min(self.max_range, data.range))
        self.left_avg = (clamped_range * self.weight) + (self.left_avg * (1 - self.weight))

    def range_right_callback(self, data: Range):
        clamped_range = max(self.min_range, min(self.max_range, data.range))
        self.right_avg = (clamped_range * self.weight) + (self.right_avg * (1 - self.weight))

    def get_average(self):
        lowest = min(self.left_avg, self.right_avg)
        highest = max(self.left_avg, self.right_avg)
        return (lowest * 2/3) + (highest * 1/3)

    def timer_callback(self, event):
        # Check distance from obstacle
        average = self.get_average()

        self.i += 1
        if self.i % 100 == 0:
            rospy.loginfo(f"Sonar average: {average}")
            self.i = 0

        if average < self.distance_stop:
            if not self.is_paused:
                self.pause_carrot(average)
            elif time.time() - self.stop_time >= 15:
                self.unpause_carrot(average)
            if (time.time() - self.stop_time) % 5:
                self.soundclient.play(SoundRequest.NEEDS_UNPLUGGING_BADLY, blocking=False)
        else:
            if self.is_paused:
                self.unpause_carrot(average)
    
    def pause_carrot(self, average):
        self.stop_time = time.time()
        self.publisher.publish(True)
        self.is_paused = True
        rospy.loginfo(f"Sonar average: {average}")
        rospy.loginfo("Sonar triggerd: Pausing Robot")
        self.soundclient.play(SoundRequest.NEEDS_UNPLUGGING_BADLY, blocking=False)
    
    def unpause_carrot(self, average):
        self.publisher.publish(False)
        self.is_paused = False
        rospy.loginfo(f"Sonar average: {average}")
        rospy.loginfo("Sonar untriggerd: Unpausing Robot")
        self.soundclient.playWave(os.join(os.path.dirname(os.path.realpath(__file__)), 'cow-nature.wav'))


    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    # Hyperparams
    distance_stop = 0.25

    detector = ObstacleDetector("/mirte/distance/left_front", "/mirte/distance/right_front", 0.1, 1.5, 0.33, "/pause", distance_stop)
    detector.spin()
