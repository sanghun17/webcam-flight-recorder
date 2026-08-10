#!/usr/bin/env python3
"""Hide a local-frame odometry stream until its global anchor is available.

The message payload is forwarded unchanged after the first anchor message.  No
pre-anchor sample is cached or replayed, so RViz never sees the temporary local
origin used while FAST-LIVO estimates its OptiTrack alignment.
"""

import math

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


class OdomAfterAnchorRelay:
    def __init__(self):
        self._aligned = False
        self._anchor_position = None
        input_topic = rospy.get_param("~input_topic")
        output_topic = rospy.get_param("~output_topic")
        anchor_topic = rospy.get_param("~anchor_topic")
        queue_size = int(rospy.get_param("~queue_size", 1000))
        history_topic = rospy.get_param("~history_topic", "")
        self._alignment_distance_threshold = float(
            rospy.get_param("~alignment_distance_threshold", 0.5)
        )
        self._history_min_distance = float(
            rospy.get_param("~history_min_distance", 0.02)
        )
        self._history_max_period = float(
            rospy.get_param("~history_max_period", 0.10)
        )
        self._history_publish_period = float(
            rospy.get_param("~history_publish_period", 0.10)
        )
        self._history_width = float(rospy.get_param("~history_width", 0.025))
        if self._alignment_distance_threshold <= 0:
            raise ValueError("alignment distance threshold must be positive")
        if self._history_min_distance <= 0:
            raise ValueError("history minimum distance must be positive")
        if self._history_max_period <= 0 or self._history_publish_period <= 0:
            raise ValueError("history max/publish periods must be positive")
        if self._history_width <= 0:
            raise ValueError("history width must be positive")

        self._history_publisher = None
        self._history_points = []
        self._last_sample_stamp = None
        self._last_sample_position = None
        self._last_publish_stamp = None
        if history_topic:
            self._history_publisher = rospy.Publisher(
                history_topic, MarkerArray, queue_size=10, latch=True
            )

        self._publisher = rospy.Publisher(
            output_topic, Odometry, queue_size=queue_size
        )
        self._anchor_subscriber = rospy.Subscriber(
            anchor_topic, Odometry, self._anchor_callback, queue_size=1
        )
        self._input_subscriber = rospy.Subscriber(
            input_topic, Odometry, self._input_callback, queue_size=queue_size
        )
        rospy.loginfo(
            "FAST-LIVO visualization gate: suppressing %s until first %s; "
            "then forwarding unchanged to %s",
            input_topic,
            anchor_topic,
            output_topic,
        )
        if self._history_publisher is not None:
            rospy.loginfo(
                "FAST-LIVO aligned history: %s (point at %.3fm or %.3fs, "
                "publish %.3fs)",
                history_topic,
                self._history_min_distance,
                self._history_max_period,
                self._history_publish_period,
            )

    def _anchor_callback(self, message):
        if self._aligned:
            return
        position = message.pose.pose.position
        self._anchor_position = (position.x, position.y, position.z)

    def _input_callback(self, message):
        if not self._aligned:
            if self._anchor_position is None:
                return
            position = message.pose.pose.position
            distance = math.sqrt(
                (position.x - self._anchor_position[0]) ** 2
                + (position.y - self._anchor_position[1]) ** 2
                + (position.z - self._anchor_position[2]) ** 2
            )
            # The anchor and high-rate odometry use independent ROS subscriber
            # queues. During accelerated replay, pre-anchor samples can therefore
            # arrive after the first anchor callback. Open only when the odometry
            # itself agrees with the global anchor; this prevents a local-origin
            # point from ever entering either the arrow or the cumulative trail.
            if distance > self._alignment_distance_threshold:
                return
            self._aligned = True
            rospy.loginfo(
                "FAST-LIVO visualization gate opened (anchor distance %.3fm)",
                distance,
            )
        self._publisher.publish(message)
        self._update_history(message)

    def _update_history(self, message):
        if self._history_publisher is None:
            return
        stamp = message.header.stamp.to_sec()
        if self._last_sample_stamp is not None and stamp < self._last_sample_stamp:
            self._history_points = []
            self._last_sample_stamp = None
            self._last_sample_position = None
            self._last_publish_stamp = None

        position = message.pose.pose.position
        current_position = (position.x, position.y, position.z)
        if self._last_sample_stamp is not None:
            elapsed = stamp - self._last_sample_stamp
            moved = math.sqrt(
                sum(
                    (current_position[i] - self._last_sample_position[i]) ** 2
                    for i in range(3)
                )
            )
            if moved < self._history_min_distance and elapsed < self._history_max_period:
                return
        self._history_points.append(Point(position.x, position.y, position.z))
        self._last_sample_stamp = stamp
        self._last_sample_position = current_position
        if (
            self._last_publish_stamp is not None
            and stamp - self._last_publish_stamp < self._history_publish_period
        ):
            return

        marker = Marker()
        marker.header = message.header
        marker.ns = "fast_livo_history"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self._history_width
        # Sienna brown (#A0522D), distinct from the cyan GT history.
        marker.color.r = 160.0 / 255.0
        marker.color.g = 82.0 / 255.0
        marker.color.b = 45.0 / 255.0
        marker.color.a = 0.95
        marker.points = list(self._history_points)
        self._history_publisher.publish(MarkerArray(markers=[marker]))
        self._last_publish_stamp = stamp


if __name__ == "__main__":
    rospy.init_node("fastlivo_visualization_alignment_gate")
    OdomAfterAnchorRelay()
    rospy.spin()
