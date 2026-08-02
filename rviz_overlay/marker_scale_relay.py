#!/usr/bin/env python3
"""Scale replay-only uncertainty ellipsoids without changing planner data."""

import math

import rospy
from visualization_msgs.msg import Marker, MarkerArray


def scale_uncertainty_ellipsoids(message, scale):
    """Scale SPHERE marker diameters in-place; DELETE markers remain untouched."""
    for marker in message.markers:
        if marker.action != Marker.ADD or marker.type != Marker.SPHERE:
            continue
        marker.scale.x *= scale
        marker.scale.y *= scale
        marker.scale.z *= scale
    return message


class MarkerScaleRelay:
    def __init__(self):
        scale = float(rospy.get_param('~scale'))
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError(f'~scale must be finite and > 0, got {scale!r}')
        input_topic = rospy.get_param(
            '~input_topic', '/local_planner/uncertainty_risk_raw')
        output_topic = rospy.get_param(
            '~output_topic', '/local_planner/uncertainty_risk')
        self.scale = scale
        self.publisher = rospy.Publisher(
            output_topic, MarkerArray, queue_size=10)
        self.subscriber = rospy.Subscriber(
            input_topic, MarkerArray, self._callback, queue_size=10)
        rospy.loginfo(
            'uncertainty marker relay: %s -> %s, ellipsoid scale x%.3g',
            input_topic, output_topic, scale)

    def _callback(self, message):
        self.publisher.publish(
            scale_uncertainty_ellipsoids(message, self.scale))


if __name__ == '__main__':
    rospy.init_node('uncertainty_marker_scale_relay', anonymous=True)
    MarkerScaleRelay()
    rospy.spin()
