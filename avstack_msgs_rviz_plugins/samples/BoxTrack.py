#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from math import cos, pi, sin

import rclpy
from geometry_msgs.msg import Vector3
from numpy import array
from numpy.linalg import norm
from rclpy.node import Node
from std_msgs.msg import Header
from vision_msgs.msg import BoundingBox3D

from avstack_msgs.msg import BoxTrack


def quaternion_about_axis(angle, axis):
    axis = array(axis)
    axis = axis / norm(axis)
    half_angle = angle / 2
    sine = sin(half_angle)
    w = cos(half_angle)
    x, y, z = axis * sine
    return x, y, z, w


class PubBoxTrack(Node):
    def __init__(self):
        super().__init__("pub_box_track_sample")
        self._pub = self.create_publisher(BoxTrack, "box_track", 10)
        self._timer = self.create_timer(0.1, self.pub_sample)
        self._counter = 0
        self._header = Header()

    def pub_sample(self):
        while self._pub.get_subscription_count() == 0:
            return
        self._header.stamp = self.get_clock().now().to_msg()
        self._header.frame_id = "map"

        # Reset counter and indices if counter is a multiple of 30
        if self._counter % 10 == 0:
            self._counter = 0

        # Create a bounding box
        bbox = BoundingBox3D()
        quat = quaternion_about_axis((self._counter % 100) * pi * 2 / 100.0, [0, 0, 1])
        bbox.center.orientation.x = quat[0]
        bbox.center.orientation.y = quat[1]
        bbox.center.orientation.z = quat[2]
        bbox.center.orientation.w = quat[3]
        # Set the center position to a fixed value
        bbox.center.position.x = 10.0
        bbox.center.position.y = 1.2
        bbox.size.x = (self._counter % 10 + 1) * 0.1
        bbox.size.y = ((self._counter + 1) % 5 + 1) * 0.1
        bbox.size.z = ((self._counter + 2) % 10 + 1) * 0.1

        # Create a twist
        velocity = Vector3()
        velocity.x = 2.2
        velocity.y = 1.0
        velocity.z = 0.0

        # Create a single ObjectState message
        trk = BoxTrack()
        trk.obj_type = "vehicle"
        trk.box = bbox
        trk.velocity = velocity

        # Publish the ObjectState message
        self._pub.publish(trk)
        self._counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = PubBoxTrack()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
