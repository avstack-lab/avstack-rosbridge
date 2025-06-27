import numpy as np
import rclpy
from avstack.environment.objects import ObjectState
from avstack.geometry import (
    Acceleration,
    AngularVelocity,
    Attitude,
    Box3D,
    GlobalOrigin3D,
    Position,
    Velocity,
    q_stan_to_cam,
)
from avstack.modules.tracking import BasicBoxTrack3D
from rclpy.node import Node
from std_msgs.msg import Header

from avstack_bridge.objects import ObjectStateBridge
from avstack_bridge.tracks import TrackBridge
from avstack_msgs.msg import BoxTrack3DArray, ObjectStateArray


def get_object_global(seed, reference=GlobalOrigin3D):
    np.random.seed(seed)
    pos_obj = Position(10 * np.random.rand(3), reference)
    rot_obj = Attitude(q_stan_to_cam, reference)
    box_obj = Box3D(pos_obj, rot_obj, [2, 2, 5])  # box in local coordinates
    vel_obj = Velocity(10 * np.random.rand(3), reference)
    acc_obj = Acceleration(np.random.rand(3), reference)
    ang_obj = AngularVelocity(np.quaternion(1), reference)
    obj = ObjectState("car")
    obj.set(0, pos_obj, box_obj, vel_obj, acc_obj, rot_obj, ang_obj)
    return obj


def object_to_BoxTrack3D(obj: ObjectState) -> BasicBoxTrack3D:
    box_noisy = obj.box3d
    box_noisy.position += np.random.randn(3)
    track = BasicBoxTrack3D(
        t0=obj.t,
        box3d=box_noisy,
        reference=obj.reference,
        obj_type=obj.obj_type,
        v=obj.velocity,
    )
    return track


class TrackAndTruthPublisher(Node):
    def __init__(self):
        super().__init__("track_and_truth_publisher")
        self.publisher_tracks = self.create_publisher(BoxTrack3DArray, "tracks", 10)
        self.publisher_truths = self.create_publisher(ObjectStateArray, "truths", 10)
        self._timer = self.create_timer(0.1, self.publish_loop)

    def publish_loop(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "world"

        # create truth messages
        n_truths = 10
        truths = [get_object_global(seed=i) for i in range(n_truths)]
        msg_truths = ObjectStateBridge.avstack_to_objecstatearray(truths, header=header)
        self.publisher_truths.publish(msg_truths)

        # create track messages
        np.random.seed(None)
        n_tracks_by_truths = 8
        tracks = [object_to_BoxTrack3D(obj) for obj in truths[:n_tracks_by_truths]]
        msg_tracks = TrackBridge.avstack_to_tracks(tracks, header=header)
        self.publisher_tracks.publish(msg_tracks)


def main(args=None):
    rclpy.init(args=args)

    trackandtruth = TrackAndTruthPublisher()

    rclpy.spin(trackandtruth)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    trackandtruth.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
