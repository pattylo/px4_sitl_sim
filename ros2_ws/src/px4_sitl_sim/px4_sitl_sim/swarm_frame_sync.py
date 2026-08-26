#!/usr/bin/env python3
"""Express every MAVROS local pose in agent 0's initial local frame."""

from copy import deepcopy
from functools import partial
from typing import Sequence

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

MAVROS_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

RELAY_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def relative_offsets(
    initial_positions: Sequence[float], vehicle_count: int
) -> tuple[tuple[float, float, float], ...]:
    """Return Gazebo ENU spawn translations relative to agent 0."""
    if vehicle_count < 1:
        raise ValueError('vehicle_count must be at least 1')

    expected_values = 3 * vehicle_count
    if len(initial_positions) != expected_values:
        raise ValueError(
            'initial_gazebo_positions must contain exactly '
            f'{expected_values} values (x, y, z for each vehicle); '
            f'got {len(initial_positions)}'
        )

    origin_x, origin_y, origin_z = (
        float(value) for value in initial_positions[0:3]
    )
    offsets = []
    for index in range(vehicle_count):
        start = 3 * index
        position = initial_positions[start:start + 3]
        x, y, z = (float(value) for value in position)
        offsets.append((x - origin_x, y - origin_y, z - origin_z))

    return tuple(offsets)


class SwarmFrameSync(Node):
    """Translate MAVROS ENU poses into agent 0's initial ENU frame."""

    def __init__(self) -> None:
        super().__init__('swarm_frame_sync')

        self.declare_parameter('vehicle_count', 1)
        self.declare_parameter('namespace_prefix', 'uav')
        self.declare_parameter(
            'initial_gazebo_positions', [0.0, 0.0, 0.3]
        )

        vehicle_count = int(self.get_parameter('vehicle_count').value)
        namespace_prefix = str(
            self.get_parameter('namespace_prefix').value
        ).strip('/')
        initial_positions = self.get_parameter(
            'initial_gazebo_positions'
        ).value

        if not namespace_prefix:
            raise ValueError('namespace_prefix must not be empty')

        self._reference_frame = f'{namespace_prefix}0/local_origin'
        self._offsets = relative_offsets(initial_positions, vehicle_count)
        # Do not use Node's private _publishers/_subscriptions attribute names.
        # rclpy owns those registries and corrupting them breaks indexed relays
        # as well as publisher/subscription cleanup during shutdown.
        self._pose_publishers = []
        self._pose_subscriptions = []

        # MAVROS local_position/pose is geometry_msgs/PoseStamped in ROS ENU.
        # Sensor-data QoS is compatible with MAVROS's best-effort publishers.
        for agent_index, offset in enumerate(self._offsets):
            mavros_namespace = f'/{namespace_prefix}{agent_index}/mavros'
            input_topic = f'{mavros_namespace}/local_position/pose'
            output_topic = (
                f'{mavros_namespace}/local_position/pose_agent0'
            )

            publisher = self.create_publisher(
                PoseStamped,
                output_topic,
                RELAY_QOS,
            )

            subscription = self.create_subscription(
                PoseStamped,
                input_topic,
                partial(self._pose_callback, agent_index),
                MAVROS_QOS,
            )

            self._pose_publishers.append(publisher)
            self._pose_subscriptions.append(subscription)

            self.get_logger().info(
                f'agent {agent_index}: {input_topic} -> {output_topic}; '
                f'agent-0 offset={offset}'
            )

    def _pose_callback(
        self, agent_index: int, message: PoseStamped
    ) -> None:
        offset_x, offset_y, offset_z = self._offsets[agent_index]

        transformed = PoseStamped()
        transformed.header = deepcopy(message.header)
        transformed.header.frame_id = self._reference_frame
        transformed.pose = deepcopy(message.pose)
        transformed.pose.position.x += offset_x
        transformed.pose.position.y += offset_y
        transformed.pose.position.z += offset_z

        # Every vehicle is spawned with the same zero yaw, so its MAVROS ENU
        # orientation already has the same axes as agent 0's local frame.
        self._pose_publishers[agent_index].publish(transformed)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SwarmFrameSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
