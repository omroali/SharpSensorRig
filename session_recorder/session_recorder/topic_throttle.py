#!/usr/bin/env python3
"""Zero-copy topic throttle for bag recording.

Relays each configured topic onto a `<topic>/throttled` counterpart at a
reduced rate, forwarding the serialized message bytes untouched (raw
subscription -> raw publish). No message is ever deserialized, so the CPU
cost is per-message bookkeeping regardless of payload size.

Spawned by recording_manager for the duration of a session with:

  throttles: ["<in_topic>:<out_topic>:<hz>", ...]

Message types are discovered from the ROS graph; relays for topics that have
not appeared yet are retried on a timer, so start order does not matter.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

DISCOVERY_PERIOD_S = 2.0


class TopicThrottle(Node):
    def __init__(self):
        super().__init__("topic_throttle")
        self.declare_parameter("throttles", Parameter.Type.STRING_ARRAY)

        self._pending: list[tuple[str, str, float]] = []
        for spec in self.get_parameter("throttles").value or []:
            spec = str(spec)
            if not spec:
                continue
            try:
                in_topic, out_topic, hz_str = spec.split(":")
                self._pending.append((in_topic, out_topic, float(hz_str)))
            except ValueError:
                self.get_logger().error(
                    f"Bad throttle spec '{spec}' (want <in>:<out>:<hz>); skipping"
                )

        if not self._pending:
            self.get_logger().warn("No throttle specs configured; idling")

        self._relays = []  # keep subs/pubs alive
        self._discovery_timer = self.create_timer(
            DISCOVERY_PERIOD_S, self._try_connect_pending
        )
        self._try_connect_pending()

    def _try_connect_pending(self):
        if not self._pending:
            self._discovery_timer.cancel()
            return
        available = dict(self.get_topic_names_and_types())
        still_pending = []
        for in_topic, out_topic, hz in self._pending:
            types = available.get(in_topic)
            if not types:
                still_pending.append((in_topic, out_topic, hz))
                continue
            try:
                self._connect(in_topic, out_topic, hz, types[0])
            except Exception as exc:  # bad type import should not kill others
                self.get_logger().error(
                    f"Failed to relay {in_topic} ({types[0]}): {exc}"
                )
        self._pending = still_pending

    def _connect(self, in_topic: str, out_topic: str, hz: float, type_name: str):
        msg_type = get_message(type_name)
        # Reliable keeps parity with the sensor bridges; the bag recorder
        # adapts its subscription QoS to whatever we offer.
        pub = self.create_publisher(
            msg_type, out_topic, QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        )
        min_interval = 1.0 / hz
        state = {"last": 0.0}

        def callback(serialized: bytes):
            now = time.monotonic()
            if now - state["last"] >= min_interval:
                state["last"] = now
                pub.publish(serialized)

        sub = self.create_subscription(
            msg_type,
            in_topic,
            callback,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
            raw=True,
        )
        self._relays.append((sub, pub))
        self.get_logger().info(
            f"Throttling {in_topic} -> {out_topic} at {hz:g} Hz ({type_name})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TopicThrottle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
