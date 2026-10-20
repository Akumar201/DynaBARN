#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ContactsState

class CollisionWatch(Node):
    def __init__(self):
        super().__init__('collision_watch')
        # default to /bumper_states; you can override with --ros-args -p topic:=/something
        self.declare_parameter('topic', '/bumper_states')
        topic = self.get_parameter('topic').value
        self._in_contact = False
        self.create_subscription(ContactsState, topic, self._on_msg, 10)
        self.get_logger().info(f"Watching collisions on {topic}")

    def _on_msg(self, msg: ContactsState):
        contact_now = len(msg.states) > 0
        # Print only when contact begins (rising edge)
        if contact_now and not self._in_contact:
            pairs = ", ".join(f"{s.collision1} <-> {s.collision2}" for s in msg.states) or "n/a"
            self.get_logger().warn(f"COLLISION! pairs: {pairs}")
        self._in_contact = contact_now

def main():
    rclpy.init()
    node = CollisionWatch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
