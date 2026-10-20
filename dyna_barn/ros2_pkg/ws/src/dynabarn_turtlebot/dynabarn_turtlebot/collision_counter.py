#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import UInt32, Bool
from std_srvs.srv import Empty

class CollisionCounter(Node):
    def __init__(self):
        super().__init__('collision_counter')

        # Params
        self.declare_parameter('bumper_topic', '/bumper')
        self.declare_parameter('debounce_ms', 150)   # ignore flaps within this window
        self.declare_parameter('print_every', 1.0)   # seconds between status prints

        self.topic = self.get_parameter('bumper_topic').value
        self.debounce = Duration(milliseconds=int(self.get_parameter('debounce_ms').value))
        self.print_every = float(self.get_parameter('print_every').value)

        # State
        self.count = 0
        self.in_contact = False
        self.last_change_time = None
        self.last_print_time = self.get_clock().now()

        # I/O
        self.sub = self.create_subscription(ContactsState, self.topic, self._on_contacts, 10)
        self.pub_count = self.create_publisher(UInt32, 'collision_count', 1)
        self.pub_contact = self.create_publisher(Bool, 'collision_contact', 1)
        self.srv_reset = self.create_service(Empty, 'reset', self._on_reset)

        self.get_logger().info(f"Listening for contacts on {self.topic}")

    def _on_contacts(self, msg: ContactsState):
        now = self.get_clock().now()
        currently_contact = len(msg.states) > 0

        # Rising/falling edge with debounce
        if currently_contact != self.in_contact:
            if self.last_change_time is None or (now - self.last_change_time) >= self.debounce:
                if not self.in_contact and currently_contact:
                    self.count += 1
                    pairs = ", ".join(f"{s.collision1} ↔ {s.collision2}" for s in msg.states)
                    self.get_logger().warn(f"HIT #{self.count}  pairs: {pairs if pairs else 'n/a'}")
                self.in_contact = currently_contact
                self.last_change_time = now

        # Periodic status publish
        if (now - self.last_print_time).nanoseconds >= int(self.print_every * 1e9):
            self.last_print_time = now
            self.pub_count.publish(UInt32(data=self.count))
            self.pub_contact.publish(Bool(data=self.in_contact))

    def _on_reset(self, req, res):
        self.count = 0
        self.in_contact = False
        self.last_change_time = None
        self.get_logger().info("Collision counter reset.")
        return res

def main():
    rclpy.init()
    node = CollisionCounter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
