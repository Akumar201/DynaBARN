#!/usr/bin/env python3
import csv
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclTime
from gazebo_msgs.msg import ContactsState

class BumperTopicLogger(Node):
    def __init__(self):
        super().__init__('bumper_topic_logger')
        # params
        self.declare_parameter('topic', '/bumper_states')    # your topic
        self.declare_parameter('logfile', '')                # optional CSV path
        self.topic = self.get_parameter('topic').value
        self.logfile = self.get_parameter('logfile').value.strip()

        # state
        self.count = 0
        self.writer = None
        self.csvf = None

        if self.logfile:
            self.csvf = open(self.logfile, 'w', newline='')
            self.writer = csv.writer(self.csvf)
            self.writer.writerow(['seq', 'arrival_wall_s', 'hdr_sec', 'hdr_nsec',
                                  'age_s', 'num_states'])

        self.create_subscription(ContactsState, self.topic, self._on_msg, 10)
        self.get_logger().info(f"Logging topic: {self.topic}")

    def _on_msg(self, msg: ContactsState):
        self.count += 1
        now = self.get_clock().now()
        hdr = RclTime.from_msg(msg.header.stamp)
        age = (now - hdr).nanoseconds / 1e9
        nstates = len(msg.states)

        # compact one-liner per message
        self.get_logger().info(
            f"#{self.count:05d}  hdr=({msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d})  "
            f"age={age:0.3f}s  states={nstates}"
        )

        # print pairs if any
        if nstates > 0:
            pairs = ", ".join(f"{s.collision1} <-> {s.collision2}" for s in msg.states)
            self.get_logger().warn(f"CONTACT: {pairs}")

        if self.writer:
            self.writer.writerow([
                self.count,
                now.nanoseconds / 1e9,
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                f"{age:.6f}",
                nstates
            ])
            self.csvf.flush()

def main():
    rclpy.init()
    node = BumperTopicLogger()
    rclpy.spin(node)
    if node.csvf:
        node.csvf.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
