#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.i = 0

    def listener_callback(self, msg):
        full_string = msg.data  # 接收的消息，例如 "N:2697"
        self.get_logger().info('I heard: "%s"' % full_string)

    def timer_callback(self):
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

