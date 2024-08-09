#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('publisher')

        self.qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,depth=1)

        self.declare_parameter('samplingtime',0.1)
        self.samplingtime = self.get_parameter('samplingtime').get_parameter_value().double_value

        self.declare_parameter('word','word')
        self.word = self.get_parameter('word').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(String, 'topic', 1)
        self.timer = self.create_timer(self.samplingtime, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = self.word + ':' + str(self.i)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

