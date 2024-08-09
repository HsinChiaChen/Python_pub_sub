#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray, Int16MultiArray

class MinimalTracking(Node):

    def __init__(self):
        super().__init__('tracking')

        self.timer = self.create_timer(0.01, self.timer_callback)
        self.i = 0
        self.y = 0.0
        self.y_design = 1.0
        self.Kp = 1.2
        self.time_initial = time.time()
        self.x_data = []
        self.y_data = []

        # 初始化圖形
        plt.ion()  # 開啟互動模式
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot(self.x_data, self.y_data)

        self.publisher_ = self.create_publisher(Float32MultiArray, 'Array_topic_x', 1)

    def timer_callback(self):
        
        self.i += 1
        # Simulation generated data
        new_x = time.time() - self.time_initial

        self.y += self.Kp * (self.y_design - self.y)
        new_y = self.y

        # Update data
        self.x_data.append(new_x)
        self.y_data.append(new_y)

        # Update graphics
        self.line.set_xdata(self.x_data)
        self.line.set_ydata(self.y_data)
        self.ax.relim()  # Recalculate axis ranges
        self.ax.autoscale_view()  # Auto zoom view
        self.fig.canvas.draw()  # Draw graphics
        self.fig.canvas.flush_events()  # refresh event

        print(new_x)
        msg = Float32MultiArray()
        msg.data = self.x_data
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalTracking()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

