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

        self.i = 0
        self.y = 0.0
        self.y1 = 0.0
        self.y_design = 1.0
        # self.Kp = 1.2
        # self.Ki = 0.005
        # self.Kd = 0.05

        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0

        self.time_initial = time.time()
        self.x_data = []
        self.y_data = []
        self.y1_data = []
        self.y_last = 0.0
        self.error_tot = 0

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'param',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.timer = self.create_timer(0.01, self.timer_callback)
        
        # 初始化圖形
        plt.ion()  # 開啟互動模式
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot(self.x_data, self.y_data, label = "PID")
        self.line1, = self.ax.plot(self.x_data, self.y1_data, label = "P")
        self.ax.legend()

        self.publisher_ = self.create_publisher(Float32MultiArray, 'Array_topic_x', 1)

    def listener_callback(self, msg):
        control_param = msg.data  # 接收的消息，例如 "N:2697"
        self.Kp = control_param[0]
        self.Ki = control_param[1]
        self.Kd = control_param[2]
        print(self.Kp, self.Ki, self.Kd)

    def timer_callback(self):

        if self.Kp != 0.0:
            print('run')
            
            self.i += 1
            # Simulation generated data
            new_x = time.time() - self.time_initial

            error = self.y - self.y_last
            self.error_tot += error
            self.y1 += self.Kp * (self.y_design - self.y1)
            self.y += self.Kp * (self.y_design - self.y) + self.Kd * (error) + self.Ki * self.error_tot
            new_y = self.y
            if self.i > 1: self.y_last = new_y

            # Update data
            self.x_data.append(new_x)
            self.y_data.append(new_y)
            self.y1_data.append(self.y1)

            if new_x < 3.0:
                # Update graphics
                self.line.set_xdata(self.x_data)
                self.line.set_ydata(self.y_data)
                self.line1.set_xdata(self.x_data)
                self.line1.set_ydata(self.y1_data)
                self.ax.relim()  # Recalculate axis ranges
                self.ax.autoscale_view()  # Auto zoom view
                self.fig.canvas.draw()  # Draw graphics
                self.fig.canvas.flush_events()  # refresh event
                time.sleep(0.01)

            print('PIDcontrol: ', new_y)
            print('Pcontrol: ', self.y1)
            print('----------------------')

            # print(self.y_last)
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

