#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
import time

class SmileDrawer(Node):
    def __init__(self):
        super().__init__('smile_drawer')
        
        self.publisher_1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.spawn_client = self.create_client(Spawn, 'spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Menunggu service spawn...')
        
        self.move_to_position(self.publisher_1, 3.0, 8.0)
        
        self.spawn_turtle('turtle2', 8.0, 8.0, 0.0)

        self.spawn_turtle('turtle3', 5.5, 5.0, 0.0)
        
        self.publisher_2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.publisher_3 = self.create_publisher(Twist, '/turtle3/cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.draw_smile)
        self.step = 0

    def spawn_turtle(self, name, x, y, theta):
        request = Spawn.Request()
        request.x = float(x)
        request.y = float(y)
        request.theta = float(theta)
        request.name = name
        
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def move_turtle(self, publisher, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        publisher.publish(twist)

    def move_to_position(self, publisher, target_x, target_y):
        twist = Twist()
        twist.linear.x = 1.0
        publisher.publish(twist)
        time.sleep(2)
        self.move_turtle(publisher, 0.0, 0.0)

    def draw_smile(self):
        if self.step == 0:
            self.move_turtle(self.publisher_1, 0.5, 2.0)  
            time.sleep(2.5) 
            self.move_turtle(self.publisher_1, 0.0, 0.0)
            self.step += 1
        
        elif self.step == 1:
            self.move_turtle(self.publisher_2, 0.5, 2.0) 
            time.sleep(2.5) 
            self.move_turtle(self.publisher_2, 0.0, 0.0)
            self.step += 1
        
        elif self.step == 2:
            self.move_turtle(self.publisher_3, 0.8, 0.8) 
            time.sleep(4)  
            self.move_turtle(self.publisher_3, 0.0, 0.0)
            self.step += 1
        
        else:
            self.timer.cancel()

def main():
    rclpy.init()
    smile_drawer = SmileDrawer()
    rclpy.spin(smile_drawer)
    smile_drawer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()