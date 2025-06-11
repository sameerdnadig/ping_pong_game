#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
import time

class MyNode(Node):
    def __init__(self):
        super().__init__("ping_pong2")
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.get_logger().info('Waiting for /spawn service...')
        self.spawn_client.wait_for_service()
        self.spawn()

        self.cmd_vel = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)
        self.create_subscription(Pose, "/turtle1/pose", self.turtle1_callback, 10)
        self.create_subscription(Pose, "/turtle2/pose", self.turtle2_callback, 10)

        self.turtle1_pose = None  # Target
        self.turtle2_pose = None  # Current

        # PID coefficients
        self.Kp = 4.0
        self.Ki = 0.1
        self.Kd = 0.4

        # PID state
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        # Timer for control loop
        self.create_timer(0.05, self.pid_control)  # 20 Hz

    def spawn(self):
        spawn_req = Spawn.Request()
        spawn_req.x = 5.0
        spawn_req.y = 2.0
        spawn_req.theta = 0.0
        spawn_req.name = 'turtle2'
        future = self.spawn_client.call_async(spawn_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Spawned turtle '{future.result().name}' at (5.0, 2.0)")
        else:
            self.get_logger().error("Failed to spawn turtle2.")

    def turtle1_callback(self, msg):
        self.turtle1_pose = msg

    def turtle2_callback(self, msg):
        self.turtle2_pose = msg

    def pid_control(self):
        if self.turtle1_pose is None or self.turtle2_pose is None:
            return

        # Calculate error
        error = self.turtle1_pose.x - self.turtle2_pose.x

        # Time step
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        if dt <= 0.0:
            return

        # PID terms
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Command turtle2 to move along x-axis
        cmd = Twist()
        cmd.linear.x = float(max(min(output, 2.0), -2.0))  # limit speed
        self.cmd_vel.publish(cmd)

        # Update for next iteration
        self.prev_error = error
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node=MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()