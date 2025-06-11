#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn, TeleportRelative


class MyNode(Node):
    def __init__(self):
        super().__init__("draw_square")
        self.kill_client = self.create_client(Kill, '/kill')
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.get_logger().info('Waiting for /kill service...')
        self.kill_client.wait_for_service()
        self.get_logger().info('Waiting for /spawn service...')
        self.spawn_client.wait_for_service()
        self.kill_and_spawn()
        self.cli = self.create_client(TeleportRelative, '/turtle1/teleport_relative')
        self.cmd_vel=self.create_publisher(Twist,"/turtle1/cmd_vel", 10)
        self.pose=self.create_subscription(Pose,"/turtle1/pose",self.send_vel,10)



    def kill_and_spawn(self):
        # Kill turtle1
        req = Kill.Request()
        req.name = 'turtle1'
        future = self.kill_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Turtle 'turtle1' killed successfully.")
        else:
            self.get_logger().error("Failed to kill 'turtle1'.")
        # Spawn a new turtle at custom location
        spawn_req = Spawn.Request()
        spawn_req.x = 5.0
        spawn_req.y = 3.0
        spawn_req.theta = 0.0
        spawn_req.name = 'turtle1'
        future = self.spawn_client.call_async(spawn_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Spawned turtle '{future.result().name}' at (5.0, 5.0)")
        else:
            self.get_logger().error("Failed to spawn new turtle.")

    def turn_turtle(self,angle):
        req = TeleportRelative.Request()
        req.linear = 0.1      # No forward motion
        req.angular = angle      # Turn 90° CCW

        future = self.cli.call_async(req)

        def on_result(fut):
            try:
                fut.result()
                self.get_logger().info('Turtle turned 90 degrees.')
            except Exception as e:
                self.get_logger().error(f'Relative turn failed: {e}')

        future.add_done_callback(on_result)

    
    def send_vel(self, pose:Pose):
        msg=Twist()
        msg.linear.x=2.0
        if pose.x>=8.0 and pose.y<=3.0:
            self.turn_turtle(1.570796)  
            return
          
        elif pose.x>=8.0 and pose.y>=8.0:
            
            self.turn_turtle(1.570796)  # Turn up (90°)
            return 
            
        elif pose.y>=8.0 and pose.x<=3.0:
            self.turn_turtle(1.570796)  # Turn up (90°)
            return
        elif pose.y<=3.0 and pose.x<=3.0:
            self.turn_turtle(1.570796)  # Turn up (90°)
            return
            
        self.cmd_vel.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node=MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
