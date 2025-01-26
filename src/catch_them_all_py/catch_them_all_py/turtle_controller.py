#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from functools import partial

from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle
    
class TurtleController(Node): 
    def __init__(self):
        super().__init__("turtle_controller") 
        self.pose = None
        self.turtle_to_catch_ = None
        
        self.declare_parameter("catch_closest_turtle_first", True)
        self.catch_closest_turtle_ = self.get_parameter("catch_closest_turtle_first").value

        self.alive_turtles_subscriber_ = self.create_subscription(
            TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)

        self.subscriber_ = self.create_subscription(
            Pose, "turtle1/pose", self.callback_turtle_pose, 10)
        self.get_logger().info("turtle_controller now subscribes to turtle1/pose")

        self.publisher_ = self.create_publisher(
            Twist, "turtle1/cmd_vel", 10)
        self.timer_ = self.create_timer(0.01, self.control_loop)
        self.get_logger().info("turtle_controller now publishes on turtle1/cmd_vel")
    
    def callback_alive_turtles(self, msg):
        if len(msg.turtle) > 0 and self.pose:
            if self.catch_closest_turtle_:
                closest_turtle = None
                closest_turtle_distance = None
                for turtle in msg.turtle:
                    diff_x = turtle.x - self.pose.x
                    diff_y = turtle.y - self.pose.y
                    distance = math.sqrt((diff_x * diff_x) + (diff_y * diff_y))
                    if closest_turtle == None or distance < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = distance
                self.turtle_to_catch_ = closest_turtle
            else:
                self.turtle_to_catch_ = msg.turtle[0]

    def callback_turtle_pose(self, msg):
        self.pose = msg
    
    def callback_catch_turtles(self, future, turtle_name):
        try:
            response = future.result()
            self.turtle_to_catch_ = None
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def control_loop(self):
        if self.pose == None or self.turtle_to_catch_ == None:
            return
        
        diff_x = self.turtle_to_catch_.x - self.pose.x
        diff_y = self.turtle_to_catch_.y - self.pose.y
        distance = math.sqrt((diff_x * diff_x) + (diff_y * diff_y))

        msg = Twist()

        if distance > 0.5:
            msg.linear.x = 2*distance

            target_theta = math.atan2(diff_y, diff_x)
            goal_theta = target_theta - self.pose.theta
            if goal_theta > math.pi:
                goal_theta -= 2*math.pi
            elif goal_theta < -math.pi:
                goal_theta += 2*math.pi
            
            msg.angular.z = 6*goal_theta
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            client = self.create_client(CatchTurtle, "catch_turtles")
            while not client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server " + client.srv_name + ".....")

            request = CatchTurtle.Request()
            request.turtle_name = self.turtle_to_catch_.name

            future = client.call_async(request)
            future.add_done_callback(partial(self.callback_catch_turtles, turtle_name = self.turtle_to_catch_.name))

        self.publisher_.publish(msg)

    
def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
