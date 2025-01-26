#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill

from functools import partial
import random
import math
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle


class TurtleSpawner(Node): 
    def __init__(self):
        super().__init__("turtle_spawner")
        
        self.counter_ = 0
        self.alive_turtles = []

        self.declare_parameter("spawn_frequency", 1.0)
        self.spawn_frequency = self.get_parameter("spawn_frequency").value
        self.declare_parameter("turtle_prefix", "Turtle_")
        self.turtle_prefix = self.get_parameter("turtle_prefix").value

        self.timer_ = self.create_timer(1.0/self.spawn_frequency,self.create_new_turtles)
        self.alive_turtles_publisher_ = self.create_publisher(TurtleArray, "alive_turtles", 10)
        catch_turtle_server = self.create_service(CatchTurtle, "catch_turtles", self.call_kill_turtle)

    def call_kill_turtle(self, request, response):
        if not self.alive_turtles:
            response.success = False
            return response
        
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server " + client.srv_name + ".....")

        request1 = Kill.Request()
        request1.name = request.turtle_name

        future = client.call_async(request1)
        future.add_done_callback(partial(self.callback_call_kill_turtle, turtle_name = request.turtle_name))
        response.success = True

        return response

    def callback_call_kill_turtle(self, future, turtle_name):
        try:
            for (i, turtle) in enumerate(self.alive_turtles):
                if turtle.name == turtle_name:
                    del self.alive_turtles[i]
                    self.publish_alive_turtles()
                    break
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
            
    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtle = self.alive_turtles
        self.alive_turtles_publisher_.publish(msg)

    def create_new_turtles(self):
        self.counter_ += 1
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0, 2*math.pi)
        name = self.turtle_prefix + str(self.counter_)
        self.call_spawner(x, y, theta, name)
        
    def call_spawner(self, x, y, theta, name):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server " + client.srv_name + ".....")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_spawner, x=x, y=y, theta=theta, turtle_name=name)) #using partial to give extra args to callback fn
    
    def callback_call_spawner(self, future, x, y, theta, turtle_name):
        try:
            response = future.result()
            if response.name != "":
                self.get_logger().info(response.name + " is now alive!")
                msg = Turtle()
                msg.x = x
                msg.y = y
                msg.theta = theta
                msg.name = turtle_name
                self.alive_turtles.append(msg)
                self.publish_alive_turtles()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
    
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
