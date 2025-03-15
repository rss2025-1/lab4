#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController(Node):
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        super().__init__("parking_controller")

        self.declare_parameter("drive_topic")
        DRIVE_TOPIC = self.get_parameter("drive_topic").value # set in launch file; different for simulator vs racecar

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)

        self.create_subscription(ConeLocation, "/relative_cone", 
            self.relative_cone_callback, 1)

        self.parking_distance = .75 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0
        
        # Pure pursuit parameters
        self.wheelbase = 0.325  # Distance between front and rear axles (meters)
        self.max_speed = 0.8    # Maximum speed (m/s)
        self.min_speed = 0.1    # Minimum speed when moving (m/s)
        self.lookahead_factor = 1.5  # Factor to determine lookahead distance
        self.k_speed = 0.5      # Speed gain

        self.get_logger().info("Parking Controller Initialized")

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        # Calculate distance to the cone
        distance_to_cone = np.sqrt(self.relative_x**2 + self.relative_y**2)
        
        # Calculate the goal point - this is where we want to be
        # If we're farther than parking_distance, our goal is parking_distance meters in front of the cone
        # If we're closer than parking_distance, our goal is to back up to parking_distance
        
        # First, calculate unit vector pointing from car to cone
        if distance_to_cone > 0.001:  # Avoid division by zero
            unit_x = self.relative_x / distance_to_cone
            unit_y = self.relative_y / distance_to_cone
        else:
            unit_x, unit_y = 1.0, 0.0  # Default to forward if we're exactly at the cone
            
        # Calculate goal point (where we want the car to be)
        goal_distance = self.parking_distance
        goal_x = self.relative_x - (distance_to_cone - goal_distance) * unit_x
        goal_y = self.relative_y - (distance_to_cone - goal_distance) * unit_y
        
        # Pure pursuit algorithm
        # Calculate lookahead distance (can be dynamic based on speed)
        lookahead_distance = max(0.1, min(distance_to_cone, self.lookahead_factor * abs(distance_to_cone - goal_distance)))
        
        # Calculate curvature (1/radius) using pure pursuit formula
        # For a car at the origin facing along the x-axis, the curvature to a point (x,y) is:
        # curvature = 2y / (x^2 + y^2)
        # But we need to handle special cases
        
        if abs(goal_x) < 0.001 and abs(goal_y) < 0.001:
            # We're already at the goal
            curvature = 0.0
        else:
            # Pure pursuit curvature calculation
            curvature = 2.0 * goal_y / (goal_x**2 + goal_y**2)
            
        # Calculate steering angle from curvature
        # steering_angle = arctan(wheelbase * curvature)
        steering_angle = np.arctan(self.wheelbase * curvature)
        
        # Calculate speed based on distance error and curvature
        distance_error = distance_to_cone - self.parking_distance
        
        # Determine direction (forward/backward) based on where the goal is
        direction = 1.0 if goal_x >= 0 else -1.0
        
        # Speed proportional to distance error, reduced by curvature
        speed = direction * self.k_speed * abs(distance_error) / (1.0 + abs(steering_angle))
        
        # Limit speed
        speed = max(min(speed, self.max_speed), -self.max_speed)
        
        # If we're very close to the goal, stop
        if abs(distance_error) < 0.05:
            speed = 0.0
        # Ensure minimum speed when moving
        elif abs(speed) < self.min_speed and abs(distance_error) > 0.05:
            speed = self.min_speed if speed > 0 else -self.min_speed
            
        # Set the drive command
        drive_cmd.drive.speed = float(speed)
        drive_cmd.drive.steering_angle = float(steering_angle)

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        # Calculate the errors
        # X error is the difference between current x position and desired x position (parking_distance)
        error_msg.x_error = self.relative_x - self.parking_distance
        
        # Y error is just the y position (we want y=0, meaning the cone is directly in front)
        error_msg.y_error = self.relative_y
        
        # Distance error is the Euclidean distance from the desired parking position
        current_distance = np.sqrt(self.relative_x**2 + self.relative_y**2)
        error_msg.distance_error = current_distance - self.parking_distance

        self.error_pub.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()

if __name__ == '__main__':
    main()