#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
import math

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController(Node):
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Uses a pure pursuit approach for smoother tracking.
    """
    def __init__(self):
        super().__init__("parking_controller")

        self.declare_parameter("drive_topic")
        DRIVE_TOPIC = self.get_parameter("drive_topic").value # set in launch file; different for simulator vs racecar
        self.get_logger().info(DRIVE_TOPIC)
        
        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)

        self.create_subscription(ConeLocation, "/relative_cone", 
            self.relative_cone_callback, 1)

        # Target distance to park in front of the cone
        self.parking_distance = 0.75  # meters
        self.relative_x = 0.0
        self.relative_y = 0.0
        
        # Car parameters
        self.wheelbase = 0.325  # Distance between front and rear axles (meters)
        
        # Controller parameters
        self.max_speed = 0.7     # Maximum speed (m/s)
        self.min_speed = 0.2     # Minimum speed when moving
        self.max_steering = 0.6  # Maximum steering angle (radians)
        
        # Pure pursuit parameter
        self.lookahead_gain = 2.0  # Gain for pure pursuit steering
        
        # Thresholds for parking
        self.distance_threshold = 0.1  # Meters
        self.angle_threshold = 0.1     # Radians (~5.7 degrees)
        
        # Control smoothing
        self.prev_steering = 0.0
        self.prev_speed = 0.0
        self.steering_smoothing = 0.5  # Smoothing factor (0-1)
        self.speed_smoothing = 0.4     # Smoothing factor (0-1)
        
        self.get_logger().info("Pure Pursuit Parking Controller Initialized")

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        
        # Calculate distance to the cone
        distance_to_cone = np.sqrt(self.relative_x**2 + self.relative_y**2)
        
        # Calculate angle to the cone
        angle_to_cone = np.arctan2(self.relative_y, self.relative_x)
        
        # Calculate distance error (how far we are from desired parking distance)
        distance_error = distance_to_cone - self.parking_distance
        
        # Initialize drive command
        drive_cmd = AckermannDriveStamped()
        
        # Pure pursuit steering calculation
        # If cone is behind us, we need special handling
        if self.relative_x < 0:
            # If cone is directly behind, turn with max steering
            if abs(self.relative_y) < 0.1:
                steering_angle = self.max_steering  # Just turn with max steering
            else:
                # Turn toward the cone with sign based on which side it's on
                steering_angle = self.max_steering * np.sign(self.relative_y)
        else:
            # Pure pursuit calculation for steering
            # The curvature is 2*y/(x²+y²) for a point at (x,y)
            # Steering angle = atan(wheelbase * curvature)
            curvature = 2.0 * self.relative_y / (self.relative_x**2 + self.relative_y**2)
            steering_angle = np.arctan(self.wheelbase * curvature * self.lookahead_gain)
            
            # Limit steering angle
            steering_angle = max(-self.max_steering, min(self.max_steering, steering_angle))
        
        # Speed control based on distance and steering
        if distance_error > self.distance_threshold:
            # We're too far, move toward the cone
            # Reduce speed when turning sharply
            steering_factor = 1.0 - 0.7 * (abs(steering_angle) / self.max_steering)
            
            # Reduce speed when getting closer to target
            distance_factor = min(1.0, distance_error / 1.0)
            
            # Combine factors
            target_speed = self.max_speed * min(steering_factor, distance_factor)
            
            # Ensure minimum speed when moving
            if target_speed < self.min_speed:
                target_speed = self.min_speed
        elif abs(angle_to_cone) > self.angle_threshold:
            # We're at the right distance but need to align
            # Back up slowly while turning
            target_speed = -0.2
            # Reverse steering direction when backing up
            steering_angle = -steering_angle * 0.7
        else:
            # We're parked!
            target_speed = 0.0
            steering_angle = 0.0
        
        # Apply smoothing to controls
        speed = self.prev_speed + self.speed_smoothing * (target_speed - self.prev_speed)
        steering = self.prev_steering + self.steering_smoothing * (steering_angle - self.prev_steering)
        
        # Save current controls for next iteration
        self.prev_speed = speed
        self.prev_steering = steering
        
        # Ensure we're not trying to turn while stationary
        if abs(speed) < 0.05 and abs(steering) > 0.1:
            # If we need to turn but are nearly stopped, add a small speed
            speed = self.min_speed if steering * self.relative_y > 0 else -self.min_speed
        
        # Set the drive command
        drive_cmd.drive.speed = float(speed)
        drive_cmd.drive.steering_angle = float(steering)
        
        # Publish drive command and error
        self.drive_pub.publish(drive_cmd)
        self.error_publisher()
        
        # Log information
        self.get_logger().info(f"Cone: x={self.relative_x:.2f}, y={self.relative_y:.2f}, "
                               f"dist_err={distance_error:.2f}, angle={angle_to_cone:.2f}, "
                               f"speed={speed:.2f}, steering={steering:.2f}")

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()
        
        # X error is the difference between current x position and desired x position
        error_msg.x_error = self.relative_x - self.parking_distance
        
        # Y error is just the y position (we want y=0)
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
