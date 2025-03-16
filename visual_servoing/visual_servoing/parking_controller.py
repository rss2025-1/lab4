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

        # Target distance to park in front of the cone (meters)
        self.parking_distance = 0.5  # ~1.5-2 feet
        self.relative_x = 0
        self.relative_y = 0
        
        # Controller parameters
        self.wheelbase = 0.325  # Distance between front and rear axles (meters)
        self.max_steering = 0.6  # Maximum steering angle (radians)
        
        # Add smoothing for controls
        self.prev_steering = 0.0
        self.prev_speed = 0.0
        self.steering_smoothing = 0.3  # Smoothing factor for steering
        self.speed_smoothing = 0.2     # Smoothing factor for speed
        
        # Frame counter for reduced logging
        self.frame_counter = 0
        self.log_every_n_frames = 20
        
        self.get_logger().info("Simplified Parking Controller Initialized")

    def smooth_control(self, target, previous, smoothing_factor):
        """
        Apply exponential smoothing to control values to prevent jittering
        """
        return previous + smoothing_factor * (target - previous)

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        
        # Only log every n frames
        self.frame_counter += 1
        should_log = (self.frame_counter % self.log_every_n_frames) == 0
        
        if should_log:
            self.get_logger().info(f"Cone position: x={self.relative_x:.2f}, y={self.relative_y:.2f}")
        
        drive_cmd = AckermannDriveStamped()

        # Calculate distance and angle to cone
        distance_to_cone = np.sqrt(self.relative_x**2 + self.relative_y**2)
        theta = np.arctan2(self.relative_y, self.relative_x)
        
        # Pure pursuit steering calculation
        num = self.wheelbase * np.sin(theta)
        den = distance_to_cone/2 + self.wheelbase * np.cos(theta)
        
        # Handle division by zero or very small denominator
        if abs(den) < 0.001:
            target_steering = self.max_steering if num > 0 else -self.max_steering
        else:
            target_steering = np.arctan(num/den)
        
        # Handle cone behind car
        if self.relative_x < 0:
            if abs(self.relative_y) < 0.001:  # Directly behind
                target_steering = self.max_steering  # Just turn to one side
            else:
                target_steering = -1 * target_steering  # Invert steering for reverse
        
        # Limit steering angle
        target_steering = max(min(target_steering, self.max_steering), -self.max_steering)
        
        # Determine speed based on distance to cone
        if distance_to_cone > self.parking_distance:
            # Forward speed proportional to distance and inversely proportional to steering angle
            # This makes the car slow down for sharp turns
            steering_factor = 1.0 - (abs(target_steering) / self.max_steering) * 0.5
            target_speed = 0.4 * steering_factor
        else:
            # We've reached the parking distance
            target_speed = 0.0
            
            # If not aligned with cone, back up
            if abs(self.relative_y) > 0.08:  # > ~5 degrees off
                target_speed = -0.3
                target_steering = -1 * target_steering  # Reverse steering for backing up
        
        # Apply smoothing to controls
        speed = self.smooth_control(target_speed, self.prev_speed, self.speed_smoothing)
        steering_angle = self.smooth_control(target_steering, self.prev_steering, self.steering_smoothing)
        
        # Save current controls for next iteration
        self.prev_speed = speed
        self.prev_steering = steering_angle
        
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
        error_msg.x_error = self.relative_x - self.parking_distance
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