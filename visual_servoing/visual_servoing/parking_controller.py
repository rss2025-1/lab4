#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController(Node):
    """
    Controller for automatically parking in proximity to a detected cone.
    Subscribes to cone position data and publishes driving commands.
    Compatible with both simulation environment and physical robot.
    """
    def __init__(self):
        super().__init__("parking_controller")

        self.declare_parameter("drive_topic")
        VEHICLE_CONTROL_TOPIC = self.get_parameter("drive_topic").value # Configured in launch file for sim/real robot

        self.drive_pub = self.create_publisher(AckermannDriveStamped, VEHICLE_CONTROL_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)

        self.create_subscription(ConeLocation, "/relative_cone", 
            self.relative_cone_callback, 1)

        self.target_distance = 0.5 # Target distance from cone in meters
        self.cone_x = 0
        self.cone_y = 0

        self.velocity_gain = 15
        self.steering_gain = .4

        self.recovery_mode = False

        self.get_logger().info("Auto Parking System Initialized")

    def relative_cone_callback(self, msg):
        self.cone_x = msg.x_pos
        self.cone_y = msg.y_pos
        command = AckermannDriveStamped()

        # Calculate distance and angle to cone
        cone_distance = np.linalg.norm([self.cone_x, self.cone_y])
        heading_angle = np.arctan2(self.cone_y, self.cone_x)

        velocity = 0.0
        
        # Calculate distance error from target
        distance_deviation = cone_distance - self.target_distance

        # Only move if we're not perfectly positioned
        if distance_deviation != 0 or heading_angle != 0:
            # Direction-aware steering calculation
            direction = 1 if distance_deviation > 0 else -1
            steering = direction * self.steering_gain * heading_angle
            
            # Only apply velocity if we're not close enough
            if np.absolute(distance_deviation) > .05:
                velocity = self.velocity_gain * distance_deviation 
            else:
                velocity = 0

        # Handle stuck condition
        if self.recovery_mode or (velocity < .5 and np.absolute(heading_angle) > 0.01):
            self.recovery_mode = True
            velocity = -1.0
            steering = -1 * self.steering_gain * heading_angle

        # Exit recovery mode when alignment improves
        if self.recovery_mode and np.absolute(heading_angle) < .03:
            self.recovery_mode = False
            if np.absolute(distance_deviation) > .05:
                velocity = self.velocity_gain * distance_deviation
            else:
                velocity = 0
            direction = 1 if distance_deviation > 0 else -1
            steering = direction * self.steering_gain * heading_angle

        # Apply velocity deadband and limits
        if np.absolute(velocity) < .25:
            velocity = 0.0
        else:
            velocity = np.clip(velocity, -1.0, 1.0)

        command.header.frame_id = 'base_link'
        command.drive.speed = velocity
        command.drive.steering_angle = steering

        self.get_logger().info(f"velocity: {command.drive.speed}")
        self.get_logger().info(f"steering: {command.drive.steering_angle}")

        self.drive_pub.publish(command)
        self.error_publisher(self.cone_x, self.cone_y, distance_deviation, cone_distance)

    def error_publisher(self, cone_x, cone_y, distance_error, total_distance):
        """
        Publishes error metrics between vehicle and cone.
        These metrics can be visualized using rqt_plot to evaluate controller performance.
        """
        error_data = ParkingError()

        error_data.x_error = distance_error * cone_x / total_distance
        error_data.y_error = distance_error * cone_y / total_distance
        error_data.distance_error = distance_error
        
        self.error_pub.publish(error_data)

def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()

if __name__ == '__main__':
    main()