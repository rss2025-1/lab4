#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController(Node):
    """
    A controller for parking in front of a cone using a pure pursuit approach.
    Listens for a relative cone location and publishes control commands.
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
        self.parking_distance = 0.6  # ~2 feet
        
        # Pure pursuit parameters
        self.wheelbase = 0.325  # Distance between front and rear axles (meters)
        self.max_speed = 1.1    # Maximum speed (m/s)
        self.min_speed = 0.5    # Minimum speed when moving (m/s)
        self.max_steering = 0.3  # Maximum steering angle (radians) - reduced from 0.4
        
        # Lookahead distance parameters
        self.min_lookahead = 0.5  # Minimum lookahead distance - increased from 0.3
        self.max_lookahead = 2.0  # Maximum lookahead distance - increased from 1.0
        self.lookahead_factor = 0.8  # Factor to multiply distance by for lookahead - increased from 0.4
        
        # Variables to store previous control values for smoothing
        self.prev_steering = 0.0
        self.prev_speed = 0.0
        
        # Smoothing factors
        self.steering_smoothing = 0.2  # Reduced from 0.3 for smoother steering
        self.speed_smoothing = 0.3
        
        # Threshold for considering the parking complete
        self.distance_threshold = 0.08  # Meters - increased from 0.05
        self.angle_threshold = 0.08     # Radians - increased from 0.05
        
        # Alignment parameters
        self.alignment_weight = 0.8  # Weight for alignment vs. distance - reduced from 2.0
        
        self.get_logger().info("Pure Pursuit Parking Controller Initialized")

    def smooth_control(self, target, previous, smoothing_factor):
        """
        Apply exponential smoothing to control values to prevent jittering
        """
        return previous + smoothing_factor * (target - previous)

    def calculate_lookahead_point(self, x, y, distance, angle):
        """
        Calculate a lookahead point based on the cone position and desired parking distance.
        Returns a target point (x, y) that the pure pursuit algorithm will aim for.
        """
        # Calculate the current distance to the cone
        current_distance = np.sqrt(x**2 + y**2)
        
        # Calculate how far we need to go (positive if we need to move forward, negative if backward)
        distance_to_go = current_distance - self.parking_distance
        
        # Calculate a dynamic lookahead distance based on how far we are from the target
        # For higher speeds, use a larger lookahead distance
        lookahead = min(self.max_lookahead, max(self.min_lookahead, 
                                               abs(distance_to_go) * self.lookahead_factor))
        
        # Increase the importance of alignment as we get closer to the target
        # But keep it lower to avoid aggressive turning
        alignment_factor = min(0.5, self.alignment_weight * (1.0 - min(1.0, abs(distance_to_go) / 1.0)))
        
        # If we're too close, we need to back up, so place the target point behind the robot
        if distance_to_go < -0.1:  # Only back up if we're significantly too close
            # Target point is behind the robot (negative x)
            target_x = -lookahead
            # Reduced lateral offset to avoid circling
            target_y = -y * 0.3
        else:
            # If we're close to the target distance but not aligned, prioritize going straight to the cone
            if abs(distance_to_go) < 0.3:
                # Direct approach to the cone
                target_x = x
                target_y = y
            else:
                # Normalize the vector to the cone
                norm = current_distance
                if norm < 0.001:  # Avoid division by zero
                    norm = 0.001
                    
                unit_x = x / norm
                unit_y = y / norm
                
                # Place the target point at the lookahead distance in the direction of the cone
                target_x = unit_x * lookahead
                target_y = unit_y * lookahead
                
                # Add a smaller lateral adjustment to improve alignment without causing circles
                lateral_adjustment = y * alignment_factor * 0.5
                target_y += lateral_adjustment
        
        return target_x, target_y

    def pure_pursuit_control(self, target_x, target_y):
        """
        Implement the pure pursuit control law to calculate steering angle.
        """
        # Calculate the curvature (1/radius) using the pure pursuit formula
        # For a target point in the robot's coordinate frame
        
        # The lateral distance to the target point
        lateral = target_y
        
        # The longitudinal distance to the target point
        longitudinal = target_x
        
        # Calculate the lookahead distance to the target
        lookahead_distance = np.sqrt(lateral**2 + longitudinal**2)
        
        # Avoid division by zero
        if lookahead_distance < 0.001:
            lookahead_distance = 0.001
            
        # Calculate the curvature (1/radius)
        # Reduce the curvature for higher speeds to avoid circling
        curvature = 2 * lateral / (lookahead_distance**2)
        
        # Scale down curvature at high speeds
        speed_scale = max(0.5, 1.0 - min(1.0, abs(self.prev_speed) / self.max_speed) * 0.5)
        curvature *= speed_scale
        
        # Calculate the steering angle using the bicycle model
        steering_angle = np.arctan(self.wheelbase * curvature)
        
        # Limit the steering angle
        steering_angle = np.clip(steering_angle, -self.max_steering, self.max_steering)
        
        return steering_angle

    def relative_cone_callback(self, msg):
        """
        Callback function for the relative cone position.
        Implements a pure pursuit approach for parking.
        """
        # Extract cone position
        x = msg.x_pos
        y = msg.y_pos
        
        # Calculate distance to the cone
        distance = np.sqrt(x**2 + y**2)
        
        # Calculate angle to the cone (in radians)
        angle = np.arctan2(y, x)
        
        # Calculate distance error (how far we are from desired parking distance)
        distance_error = distance - self.parking_distance
        
        # Calculate the target point for pure pursuit
        target_x, target_y = self.calculate_lookahead_point(x, y, distance, angle)
        
        # Calculate steering angle using pure pursuit
        target_steering = self.pure_pursuit_control(target_x, target_y)
        
        # Calculate speed based on distance error and alignment
        # Go slower as we get closer to the target
        if abs(distance_error) < self.distance_threshold and abs(angle) < self.angle_threshold:
            # We're at the target, stop
            target_speed = 0.0
            target_steering = 0.0
            self.get_logger().info("Parked successfully!")
        else:
            # Set speed proportional to distance error, with a sign to determine direction
            # Use a more direct approach with higher minimum speed
            if distance_error > 0:  # Need to move forward
                # Forward speed scales with distance but maintains minimum speed
                speed_factor = min(1.0, max(0.5, distance_error / 1.0))
                target_speed = self.min_speed + (self.max_speed - self.min_speed) * speed_factor
            elif distance_error < -0.1:  # Need to back up, but only if significantly too close
                # Backing up speed is constant and slower
                target_speed = -0.5
            else:
                # We're close enough, just focus on alignment
                target_speed = 0.0
            
            # Reduce speed when the angle is very large
            if abs(angle) > 0.5:  # Only slow down for large angles
                target_speed *= 0.7
            
            # Special case: if we're close but not aligned, prioritize alignment
            if abs(distance_error) < 0.2 and abs(angle) > 0.1:
                if target_speed > 0:
                    target_speed *= 0.5  # Reduce forward speed for alignment
                # Steering is already handled by pure pursuit
        
        # Apply stronger smoothing to steering to prevent oscillations
        smooth_steering = self.smooth_control(target_steering, self.prev_steering, self.steering_smoothing)
        smooth_speed = self.smooth_control(target_speed, self.prev_speed, self.speed_smoothing)
        
        # Store current controls for next iteration
        self.prev_speed = smooth_speed
        self.prev_steering = smooth_steering
        
        # Create drive command
        drive_cmd = AckermannDriveStamped()
        drive_cmd.drive.speed = smooth_speed
        drive_cmd.drive.steering_angle = smooth_steering
        
        # Publish drive command
        self.drive_pub.publish(drive_cmd)
        
        # Publish error for visualization
        error_msg = ParkingError()
        error_msg.x_error = x - self.parking_distance  # x_error is the forward error
        error_msg.y_error = y                         # y_error is the lateral error
        error_msg.distance_error = distance_error     # distance_error is the overall distance error
        self.error_pub.publish(error_msg)
        
        # Log information periodically
        if int(self.get_clock().now().nanoseconds / 1e8) % 10 == 0:  # Log every ~1 second
            self.get_logger().info(
                f"Distance: {distance:.2f}m, Angle: {angle:.2f}rad, "
                f"Target: ({target_x:.2f}, {target_y:.2f}), "
                f"Speed: {smooth_speed:.2f}m/s, Steering: {smooth_steering:.2f}rad"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ParkingController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
