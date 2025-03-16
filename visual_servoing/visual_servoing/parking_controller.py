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
        self.max_speed = 0.5    # Maximum speed (m/s)
        self.min_speed = 0.1    # Minimum speed when moving (m/s)
        
        # Thresholds and gains
        self.angle_threshold = 0.1  # Radians (~5.7 degrees)
        self.distance_threshold = 0.05  # Meters
        self.k_angular = 8.0    # Extremely aggressive steering angle gain
        self.k_distance = 0.4   # Distance gain
        
        # State machine states
        self.ALIGN = 0
        self.APPROACH = 1
        self.PARKED = 2
        self.state = self.ALIGN
        
        # Add a counter to force backing up periodically during alignment
        self.align_counter = 0
        self.force_backup_every = 5  # Force backup more frequently
        
        # Maximum steering angle (radians)
        self.max_steering = 0.8  # Increased to ~46 degrees for very aggressive turning
        
        # Add a frame counter for logging
        self.frame_counter = 0
        self.log_every_n_frames = 20
        
        self.get_logger().info("Parking Controller Initialized")

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        self.get_logger().info("Relative cone position: x={:.2f}, y={:.2f}".format(self.relative_x, self.relative_y))
        drive_cmd = AckermannDriveStamped()

        # Calculate distance to the cone
        distance_to_cone = np.sqrt(self.relative_x**2 + self.relative_y**2)
        
        # Calculate angle to the cone (in radians)
        # arctan2 gives the angle in the correct quadrant
        angle_to_cone = np.arctan2(self.relative_y, self.relative_x)
        
        # Calculate distance error (how far we are from desired parking distance)
        distance_error = distance_to_cone - self.parking_distance
        
        # Increment frame counter
        self.frame_counter += 1
        
        # Only log every n frames
        should_log = (self.frame_counter % self.log_every_n_frames) == 0
        
        # Log current state for debugging (only every n frames)
        if should_log:
            self.get_logger().info(f"State: {self.state}, Angle: {angle_to_cone:.2f}, Distance: {distance_to_cone:.2f}")
        
        # State machine for parking
        if self.state == self.ALIGN:
            # Increment the counter in ALIGN state
            self.align_counter += 1
            
            # In ALIGN state, prioritize facing the cone
            if should_log:
                self.get_logger().info(f"ALIGN: angle={angle_to_cone:.2f}, distance={distance_to_cone:.2f}")
            
            # Determine if we should back up - only when needed
            should_back_up = False
            
            # Back up if cone is significantly off to the side or behind
            if abs(angle_to_cone) > 0.5:  # ~28 degrees
                should_back_up = True
                if should_log:
                    self.get_logger().info("Backing up: Large angle")
            
            # Back up if we're too close to the cone
            if distance_to_cone < self.parking_distance * 1.2:
                should_back_up = True
                if should_log:
                    self.get_logger().info("Backing up: Too close")
                
            # Force backing up periodically if we're still in align state and angle is still large
            if self.align_counter >= self.force_backup_every and abs(angle_to_cone) > 0.3:
                should_back_up = True
                if should_log:
                    self.get_logger().info("Backing up: Forced periodic backup")
                self.align_counter = 0
            
            # Calculate steering angle - super aggressive for alignment
            # Use a highly non-linear function for more aggressive turning
            angle_sign = 1 if angle_to_cone > 0 else -1
            
            # Exponential steering response for more aggressive turning
            if abs(angle_to_cone) > 0.3:  # ~17 degrees
                # Very aggressive steering for larger angles
                steering_angle = angle_sign * self.max_steering
            else:
                # Still aggressive but proportional for smaller angles
                steering_angle = self.k_angular * angle_to_cone
            
            # Limit to max steering
            steering_angle = max(min(steering_angle, self.max_steering), -self.max_steering)
            
            # Set speed based on alignment needs
            if should_back_up:
                # Back up when needed
                speed = -0.3
                
                # When backing up, reverse steering for better alignment
                if abs(angle_to_cone) > 0.3:  # ~17 degrees
                    # Reverse steering direction when backing up with angle
                    steering_angle = -steering_angle
            else:
                # Move forward slowly if not backing up
                # Use slower speed for sharper turns
                if abs(angle_to_cone) > 0.5:  # ~28 degrees
                    speed = 0.1  # Very slow for sharp turns
                else:
                    speed = 0.2  # Faster for smaller angles
            
            # If we're well-aligned with the cone, transition to APPROACH
            if abs(angle_to_cone) < self.angle_threshold:
                self.state = self.APPROACH
                self.align_counter = 0
                if should_log:
                    self.get_logger().info("Aligned with cone, now approaching")
        
        elif self.state == self.APPROACH:
            # In APPROACH state, maintain alignment while approaching/backing to the correct distance
            if should_log:
                self.get_logger().info(f"APPROACH: angle={angle_to_cone:.2f}, distance_error={distance_error:.2f}")
            
            # Still maintain alignment with aggressive steering
            if abs(angle_to_cone) > 0.2:  # For angles > ~11 degrees
                angle_sign = 1 if angle_to_cone > 0 else -1
                steering_angle = angle_sign * self.max_steering * 0.8  # Slightly less extreme
            else:
                steering_angle = self.k_angular * angle_to_cone
                
            # Limit steering
            steering_angle = max(min(steering_angle, self.max_steering), -self.max_steering)
            
            # If we've lost significant alignment, go back to ALIGN state
            if abs(angle_to_cone) > 0.3:  # ~17 degrees
                self.state = self.ALIGN
                self.align_counter = 0
                if should_log:
                    self.get_logger().info("Lost alignment, going back to alignment phase")
                return  # Exit early to immediately handle the alignment
            
            # Determine speed based on distance error
            speed = self.k_distance * distance_error
            
            # Ensure minimum speed when moving
            if abs(speed) < self.min_speed and abs(distance_error) > self.distance_threshold:
                speed = self.min_speed if speed > 0 else -self.min_speed
            
            # If we're at the right distance and still aligned, we're parked
            if abs(distance_error) < self.distance_threshold and abs(angle_to_cone) < self.angle_threshold:
                self.state = self.PARKED
                if should_log:
                    self.get_logger().info("Successfully parked!")
        
        elif self.state == self.PARKED:
            # In PARKED state, just stop
            if should_log:
                self.get_logger().info("PARKED")
            speed = 0.0
            steering_angle = 0.0
            
            # If we drift too far from the parking position, go back to ALIGN
            if abs(distance_error) > self.distance_threshold * 2 or abs(angle_to_cone) > self.angle_threshold * 2:
                self.state = self.ALIGN
                self.align_counter = 0
                if should_log:
                    self.get_logger().info("Drifted from parking position, realigning")
        
        # Limit speed
        speed = max(min(speed, self.max_speed), -self.max_speed)
        
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