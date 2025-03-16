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
        self.parking_distance = 0.75  # ~2.5 feet (increased from 0.5m)
        self.relative_x = 0
        self.relative_y = 0
        
        # Controller parameters
        self.wheelbase = 0.325  # Distance between front and rear axles (meters)
        self.max_speed = 0.7    # Maximum speed (m/s) - increased from 0.5
        self.min_speed = 0.2    # Minimum speed when moving (m/s) - increased from 0.15
        
        # Thresholds and gains
        self.angle_threshold = 0.1  # Radians (~5.7 degrees)
        self.distance_threshold = 0.1  # Meters - increased from 0.05 to reduce oscillation
        self.k_angular = 4.0    # Reduced from 5.0 for smoother turning
        self.k_distance = 0.5   # Distance gain - increased from 0.4 for faster approach
        
        # State machine states
        self.ALIGN = 0
        self.APPROACH = 1
        self.PARKED = 2
        self.state = self.ALIGN
        
        # Add a counter to force backing up periodically during alignment
        self.align_counter = 0
        self.force_backup_every = 5  # Force backup more frequently
        
        # Add a counter to prevent getting stuck in a loop
        self.stuck_counter = 0
        self.stuck_threshold = 30  # Number of frames to consider "stuck"
        self.last_distance = 0.0
        self.last_angle = 0.0
        
        # Maximum steering angle (radians)
        self.max_steering = 0.6  # Reduced to ~34 degrees for smoother turning
        
        # Add a frame counter for logging
        self.frame_counter = 0
        self.log_every_n_frames = 20
        
        # Add smoothing for controls
        self.prev_steering = 0.0
        self.prev_speed = 0.0
        self.steering_smoothing = 0.6  # Increased from 0.5 for even smoother steering
        self.speed_smoothing = 0.5     # Increased from 0.4 for smoother acceleration/deceleration
        
        # Minimum speed required for turning (to prevent jittering)
        self.min_turn_speed = 0.25  # Minimum speed when turning - increased from 0.2

        self.get_logger().info("Parking Controller Initialized")

    def smooth_control(self, target, previous, smoothing_factor):
        """
        Apply exponential smoothing to control values to prevent jittering
        """
        return previous + smoothing_factor * (target - previous)

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
        
        # Check if we're stuck (not making progress)
        is_stuck = False
        if self.frame_counter > 1:  # Skip the first frame
            distance_change = abs(distance_to_cone - self.last_distance)
            angle_change = abs(angle_to_cone - self.last_angle)
            
            # If we're not making significant progress
            if distance_change < 0.02 and angle_change < 0.02:
                self.stuck_counter += 1
                if self.stuck_counter > self.stuck_threshold and self.state != self.PARKED:
                    is_stuck = True
                    self.stuck_counter = 0
                    if should_log:
                        self.get_logger().info("Detected stuck condition, changing strategy")
            else:
                # Reset counter if we're making progress
                self.stuck_counter = 0
        
        # Save current values for next iteration
        self.last_distance = distance_to_cone
        self.last_angle = angle_to_cone
        
        # Initialize target speed and steering
        target_speed = 0.0
        steering_angle = 0.0
        
        # State machine for parking
        if self.state == self.ALIGN:
            # Increment the counter in ALIGN state
            self.align_counter += 1
            
            # In ALIGN state, prioritize facing the cone
            if should_log:
                self.get_logger().info(f"ALIGN: angle={angle_to_cone:.2f}, distance={distance_to_cone:.2f}")
            
            # Determine if we should back up - only when needed
            should_back_up = False
            
            # If we're stuck, try backing up to get unstuck
            if is_stuck:
                should_back_up = True
                if should_log:
                    self.get_logger().info("Backing up: Stuck condition")
            
            # Back up if cone is significantly off to the side or behind
            if abs(angle_to_cone) > 0.5:  # ~28 degrees
                should_back_up = True
                if should_log:
                    self.get_logger().info("Backing up: Large angle")
            
            # Back up if we're too close to the cone - adjusted to prevent oscillation
            if distance_to_cone < (self.parking_distance * 0.9):  # Only back up if significantly closer than target
                should_back_up = True
                if should_log:
                    self.get_logger().info("Backing up: Too close")
                
            # Force backing up periodically if we're still in align state and angle is still large
            if self.align_counter >= self.force_backup_every and abs(angle_to_cone) > 0.3:
                should_back_up = True
                if should_log:
                    self.get_logger().info("Backing up: Forced periodic backup")
                self.align_counter = 0
            
            # Calculate steering angle - smoother for alignment
            # Use a more gradual response curve for steering
            if abs(angle_to_cone) > 0.5:  # ~28 degrees
                # Less aggressive steering for larger angles
                angle_sign = 1 if angle_to_cone > 0 else -1
                steering_angle = angle_sign * self.max_steering * 0.7  # Reduced from 0.8
            elif abs(angle_to_cone) > 0.3:  # ~17 degrees
                # Medium steering for medium angles
                angle_sign = 1 if angle_to_cone > 0 else -1
                steering_angle = angle_sign * self.max_steering * 0.5  # Reduced from 0.6
            else:
                # Proportional for smaller angles
                steering_angle = self.k_angular * angle_to_cone
            
            # Limit to max steering
            steering_angle = max(min(steering_angle, self.max_steering), -self.max_steering)
            
            # Set speed based on alignment needs
            if should_back_up:
                # Back up when needed - smoother speed
                target_speed = -0.3  # Faster backing up - increased from -0.25
                
                # When backing up, reverse steering for better alignment
                if abs(angle_to_cone) > 0.3:  # ~17 degrees
                    # Reverse steering direction when backing up with angle
                    steering_angle = -steering_angle * 0.7  # Reduced from 0.8
            else:
                # Move forward with speed proportional to angle (slower for sharper turns)
                turn_factor = 1.0 - min(0.8, abs(angle_to_cone) / 0.8)  # Modified to maintain more speed in turns
                target_speed = self.min_turn_speed + (0.4 * turn_factor)  # Increased from 0.3
            
            # If we're well-aligned with the cone, transition to APPROACH
            if abs(angle_to_cone) < self.angle_threshold:
                # Only transition to APPROACH if we're not too close already
                if distance_to_cone > self.parking_distance:
                    self.state = self.APPROACH
                    self.align_counter = 0
                    if should_log:
                        self.get_logger().info("Aligned with cone, now approaching")
                else:
                    # If we're already close enough, go straight to PARKED
                    self.state = self.PARKED
                    if should_log:
                        self.get_logger().info("Already at parking distance, now parked")
        
        elif self.state == self.APPROACH:
            # In APPROACH state, maintain alignment while approaching/backing to the correct distance
            if should_log:
                self.get_logger().info(f"APPROACH: angle={angle_to_cone:.2f}, distance_error={distance_error:.2f}")
            
            # Smoother steering for maintaining alignment
            if abs(angle_to_cone) > 0.2:  # For angles > ~11 degrees
                angle_sign = 1 if angle_to_cone > 0 else -1
                steering_angle = angle_sign * self.max_steering * 0.6  # Less extreme
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
            
            # Determine speed based on distance error - smoother approach
            target_speed = self.k_distance * distance_error
            
            # Ensure minimum speed when moving, but scale down for turns
            if abs(target_speed) < self.min_speed and abs(distance_error) > self.distance_threshold:
                turn_factor = 1.0 - min(1.0, abs(angle_to_cone) / 0.4)  # Reduce speed more for turns
                min_approach_speed = self.min_speed * turn_factor
                target_speed = min_approach_speed if target_speed > 0 else -min_approach_speed
            
            # Add a small deadband to prevent oscillation around the target
            if abs(distance_error) < self.distance_threshold * 0.5:
                target_speed = 0.0
            
            # If we're at the right distance and still aligned, we're parked
            if abs(distance_error) < self.distance_threshold and abs(angle_to_cone) < self.angle_threshold:
                self.state = self.PARKED
                if should_log:
                    self.get_logger().info("Successfully parked!")
        
        elif self.state == self.PARKED:
            # In PARKED state, just stop
            if should_log:
                self.get_logger().info("PARKED")
            target_speed = 0.0
            steering_angle = 0.0
            
            # If we drift too far from the parking position, go back to ALIGN
            if abs(distance_error) > self.distance_threshold * 2 or abs(angle_to_cone) > self.angle_threshold * 2:
                self.state = self.ALIGN
                self.align_counter = 0
                if should_log:
                    self.get_logger().info("Drifted from parking position, realigning")
        
        # Apply smoothing to controls to prevent jittering
        speed = self.smooth_control(target_speed, self.prev_speed, self.speed_smoothing)
        
        # Apply more smoothing to steering than to speed to keep responsiveness
        # Use a non-linear smoothing for steering to reduce jitter
        steering_diff = steering_angle - self.prev_steering
        if abs(steering_diff) > 0.2:  # For large steering changes
            # Apply more aggressive smoothing
            steering_angle = self.prev_steering + (steering_diff * 0.4)
        else:
            # Normal smoothing for small changes
            steering_angle = self.smooth_control(steering_angle, self.prev_steering, self.steering_smoothing)
        
        # Save current controls for next iteration
        self.prev_speed = speed
        self.prev_steering = steering_angle
        
        # Ensure we're not trying to turn while stationary (causes jittering)
        if abs(speed) < 0.05 and abs(steering_angle) > 0.1:
            # If we need to turn but are nearly stopped, add a small speed
            # Use a speed that matches the direction we need to turn for more natural movement
            speed = self.min_turn_speed if (steering_angle * self.relative_y) > 0 else -self.min_turn_speed
        
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