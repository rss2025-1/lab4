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

        self.parking_distance = 1 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

        self.kp_drive = 15
        self.kp_angle = .4

        self.stuck = False

        self.get_logger().info("Parking Controller Initialized")

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd

        #################################
        #gets distance from cone
        distance_from_cone = np.linalg.norm([self.relative_x, self.relative_y])

        #gets relative angle from cone
        cone_angle = np.arctan2(self.relative_y, self.relative_x)

        car_speed = 0.0
        
        distance_error = distance_from_cone - self.parking_distance

        if distance_error != 0 or cone_angle != 0:
            cone_angle = (1 if distance_error > 0 else -1) * self.kp_angle * cone_angle
            car_speed = self.kp_drive * distance_error if np.absolute(distance_error) > .05 else 0

        if self.stuck or (car_speed < .5 and np.absolute(cone_angle) > 0.01):
            self.stuck = True
            car_speed = -1.0
            cone_angle = np.arctan2(self.relative_y, self.relative_x)
            cone_angle = -1 * self.kp_angle * cone_angle

        if self.stuck and np.absolute(cone_angle) < .03:
            self.stuck = False
            car_speed = self.kp_drive * distance_error if np.absolute(distance_error) > .05 else 0
            cone_angle = np.arctan2(self.relative_y, self.relative_x)
            cone_angle = (1 if distance_error > 0 else -1) * self.kp_angle * cone_angle

        car_speed = np.clip(car_speed, -1.0, 1.0) if np.absolute(car_speed) >= .25 else 0.0

        drive_cmd.header.frame_id = 'base_link'
        drive_cmd.drive.speed = car_speed
        drive_cmd.drive.steering_angle = cone_angle

        self.get_logger().info(f"car_speed: {drive_cmd.drive.speed}")
        self.get_logger().info(f"car_angle: {drive_cmd.drive.steering_angle}")

        self.drive_pub.publish(drive_cmd)
        self.error_publisher(self.relative_x, self.relative_y, distance_error, distance_from_cone)

    def error_publisher(self, relative_x, relative_y, distance_e, distance_from_cone):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)

        #################################
        error_msg.x_error = distance_e * relative_x / distance_from_cone
        error_msg.y_error = distance_e * relative_y / distance_from_cone
        error_msg.distance_error = distance_e
        
        self.error_pub.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()

if __name__ == '__main__':
    main()