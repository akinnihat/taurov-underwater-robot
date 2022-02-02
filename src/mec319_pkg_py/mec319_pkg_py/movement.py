#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pymavlink import mavutil

from mec319_interfaces.msg import Distance
from mec319_interfaces.msg import ObjectCoordinates

class MovementNode(Node): 
    def __init__(self):
        super().__init__("movement_node")

        self.master = mavutil.mavlink_connection("/dev/ttyACM0", 115600)
        self.arm_auv()

        self.sonar_client_ = self.create_subscription(
            Distance, "distance_data", self.callback_sonar_data, 10)

        self.object_status_client_ = self.create_subscription(
            ObjectCoordinates, "object_status", self.callback_object_status, 10)
        
        self.object_detected = False
        self.wall_threshold = 60
        self.counter_for_search = 0
        self.search_forwards = 30
        self.search_right = self.search_forwards*2
        self.search_left = self.search_forwards*3

        self.get_logger().info("Movement Node has been started.")

    def callback_sonar_data(self, msg):
        if msg.distance < self.wall_threshold and not self.object_detected:
            self.get_logger().info("\nWALL! Turn around.")
            self.set_rc_channel_pwm(6, 1600)
        else: self.get_logger().info("Searching for object.")

    def callback_object_status(self, msg):
        if msg.is_found and msg.x_coordinate != 0 and msg.y_coordinate != 0:
            self.object_detected = True
            #self.get_logger().info("X Coordinate: " + str(msg.x_coordinate) + "\n" + 
            #                      "Y Coordinate: " + str(msg.y_coordinate) + "\n" )
            self.adjust_movement(msg.x_coordinate, msg.y_coordinate)
            if msg.radius >= 100:
                self.disarm_auv() 

        else:
            self.object_detected = False
            self.search_for_object()

    def search_for_object(self):
            if self.counter_for_search <= self.search_forwards:
                self.set_rc_channel_pwm(5, 1700)
                self.get_logger().info("\nCounter for search: " + str(self.counter_for_search) + "\nGo forwards.")
            elif self.search_forwards < self.counter_for_search <= self.search_right:    
                self.set_rc_channel_pwm(6, 1400)
                self.get_logger().info("\nCounter for search: " + str(self.counter_for_search) + "\nTurn right.")
            elif self.search_right < self.counter_for_search <= self.search_left:
                self.set_rc_channel_pwm(6, 1600)
                self.get_logger().info("\nCounter for search: " + str(self.counter_for_search) + "\nTurn left.")
            else:
                self.counter_for_search = 0
            self.counter_for_search += 1

    def adjust_movement(self, object_x, object_y):
        if 0 <= object_x <= 280:
            self.set_rc_channel_pwm(4, 1300) # Go Left
            self.get_logger().info("Go Left")
        elif object_x > 360:
            self.set_rc_channel_pwm(4, 1700) # Go Right
            self.get_logger().info("Go Right")
        else:
            if 0 <= object_y <= 210:
                self.set_rc_channel_pwm(3, 1800) # Go Up
                self.get_logger().info("Go Up")
            elif object_y > 270:
                self.set_rc_channel_pwm(3, 1200) # Go Down
                self.get_logger().info("Go Down")
            else: 
                self.set_rc_channel_pwm(5, 1700) # Go Forwards
                self.get_logger().info("Go Forwards")

    # Pymavlink Methods
    def resetPWMs(self):
        for i in range():
            self.set_rc_channel_pwm(i, 1500)

    def set_rc_channel_pwm(self, channel_id, pwm=1500):

        if channel_id < 1 or channel_id > 8:
            print("Channel does not exist.")
            return

        rc_channel_values = [1500 for _ in range(8)]
        rc_channel_values[channel_id - 1] = pwm
        self.master.mav.rc_channels_override_send(
            self.master.target_system,                
            self.master.target_component,             
            *rc_channel_values)    

    def arm_auv(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)
    
    def disarm_auv(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0)
        


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()