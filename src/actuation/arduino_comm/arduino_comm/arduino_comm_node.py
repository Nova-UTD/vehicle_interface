import os
import random
import time
import io

import rclpy
from rclpy.node import Node
from navigator_msgs.msg import VehicleControl
import serial

class ArduinoSenderNode(Node):
    def __init__(self):
        super().__init__('arduino_sender')
        
        # Initialize serial connection
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        
        # Subscription to VehicleControl messages
        self.vehicle_command_sub = self.create_subscription(
            VehicleControl, '/vehicle/control', self.commandCb, 1
        )
        
        self.get_logger().info('Arduino Sender Node started, serial port opened')

    def commandCb(self, msg):
        steer = msg.steer
        angle = int((steer + 1) * 90)
        
        command = f'SERVO:{angle}\n'  # Format for Arduino to parse
        self.ser.write(command.encode())
        
        # Log the command sent
        self.get_logger().info(f'Sent command to Arduino: {command.strip()}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSenderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Close serial connection
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
