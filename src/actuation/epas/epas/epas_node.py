"""
ROS2 node for EPAS interface.

PID in EPAS is entirely experimental and values are subject to change during vehicle testing.
"""

import can
from can.bus import BusState
import math

from navigator_msgs.msg import VehicleControl, Mode
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.node import Node
from dataclasses import dataclass
from std_msgs.msg import Float32
import numpy as np


@dataclass
class EpasState():
    duty: int
    current: int
    supply_voltage_mv: int
    temperature: int
    angle: int
    errors: int


class EpasNode(Node):

    def __init__(self):
        super().__init__('epas_node')

        # These limits are torque bounds for values sent to the EPAS motor.
        # If steering wheel hits one end and bounces back, then it means you need to tighten the bounds.
        # In theory, these torque bounds should be from [0, 255]. 
        # They were originally 19 and 230,respectively, before we changed them to -11 and 255 during testing.
        # Feel free to change these values experimentally.
        self.limit_left: int = -11
        self.limit_right: int = 255

        self.bus = None

        # The number of past errors that the integral term will consider.
        self.past_errors = np.zeros(10)

        self.command_sub = self.create_subscription(
            VehicleControl, '/vehicle/control', self.commandCb, 1)
        self.cmd_msg = None
        self.cmd_timer = self.create_timer(.01, self.vehicleControlCb)
        self.current_angle = 0.0

        # The previous normalized angle needed to calculate the derivative term for PID.
        self.prev_angle_normalized = 0.0

        self.clock = Clock().clock
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 10)
        self.target_angle = 0.0
        self.cached_msg1 = None

        self.status = DiagnosticStatus()
        self.status_pub = self.create_publisher(
            DiagnosticStatus, '/node_statuses', 1)
        
        self.current_angle_pub = self.create_publisher(Float32, '/epas/current_angle', 10)

        self.clock = Clock().clock
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 10)

        self.current_mode = Mode.DISABLED
        self.current_mode_sub = self.create_subscription(
            Mode, '/guardian/mode', self.currentModeCb, 1)

        self.retry_connection_timer = self.create_timer(1.0, self.connectToBus)

    def connectToBus(self):
        self.bus: can.interface.Bus
        if self.bus is not None and self.bus.state == can.bus.BusState.ACTIVE:
            return
        channel='/dev/serial/by-id/usb-Protofusion_Labs_CANable_1205aa6_https:__github.com_normaldotcom_cantact-fw_001500174E50430520303838-if00'
        try:
            self.bus = can.interface.Bus(
                bustype='slcan', channel=channel, bitrate=500000, receive_own_messages=True)
            self.get_logger().info(f"EPAS connected to CAN bus on {channel}")
        except can.exceptions.CanInitializationError as e:
            self.status.level = DiagnosticStatus.ERROR
            self.status.message = f"EPAS failed to connect to bus. {e}"
            self.status_pub.publish(self.status)
        return

    def currentModeCb(self, msg: Mode):
        self.current_mode = msg.mode

    def initStatusMsg(self) -> DiagnosticStatus:
        status = DiagnosticStatus()

        status.name = self.get_name()

        stamp = KeyValue()
        stamp.key = 'stamp'
        stamp.value = str(self.clock.sec+self.clock.nanosec*1e-9)
        status.values.append(stamp)

        status.level = DiagnosticStatus.OK

        return status

    def clockCb(self, msg: Clock):
        self.clock = msg.clock

    def commandCb(self, msg: VehicleControl):
        self.cmd_msg = msg

    def parseIncomingMessages(self, msg1_data: bytearray, msg2_data: bytearray) -> EpasState:
        torque = msg1_data[0]
        duty = msg1_data[1]
        current = msg1_data[2]
        supply_voltage = msg1_data[3]
        switch_pos = msg1_data[4]
        temp = msg1_data[5]
        torque_a = msg1_data[6]
        torque_b = msg1_data[7]

        angle = msg2_data[0]
        analog_c1 = msg2_data[1]
        analog_c2 = msg2_data[2]
        selected_map = msg2_data[3]
        errors = msg2_data[4]
        dio_bitfield = msg2_data[5]
        status_bitfield = msg2_data[6]
        limit_bitfield = msg2_data[7]

        current_state = EpasState(
            duty, current, supply_voltage, temp, angle, errors)

        return current_state

    def sendCommand(self, target, bus):
        current_angle_normalized = (
            (self.current_angle-self.limit_left)/(self.limit_right-self.limit_left)*2)-1
        e = target - current_angle_normalized  # Error = target - current

        # Integral and derivative term are HIGHLY INCOMPLETE/EXPERIMENTAL.
        # Last changed during Spring'24 testing where we were unable to get steering to work with 
        # PID epas, RTP planning, and a Pure Pursuit Controller.
        Kp = 1.0
        Ki = 0.0
        Kd = 0.0

        # P
        proportional_term = e * Kp

        # D
        derivative_term = (current_angle_normalized - self.prev_angle_normalized) * Kd
        self.prev_angle_normalized = current_angle_normalized

        INTEGRAL_CAP = .3
        TORQUE_LIMIT = 200

        # Shift past errors to the left and replace the last value.
        self.past_errors = np.roll(self.past_errors, -1)
        self.past_errors[-1] = e

        # I
        past_error_sum = np.sum(self.past_errors)
        integral_term = past_error_sum * Ki

        if(integral_term > INTEGRAL_CAP):
            integral_term = INTEGRAL_CAP
        elif(integral_term < -1 * INTEGRAL_CAP):
            integral_term = -1 * INTEGRAL_CAP

        # Power is an abstract value from [-1., 1.], where -1 is hard push left
        power = proportional_term + integral_term - derivative_term

        torqueA: int = min(TORQUE_LIMIT, max(0, math.ceil((power+1) * (255/2))))
        torqueB: int = 255-torqueA

        data = [0x03, torqueA, torqueB, 0x00, 0x00, 0x00, 0x00, 0x00]
        message = can.Message(arbitration_id=0x296, data=data,
                              check=True, is_extended_id=False)
        bus.send(message, timeout=0.2)
    
    def vehicleControlCb(self):
        self.status = self.initStatusMsg()


        if self.bus is None or self.bus.state == BusState.ERROR:
            # Status already generated by connectToBus

            # self.status.level = DiagnosticStatus.ERROR
            # self.status.message = "CAN bus was in error state."
            # self.status_pub.publish(self.status)
            return
        elif self.cmd_msg == None:
            self.status.level = DiagnosticStatus.WARN
            self.status.message = "EPAS command message not received."
            self.status_pub.publish(self.status)
            return

        if self.cmd_msg != None:
            self.target_angle = self.cmd_msg.steer

        response_msg = self.bus.recv(0.1)
        if (response_msg is None):
            self.get_logger().error("No message received")
        elif (response_msg.arbitration_id == 0x290):
            self.cached_msg1 = response_msg.data
        elif (response_msg.arbitration_id == 0x292):
            if (self.cached_msg1 is None):
                return
            msg1 = self.cached_msg1
            msg2 = response_msg.data
            current_state = self.parseIncomingMessages(msg1, msg2)
            self.current_angle = current_state.angle
            self.publish_current_angle() # Publishes current angle

        if self.current_mode == Mode.MANUAL or \
                self.current_mode == Mode.AUTO:
            self.sendCommand(self.target_angle, self.bus)
        self.status_pub.publish(self.status)

    def publish_current_angle(self):
        angle_msg = Float32()
        angle_msg.data = ((self.current_angle-self.limit_left)/(self.limit_right-self.limit_left)*2)-1 # Normalized angle [-1, 1]
        self.current_angle_pub.publish(angle_msg)

def main(args=None):
    rclpy.init(args=args)
    epas_node = EpasNode()
    rclpy.spin(epas_node)
    epas_node.destroy_node()
    rclpy.shutdown()