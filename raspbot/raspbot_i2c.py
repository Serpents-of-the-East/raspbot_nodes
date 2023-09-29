import smbus
import time
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped

class Car:
    def __init__(self):
        self._addr = 0x16
        self._device = smbus.SMBus(1)

    def __write_u8(self, register, data):
        try:
            self._device.write_byte_data(self._addr, register, data)
        except:
            print('write_u8 error')

    def __write_register(self, register):
        try:
            self._device.write_byte(self._addr, register)
        except:
            print('write_register error')

    def __write_array(self, register, data):
        try:
            self._device.write_i2c_block_data(self._addr, register, data)
        except:
            print('write_array error')

    def control_car(self, left, right):
        """
        left: int (-255, 255)
        right: int (-255, 255)

        sets the motor with the speed given (not actually in unit, just a power amount)
        """
        register = 0x01
        left_direction = 0 if left < 0 else 1
        right_direction = 0 if right < 0 else 1

        if left < 0:
            left *= -1
        if right < 0:
            right *= -1

        data = [left_direction, left, right_direction, right]
        self.__write_array(register, data)

    def stop(self):
        register = 0x02
        self.__write_u8(register, 0x00)

    def set_servo(self, servo_id, angle):
        register = 0x03
        if angle < 0:
            angle = 0
        elif angle > 180:
            angle = 180
        data = [servo_id, angle]
        self.__write_array(register, data)

ROBOT_WIDTH = 0.16
WHEEL_RADIUS = 0.065

class RaspbotSubscriber(Node):
    def __init__(self):
        super().__init__('raspbot_i2c')
        self.car = Car()
        self.motor_sub = self.create_subscription(Twist, '/cmd_vel', self.motor_callback, 10)
        self.odom_pub = self.publisher = self.create_publisher(PoseStamped, 'robot_pose', 10)
        self.v_left = 0
        self.v_right = 0
        self.last_time = time.time()
        self.current_x = 0
        self.current_z = 0
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.current_rotation = 0
        self.angular_vel = 0
        self.linear_vel = 0
    
    def vel_to_power(self, vel):
        if abs(vel) < 0.1:
            return 0
        if vel > 1.3:
            return 230
        if vel < -1.3:
            return -230
        scaled = (vel - 0.1) / 1.2
        return int(scaled * 200 + 30)
    
    def motor_callback(self, msg):
        linear_vel_forward = msg.linear.x
        turn_vel = msg.angular.z
        self.angular_vel = turn_vel
        self.linear_vel = linear_vel_forward
        
        self.v_left = (linear_vel_forward - turn_vel * ROBOT_WIDTH / 2) / WHEEL_RADIUS
        self.v_right = (linear_vel_forward + turn_vel * ROBOT_WIDTH / 2) / WHEEL_RADIUS
        
        power_left = self.vel_to_power(self.v_left)
        power_right = self.vel_to_power(self.v_right)
        self.car.control_car(power_left, power_right)
    
    def timer_callback(self):
        elapsed = time.time() - self.last_time
        d_forward = self.linear_vel * elapsed
        d_x = math.cos(self.current_rotation) * d_forward
        d_z = math.sin(self.current_rotation) * d_forward
        d_theta = self.angular_vel * elapsed
        self.current_x += d_x
        self.current_z += d_z
        self.current_rotation += d_theta
        pose = PoseStamped()
        pose.pose.position.x = self.current_x
        pose.pose.position.z = self.current_z
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = '/world'
        self.odom_pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    subscriber = RaspbotSubscriber()
    
    try:
        rclpy.spin(subscriber)
    except Exception as e:
        print(e)
        subscriber.car.stop()
    subscriber.destroy_node()
    rclpy.shutdown()