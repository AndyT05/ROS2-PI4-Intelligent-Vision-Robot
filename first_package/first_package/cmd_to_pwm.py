'''
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO

class CmdToPwm(Node):

    def __init__(self):
        super().__init__('cmd_to_pwm')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_to_pwm_callback,
            10
        )
        right_motor_a = 16
        right_motor_b = 20
        right_motor_en = 21

        left_motor_a = 17
        left_motor_b = 27
        left_motor_en = 22
        
        # Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(right_motor_a, GPIO.OUT)
        GPIO.setup(right_motor_b, GPIO.OUT)
        GPIO.setup(right_motor_en, GPIO.OUT)
        GPIO.setup(left_motor_a, GPIO.OUT)
        GPIO.setup(left_motor_b, GPIO.OUT)
        GPIO.setup(left_motor_en, GPIO.OUT)

        # PWM setup
        self.pwm_r = GPIO.PWM(right_motor_en, 1000)
        self.pwm_l = GPIO.PWM(left_motor_en, 1000)

        self.pwm_r.start(75)  # Start at 75% speed
        self.pwm_l.start(75)
        
        self.mr_a = right_motor_a
        self.mr_b = right_motor_b

        self.ml_a = left_motor_a
        self.ml_b = left_motor_b
        #self.subscription  # prevent unused variable warning

    def cmd_to_pwm_callback(self, msg):
        #pwm_value = msg.data  # Process the command to PWM conversion
        #pwm_msg = Int16()
        #pwm_msg.data = pwm_value
        #self.publisher_.publish(pwm_msg)
        #self.get_logger().info('Publishing PWM value: %d' % pwm_value)
        right_wheel_vel = (msg.linear.x + msg.angular.z) /2
        left_wheel_vel = (msg.linear.x - msg.angular.z) /2
        print(right_wheel_vel, " / ", left_wheel_vel)

def main(args=None):
    rclpy.init(args=args)

    cmd_to_pwm = CmdToPwm()

    rclpy.spin(cmd_to_pwm)

    # Destroy the node explicitly
    cmd_to_pwm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO


class CmdVelListener(Node):
    def __init__(self):
        super().__init__('cmd_vel_listener')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )
        right_motor_a = 16
        right_motor_b = 20
        right_motor_en = 21

        left_motor_a = 17
        left_motor_b = 27
        left_motor_en = 22
        
        # Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(right_motor_a, GPIO.OUT)
        GPIO.setup(right_motor_b, GPIO.OUT)
        GPIO.setup(right_motor_en, GPIO.OUT)
        GPIO.setup(left_motor_a, GPIO.OUT)
        GPIO.setup(left_motor_b, GPIO.OUT)
        GPIO.setup(left_motor_en, GPIO.OUT)

        # PWM setup
        self.pwm_r = GPIO.PWM(right_motor_en, 1000)
        self.pwm_l = GPIO.PWM(left_motor_en, 1000)

        self.pwm_r.start(75)  # Start at 75% speed
        self.pwm_l.start(75)
        
        self.mr_a = right_motor_a
        self.mr_b = right_motor_b

        self.ml_a = left_motor_a
        self.ml_b = left_motor_b

    def listener_callback(self, msg):
        right_wheel_vel = (msg.linear.x + msg.angular.z) / 2
        left_wheel_vel = (msg.linear.x - msg.angular.z) / 2
        print(right_wheel_vel, "/", left_wheel_vel)
        
        GPIO.output(self.mr_a, right_wheel_vel > 0)
        GPIO.output(self.mr_b, right_wheel_vel < 0)
        GPIO.output(self.ml_a, left_wheel_vel > 0)
        GPIO.output(self.ml_b, left_wheel_vel < 0)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


