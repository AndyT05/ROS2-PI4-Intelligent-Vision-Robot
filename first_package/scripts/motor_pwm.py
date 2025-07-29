import RPi.GPIO as GPIO
import time

# Define GPIO pins
""" right_motor_a = 16
right_motor_b = 20
right_motor_en = 21

left_motor_a = 3
left_motor_b = 5
left_motor_en = 7 """

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
pwm_r = GPIO.PWM(right_motor_en, 1000)
pwm_l = GPIO.PWM(left_motor_en, 1000)

pwm_r.start(75)  # Start at 75% speed
pwm_l.start(75)

# Movement Functions
def forward(sec):
    print("Moving forward")
    GPIO.output(right_motor_a, GPIO.HIGH)
    GPIO.output(right_motor_b, GPIO.LOW)
    GPIO.output(left_motor_a, GPIO.HIGH)
    GPIO.output(left_motor_b, GPIO.LOW)
    time.sleep(sec)

def reverse(sec):
    print("Moving reverse")
    GPIO.output(right_motor_a, GPIO.LOW)
    GPIO.output(right_motor_b, GPIO.HIGH)
    GPIO.output(left_motor_a, GPIO.LOW)
    GPIO.output(left_motor_b, GPIO.HIGH)
    time.sleep(sec)

def left(sec):
    print("Turning left")
    GPIO.output(right_motor_a, GPIO.HIGH)
    GPIO.output(right_motor_b, GPIO.LOW)
    GPIO.output(left_motor_a, GPIO.LOW)
    GPIO.output(left_motor_b, GPIO.HIGH)
    time.sleep(sec)

def right(sec):
    print("Turning right")
    GPIO.output(right_motor_a, GPIO.LOW)
    GPIO.output(right_motor_b, GPIO.HIGH)
    GPIO.output(left_motor_a, GPIO.HIGH)
    GPIO.output(left_motor_b, GPIO.LOW)
    time.sleep(sec)

def stop():
    pwm_r.ChangeDutyCycle(0)
    pwm_l.ChangeDutyCycle(0)

def exit_():
    stop()
    GPIO.cleanup()

# Main function
def main():
    forward(2)
    reverse(2)
    left(2)
    right(2)
    stop()
    exit_()

if __name__ == '__main__':
    main()
