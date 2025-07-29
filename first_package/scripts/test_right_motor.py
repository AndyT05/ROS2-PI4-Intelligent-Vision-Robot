import RPi.GPIO as GPIO
import time

# Right motor GPIO pins
right_motor_a = 16
right_motor_b = 20
right_motor_en = 21

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(right_motor_a, GPIO.OUT)
GPIO.setup(right_motor_b, GPIO.OUT)
GPIO.setup(right_motor_en, GPIO.OUT)

# PWM setup for right motor
pwm_r = GPIO.PWM(right_motor_en, 1000)
pwm_r.start(75)  # Start at 75% speed

# Movement functions for right motor only
def right_motor_forward(sec):
    print("Right motor moving forward")
    GPIO.output(right_motor_a, GPIO.HIGH)
    GPIO.output(right_motor_b, GPIO.LOW)
    time.sleep(sec)

def right_motor_reverse(sec):
    print("Right motor moving reverse")
    GPIO.output(right_motor_a, GPIO.LOW)
    GPIO.output(right_motor_b, GPIO.HIGH)
    time.sleep(sec)

def stop():
    print("Stopping right motor")
    pwm_r.ChangeDutyCycle(0)

def exit_():
    stop()
    GPIO.cleanup()

# Main test function
def main():
    right_motor_forward(2)
    time.sleep(1)
    right_motor_reverse(2)
    stop()
    exit_()

if __name__ == '__main__':
    main()
