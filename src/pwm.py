#!/usr/bin/env python

from RPi import GPIO
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

class PWM:
    def __init__(self, switcher_pin, duty_cycle_pin, frequency):
        self.switcher_pin = switcher_pin
        self.duty_cycle_pin = duty_cycle_pin
        self.frequency = frequency
        self.gpio_setup(GPIO.BOARD)
        self.direction = 1
        self.previousValue = 0

    def gpio_setup(self, mode):
        GPIO.setwarnings(False)
        GPIO.setmode(mode)
        GPIO.setup(self.switcher_pin, GPIO.OUT)
        GPIO.setup(self.duty_cycle_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.duty_cycle_pin, self.frequency)
        self.pwm.start(0)

    def callback(self, data):
    # Created a 0.1 threshold before changing the speed
        if abs(self.previousValue - data.axes[3]) > 0.1:
            self.pwm.ChangeDutyCycle(abs(data.axes[3]) * 100)
            # switch the direction if the previous number and this number have different signs using xor
            if self.direction * data.axes[3] < 0:
              self.direction = -1 if data.axes[3] < 0 else 1
              rospy.loginfo('micro_rov: switched direction to {}'.format('forwards' if data.axes[3] > 0 else 'backwards'))
              GPIO.output(self.switcher_pin, int(data.axes[3] > 0))
            self.previousValue = data.axes[3]

if __name__ == "__main__":
  pwm = PWM(32, 33, 1000)
  rospy.init_node("micro_rov")
  rospy.Subscriber("/joy/joy1", Joy, pwm.callback)
  rospy.spin()
  GPIO.cleanup()
