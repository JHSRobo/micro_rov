#!/usr/bin/env python

from RPi import GPIO
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy


def changeSpeed(data):
  print(data.data)





if __name__ == "__main__":
  GPIO.setmode(GPIO.BOARD)
  switcher_pin = 32
  duty_cycle_pin = 33
  frequency = 1000
  GPIO.setup(switcher_pin, GPIO.OUT)
  GPIO.setup(duty_cycle_pin, GPIO.OUT) 
  pwm = GPIO.PWM(duty_cycle_pin, frequency)

  rospy.Subscriber("joy/joy1", Joy, changeSpeed)
