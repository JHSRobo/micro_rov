#!/usr/bin/env python

from RPi import GPIO
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy


def changeSpeed(values):
  print(values)
  def callback(data):
    print(data.axes[3])
    if abs(values[0] - abs(data.axes[3])) < 0.1: 
      values[1].ChangeDutyCycle(round(abs(data.axes[3]), 2)) 
      values[0] = abs(data.axes[3])
  return callback


if __name__ == "__main__":
  GPIO.setmode(GPIO.BOARD)
  switcher_pin = 32
  duty_cycle_pin = 33
  frequency = 1000
  GPIO.setup(switcher_pin, GPIO.OUT)
  GPIO.setup(duty_cycle_pin, GPIO.OUT) 
  pwm = GPIO.PWM(duty_cycle_pin, frequency)
  pwm.start(0)
  previousValue = 0

  rospy.init_node("micro_rov")
  rospy.Subscriber("/joy/joy1", Joy, changeSpeed([previousValue, pwm]))
  rospy.spin()

