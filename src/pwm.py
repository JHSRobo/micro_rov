#!/usr/bin/env python

from RPi import GPIO
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy


def changeSpeed(values):
  rospy.logerr(values)
  def callback(data):
    if abs(values[0] - abs(data.axes[3])) > 0.1: 
      rospy.logerr(data.axes[3])
      values[1].ChangeDutyCycle(abs(data.axes[3]))  
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
  pwm.start(100)
  previousValue = 0

  rospy.init_node("micro_rov")
  rospy.logerr(123)
  rospy.Subscriber("/joy/joy1", Joy, changeSpeed([previousValue, pwm]))
  rospy.spin()

