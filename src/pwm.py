#!/usr/bin/env python

from RPi import GPIO
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy


def changeSpeed(values):
  GPIO.setmode(GPIO.BOARD)
  switcher_pin = 32
  duty_cycle_pin = 33
  frequency = 1000
  GPIO.setup(switcher_pin, GPIO.OUT)
  GPIO.setup(duty_cycle_pin, GPIO.OUT) 
  pwm = GPIO.PWM(duty_cycle_pin, frequency)
  pwm.start(100)
  
  previousValue = 0
  
  def callback(data):
    if abs(previousValue - abs(data.axes[3])) > 0.1: 
      rospy.logerr(type(data.axes[3]))
      pwm.ChangeDutyCycle(abs(data.axes[3]))  
      previousValue = abs(data.axes[3])
      rospy.logerr(previousValue)
  return callback


if __name__ == "__main__":
  rospy.init_node("micro_rov")
  rospy.logerr(123)
  rospy.Subscriber("/joy/joy1", Joy, changeSpeed())
  rospy.spin()

