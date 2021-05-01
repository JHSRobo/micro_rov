#!/usr/bin/env python

from RPi import GPIO
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy


def changeSpeed(previousValue, pwm):
  def callback(data):
    print(data.axes[3])
    if abs(previousValue - abs(data.axes[3])) < 0.1: 
      pwm.ChangeDutyCycle(round(abs(data.axes[3]), 2)) 
      previousValue = abs(data.axes[3])



if __name__ == "__main__":
  GPIO.setmode(GPIO.BOARD)
  switcher_pin = 32
  duty_cycle_pin = 33
  frequency = 1000
  GPIO.setup(switcher_pin, GPIO.OUT)
  GPIO.setup(duty_cycle_pin, GPIO.OUT) 
  pwm = GPIO.PWM(duty_cycle_pin, frequency)
  pwm.start(0)

  rospy.init_node("micro_rov")
  rospy.Subscriber("joy/joy1", Joy, changeSpeed)
  rospy.spin()

