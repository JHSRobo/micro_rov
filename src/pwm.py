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
  pwm.start(0)

  previousValue = 0
  direction = 1

  def callback(data):
    # Created a 0.1 threshold before changing the speed
    if abs(previousValue - abs(data.axes[3])) > 0.1:
      pwm.ChangeDutyCycle(abs(data.axes[3]) * 100)
      previousValue = abs(data.axes[3])
    # switch the direction if the previous number and this number have different signs using xor
    if previousValue ^ data.axes[3] < 0:
      direction *= -1
      rospy.loginfo('micro_rov: switched direction to {}'.format(direction))
      GPIO.output(switcher_pin, direction == 1)
  return callback


if __name__ == "__main__":
  rospy.init_node("micro_rov")
  # This IS RIGHT. changeSpeed returns a function so it will call changeSpeed()()
  # If you change this and it breaks it is your fault
  rospy.Subscriber("/joy/joy1", Joy, changeSpeed())
  rospy.spin()
