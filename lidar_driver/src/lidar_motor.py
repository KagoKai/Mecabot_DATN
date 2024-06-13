#!/usr/bin/env python

import time
import rospy
import tf
from std_msgs.msg import Int8
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT, initial=GPIO.LOW)

p = GPIO.PWM(12, 50)
p.start(0)

def pwm_duty_callback(data):
    p.ChangeDutyCycle(data)
    time.sleep(0.01)

def RplidarDutyListener():
    rospy.init_node("sub_rplidar_duty")
    rospy.Subscriber("rplidar_duty", Int8, pwm_duty_callback)
    # Parameters
    init_duty = rospy.get_param("motor_duty", 100)
    p.ChangeDutyCycle(init_duty)
    # Keep python from exiting
    rospy.spin()

if __name__ == '__main__':
    RplidarDutyListener()
    p.stop()
    GPIO.cleanup()