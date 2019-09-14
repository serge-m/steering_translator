#!/usr/bin/env python
import os

import rospy

from dynamic_reconfigure.server import Server
from steering_translator.cfg import SteeringTranslatorConfig
from ackermann_msgs.msg import AckermannDriveStamped
from pwm_radio_arduino.msg import pwm_steering

DIR = os.path.dirname(os.path.realpath(__file__))
CONFIG_FILE = os.path.join(DIR, "..", "steering_translator.json")

_DEFAULT_CONFIG = {
    "steering_pwm_left": 0,
    "steering_pwm_center": 0,
    "steering_pwm_right": 0,

    "throttle_pwm_left": 0,
    "throttle_pwm_center": 0,
    "throttle_pwm_right": 0,
}

config = _DEFAULT_CONFIG.copy()


def handle_config_update(received_config, level):
    rospy.loginfo("""Reconfigure Request: {}""".format(received_config))

    for key, old_value in config.items():
        config[key] = received_config.get(key, old_value)

    return received_config


def handle_input(data):
    rospy.loginfo("""new message {}""".format(data))
    steering = data.drive.steering_angle



if __name__ == '__main__':
    rospy.init_node("steering_translator", anonymous=False)
    steering_publisher = rospy.Publisher('pwm_radio_arduino/driver_pwm', pwm_steering, queue_size=10)
    rospy.Subscriber('ackermann_cmd_mux/output', AckermannDriveStamped, handle_input)
    srv = Server(SteeringTranslatorConfig, handle_config_update)
    rospy.spin()
