#!/usr/bin/env python3
import os

import numpy as np
import rospy

from dynamic_reconfigure.server import Server
from steering_translator.cfg import SteeringTranslatorConfig
from ackermann_msgs.msg import AckermannDriveStamped
from pwm_radio_arduino.msg import control

_DEFAULT_CONFIG = {
    "steering_pwm_left": 0,
    "steering_pwm_center": 0,
    "steering_pwm_right": 0,

    "throttle_pwm_backward": 0,
    "throttle_pwm_stop": 0,
    "throttle_pwm_forward": 0,
}

config = _DEFAULT_CONFIG.copy()
INPUT_RANGE_STEERING = np.array([-1., 0., 1.], dtype='float32')
INPUT_RANGE_THROTTLE = np.array([-1., 0., 1.], dtype='float32')
pwm_range_steering = np.array([-1, 0, 1], dtype='int32')
pwm_range_throttle = np.array([-1, 0, 1], dtype='int32')


def handle_config_update(received_config, level):
    for key, old_value in config.items():
        config[key] = received_config.get(key, old_value)

    pwm_range_steering[:] = np.array([
        config["steering_pwm_left"],
        config["steering_pwm_center"],
        config["steering_pwm_right"]
    ])

    pwm_range_throttle[:] = np.array([
        config["throttle_pwm_backward"],
        config["throttle_pwm_stop"],
        config["throttle_pwm_forward"]
    ])

    rospy.loginfo("reconfiguration. "
                  "request: {}, "
                  "pwm_range_steering: {}, "
                  "pwm_range_throttle: {}".format(received_config, pwm_range_steering, pwm_range_throttle))

    return received_config


def handle_input(data):
    msg = control()
    msg.angle_control_usec = np.interp(
        data.drive.steering_angle, INPUT_RANGE_STEERING, pwm_range_steering
    ).astype('int32')
    msg.speed_control_usec = np.interp(
        data.drive.speed, INPUT_RANGE_THROTTLE, pwm_range_throttle
    ).astype('int32')
    steering_publisher.publish(msg)


if __name__ == "__main__":
    rospy.init_node("steering_translator", anonymous=False)
    steering_publisher = rospy.Publisher(
        "pwm_radio_arduino/control_driver", control, queue_size=1
    )
    rospy.Subscriber(
        "ackermann_cmd", AckermannDriveStamped, handle_input
    )
    srv = Server(SteeringTranslatorConfig, handle_config_update)
    rospy.spin()
