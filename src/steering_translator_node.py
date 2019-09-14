#!/usr/bin/env python
import rospy

from dynamic_reconfigure.server import Server
from steering_translator.cfg import SteeringTranslatorConfig


def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {}""".format(config))
    return config


if __name__ == "__main__":
    rospy.init_node("steering_translator", anonymous=False)
    srv = Server(SteeringTranslatorConfig, callback)
    rospy.spin()

