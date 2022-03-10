#!/usr/bin/env python

import rospy
from dialogue_package import RobotVoice as rv
from std_msgs.msg import String

voice = rv.RobotVoice()

speech_filler = "ok yes ok "

def robot_talker():

    listen = rospy.Subscriber('robot_talk', String, talk_function)

    rospy.init_node('robot_talker', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()


def talk_function(data):
    voice.say_something(speech_filler + data.data)



if __name__ == '__main__':
    try:
        robot_talker()
    except rospy.ROSInterruptException:
        pass

