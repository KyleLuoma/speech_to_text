#!/usr/bin/env python

import rospy
from dialogue_package import MicrophoneListener as ml
from dialogue_package import RobotVoice as rv
from std_msgs.msg import String

voice = rv.RobotVoice()

def speech_text_broadcaster():

    mic_listener = ml.MicrophoneListener()

    pub = rospy.Publisher('speech_commands', String, queue_size=20)
    listen = rospy.Subscriber('robot_talk', String, talk_function)


    rospy.init_node('speech_text_broadcaster', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        speech_command = mic_listener.listen().strip()
        rospy.loginfo(speech_command)
        pub.publish(speech_command)
        rate.sleep()


def talk_function(data):
    voice.say_something(data.data)



if __name__ == '__main__':
    try:
        speech_text_broadcaster()
    except rospy.ROSInterruptException:
        pass

