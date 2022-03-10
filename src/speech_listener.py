#!/usr/bin/env python

import rospy
from dialogue_package import MicrophoneListener as ml
from dialogue_package import RobotVoice as rv
from std_msgs.msg import String
import time

class DialogueController:

    def __init__(self):

        self.voice = rv.RobotVoice()
        self.speech_filler = "yes ok "
        self.use_local_package_for_voice = False
        self.robot_is_talking = False
        self.pub = rospy.Publisher('speech_commands', String, queue_size=20)
        self.talker_pub = rospy.Publisher('robot_talk', String, queue_size=5)
        self.talker_sub = rospy.Subscriber('robot_talk', String, self.talk_function)
        rospy.init_node('speech_text_broadcaster', anonymous=True)
        self.rate = rospy.Rate(10)

    def speech_text_broadcaster(self):

        mic_listener = ml.MicrophoneListener()

        while not rospy.is_shutdown():
            rospy.loginfo("Inside while loop, robot is talking is " + str(self.robot_is_talking))
            if not self.robot_is_talking:
                self.robot_say_phrase("Social bot is listening.", self.talker_pub)
                speech_command = mic_listener.listen().strip()
                rospy.loginfo(speech_command)

                if speech_command == "unrecognized input":
                    self.robot_say_phrase(
                        "Sorry, I didn't understand what you said. Please say it again.", 
                        self.talker_pub
                        )

                else:
                    self.robot_say_phrase(
                        ("Oh, hello. I heard you say " + speech_command + ". Is that correct?"), 
                        self.talker_pub
                        )

                    rospy.loginfo("Waiting for confirmation")
                    confirmation = mic_listener.listen().strip()
                    confirmed = False
                    for word in ["yes", "yeah", "correct", "affirmative", "roger", "yup", "true"]:
                        if word in confirmation:
                            self.robot_say_phrase("Great, I will see what I can do for you.", self.talker_pub)

                            self.pub.publish(speech_command)
                            confirmed = True
                            break
                    if not confirmed:
                        self.robot_say_phrase("Oops, ok, let's try again.", self.talker_pub)

            self.rate.sleep()

    def robot_say_phrase(self, phrase, talker_publisher, use_local_package_for_voice = False):
        seconds_to_wait = len(phrase.split(" ")) * .3 #Robot talks at ~2 words per second average
        if use_local_package_for_voice:
            voice.say_something(self.speech_filler + phrase)
        else:
            talker_publisher.publish(phrase)
            time.sleep(seconds_to_wait)


    def talk_function(self, data):
        if not self.robot_is_talking:
            seconds_to_wait = len(data.data.split(" ")) * .3
            self.robot_is_talking = True
            rospy.loginfo("Robot is talking, so we shouldn't be listening.")
            self.voice.say_something(self.speech_filler + data.data)
            time.sleep(seconds_to_wait)
            self.robot_is_talking = False
            rospy.loginfo("Robot should be done talking, so we should listen again.")
        else:
            rospy.loginfo("Waiting for the robot to stop talking before we say " + data.data)
            while self.robot_is_talking:
                self.rate.sleep()
            seconds_to_wait = len(data.data.split(" ")) * .3
            self.robot_is_talking = True
            rospy.loginfo("Robot is talking, so we shouldn't be listening.")
            self.voice.say_something(self.speech_filler + data.data)
            time.sleep(seconds_to_wait)
            self.robot_is_talking = False
            rospy.loginfo("Robot should be done talking, so we should listen again.")



if __name__ == '__main__':
    try:
        dc = DialogueController()
        dc.speech_text_broadcaster()
    except rospy.ROSInterruptException:
        pass

