#!/usr/bin/env python

import rospy
from dialogue_package import MicrophoneListener as ml
from dialogue_package import RobotVoice as rv
from std_msgs.msg import String
import time
from random import randrange

class DialogueController:

    def __init__(self):

        self.voice = rv.RobotVoice()
        self.speech_filler = "yes ok "
        self.use_local_package_for_voice = False
        self.robot_is_talking = False
        self.dialogue_mode = False
        self.pub = rospy.Publisher('speech_commands', String, queue_size=20)
        self.talker_pub = rospy.Publisher('robot_talk', String, queue_size=5)
        self.talker_sub = rospy.Subscriber('robot_talk', String, self.talk_function)
        rospy.init_node('speech_text_broadcaster', anonymous=True)
        self.rate = rospy.Rate(10)
        self.mic_listener = ml.MicrophoneListener()
        self.last_activity_time = rospy.get_time()
        self.timeout_threshold = 45
        

        self.robot_jokes = [
            "Why did the robot get upset? Because everyone was pushing his buttons!",
            "Why did the robot avoid having his photo taken? He was a photo-resistor!",
            "What do you call a robot who likes to row? A row-bot!",
            "How do robots pay for things? With cache, of course!",
            "How do nano-robots travel? On the nano-tube!",
            "What do you get when you cross a cat with a robot? A meow-bot!",
            "What's the difference between a robot and a human? Humans have a sense of humor"
        ]

        # Multiply this by num of words in a phrase to come up with the seconds to mute
        # the microphone:
        self.delay_multiplier = 0.2

    def speech_text_broadcaster(self):

        mic_listener = self.mic_listener
        self.robot_say_phrase("Dialogue Controller has started. Social Bot is now listening.", self.talker_pub)

        while not rospy.is_shutdown():

            time_since_last_activity = rospy.get_time() - self.last_activity_time

            if time_since_last_activity > self.timeout_threshold:
                if self.dialogue_mode:
                    self.robot_say_phrase("I'm not listening anymore, say social bot to wake me up again.", self.talker_pub)
                self.dialogue_mode = False
                

            if not self.voice.is_talking():
                speech_command = mic_listener.listen().strip()
                rospy.loginfo(speech_command)

            if self.dialogue_mode:            
            
                if speech_command == "shutdown dialog controller":
                    self.robot_say_phrase("Ok, shutting down the dialogue controller.", self.talker_pub)
                    break

                elif speech_command == "what is your name" or speech_command == "what's your name":
                    self.robot_say_phrase("I am socio bot. It's nice to meet you", self.talker_pub)
                    self.mic_listener.mute_microphone()

                elif "joke" in speech_command:
                    self.robot_say_phrase(self.robot_jokes[randrange(len(self.robot_jokes) - 1)] + " ha ha ha ha. ha.", self.talker_pub)
                    self.mic_listener.mute_microphone()

                elif speech_command == "unrecognized input":
                    self.robot_say_phrase(
                        "Sorry, I didn't understand what you said. Please say it again.", 
                        self.talker_pub
                        )

                elif speech_command == "stop listening":
                    self.robot_say_phrase(
                        "Ok, just say social bot to get my attention", self.talker_pub
                    )
                    self.dialogue_mode = False

                else:
                    self.robot_say_phrase(
                        ("I heard you say " + speech_command + ". Is that correct?"), 
                        self.talker_pub
                        )

                    rospy.loginfo("Waiting for confirmation")
                    confirmation = mic_listener.listen().strip()
                    confirmed = False
                    for word in ["yes", "yeah", "correct", "affirmative", "roger", "yup", "true"]:
                        if word in confirmation:
                            self.robot_say_phrase("Got it.", self.talker_pub)

                            self.pub.publish(speech_command)
                            confirmed = True
                            self.mic_listener.mute_microphone()
                            break
                    if not confirmed:
                        self.robot_say_phrase("Oops, ok, let's try again.", self.talker_pub)

            for word in ["socio bot", "social bot", "social", "bot", "hello social bot"]:
                if word in speech_command:
                    self.robot_say_phrase("Yes? What can I do for you?", self.talker_pub)
                    self.dialogue_mode = True
                    self.reset_last_activity_time()
                    break

            self.rate.sleep()



    def robot_say_phrase(self, phrase, talker_publisher, use_local_package_for_voice = False):
        if use_local_package_for_voice:
            voice.say_something(self.speech_filler + phrase)
        else:
            talker_publisher.publish(phrase)



    def talk_function(self, data):
        if not self.voice.is_talking():
            self.robot_is_talking = True
            rospy.loginfo("Robot is talking, so we shouldn't be listening.")
            self.mic_listener.mute_microphone()
            self.voice.say_something(self.speech_filler + data.data)
            while(self.voice.is_talking()):
                self.rate.sleep()
            self.robot_is_talking = False
            self.mic_listener.unmute_microphone()
            rospy.loginfo("Robot should be done talking, so we should listen again.")
        #Yes, this is duplicate of above, but it's basically a queue for messages that came across the topic
        # and it will spin until the robot is done speaking, then will proceed with saying the message:
        else:
            rospy.loginfo("Waiting for the robot to stop talking before we say " + data.data)
            while self.voice.is_talking():
                self.rate.sleep()
            self.robot_is_talking = True
            self.mic_listener.mute_microphone()
            rospy.loginfo("Ok, now it's our turn. Robot is talking, so we shouldn't be listening.")
            self.voice.say_something(self.speech_filler + data.data)
            while(self.voice.is_talking()):
                self.rate.sleep()
            self.robot_is_talking = False
            self.mic_listener.unmute_microphone()
            rospy.loginfo("Robot should be done talking, so we should listen again.")
        self.reset_last_activity_time()


    def reset_last_activity_time(self):
        self.last_activity_time = rospy.get_time()



if __name__ == '__main__':
    try:
        dc = DialogueController()
        dc.speech_text_broadcaster()
    except rospy.ROSInterruptException:
        pass

