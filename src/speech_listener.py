#!/usr/bin/env python

import rospy
from dialogue_package import MicrophoneListener as ml
from dialogue_package import RobotVoice as rv
from std_msgs.msg import String, Bool
import time
from random import randrange

class DialogueController:

    def __init__(self):

        # Voice object we use to generate synthetic voice from text:
        self.voice = rv.RobotVoice()

        # Use this to overcome the hardware issue where the first second of speech
        # gets cut off at random
        self.speech_filler = "yes ok "

        # Keep this as false to use the topic to request a voice action
        self.use_local_package_for_voice = False

        # Semaphore that tells certain processes to wait until the robot is done talking
        self.robot_is_talking = False

        # Sempaphore that indicates if the robot should respond to anything other than its
        # wake word. If false, then it will only respond to the wake word.
        self.dialogue_mode = False

        # Publish verified speech commands to this topic for other modules to perform actions.
        self.pub = rospy.Publisher('speech_commands', String, queue_size=20)

        # Publish requests for the robot to generate synthetic speech to this topic.
        self.talker_pub = rospy.Publisher('robot_talk', String, queue_size=5)

        # We also listen to the same topic. We perform both speech generation and recognition
        # in this module in order to keep things well synched.
        self.talker_sub = rospy.Subscriber('robot_talk', String, self.talk_function)

        # Listen to this topic for instructions to initiate dialogue with a guest. Mostly comes
        # from the find a guest module where the robot identifies and approaches a person to 
        # interact with them.
        self.task_start_sub = rospy.Subscriber('task/start', Bool, self.task_start_callback)


        rospy.init_node('speech_text_broadcaster', anonymous=True)
        self.rate = rospy.Rate(10)
        self.mic_listener = ml.MicrophoneListener()

        # Timeout threshold indicates how long we should wait before exiting dialogue mode 
        # since the robot's last speech activity.
        self.last_activity_time = rospy.get_time()
        self.timeout_threshold = 45
        
        #Kind of an easter egg, a  list of jokes that the robot can select from at random:
        self.robot_jokes = [
            "Why did the robot get upset? Because everyone was pushing his buttons!",
            "Why did the robot avoid having his photo taken? He was a photo-resistor!",
            "What do you call a robot who likes to row? A row-bot!",
            "How do robots pay for things? With cache, of course!",
            "How do nano-robots travel? On the nano-tube!",
            "What do you get when you cross a cat with a robot? A meow-bot!",
            "What's the difference between a robot and a human? Humans have a sense of humor"
        ]



    # Start function for the dialogue system. Waits for a wake word to enter dialogue mode.
    # once it enters dialogue mode, then it can interact with guests. If a command is issued,
    # it confirms the command with the guest and then publishes the command to the speech_commands
    # topic. 
    def speech_text_broadcaster(self):

        mic_listener = self.mic_listener
        self.robot_say_phrase("Dialogue Controller has started. Social Bot is now listening.", self.talker_pub)

        while not rospy.is_shutdown():

            time_since_last_activity = rospy.get_time() - self.last_activity_time
            rospy.loginfo("Timeout timer: " + str(time_since_last_activity))

            if time_since_last_activity > self.timeout_threshold:
                if self.dialogue_mode:
                    self.robot_say_phrase("I'm not listening anymore, say social bot to get my attention.", self.talker_pub)
                self.dialogue_mode = False
                

            if not self.voice.is_talking():
                speech_command = mic_listener.listen().strip()
                speech_command = speech_command.replace(" one ", " 1 ").replace(" two ", " 2 ").replace(" three ", " 3 ").replace(" four ", " 4 ")
                speech_command = speech_command.replace(" five ", " 5 ").replace(" six ", " 6 ").replace(" seven ", " 7 ")
                speech_command = speech_command.replace(" eight ", " 8 ").replace(" nine ", " 9 ")
                rospy.loginfo(speech_command)

            if self.dialogue_mode:            
            
                if "shutdown dialog controller" in speech_command:
                    self.robot_say_phrase("Ok, shutting down the dialogue controller.", self.talker_pub)
                    break

                elif "what is your name" in speech_command or "what's your name" in speech_command:
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

                elif "stop listening" in speech_command:
                    self.robot_say_phrase(
                        "Ok, just say social bot to get my attention", self.talker_pub
                    )
                    self.dialogue_mode = False

                elif speech_command == "":
                    pass

                elif speech_command in ["what are my options", "what are my choices", "what can you do for me"]:
                    self.robot_say_phrase(
                        "I'm glad you asked. I can send a text message to the celebrant, take a picture of you, " +
                        "play a song, or usher you to your table. I might even tell you a joke if you're nice.",
                        self.talker_pub
                    )

                else:
                    self.robot_say_phrase(
                        ("I heard you say " + speech_command + ". Is that correct?"), 
                        self.talker_pub
                        )

                    rospy.loginfo("Waiting for confirmation")
                    confirmation = mic_listener.listen().strip()
                    confirmed = False
                    for word in ["yes", "yeah", "correct", "affirmative", "roger", "yup", "true"]:
                        if word in confirmation and confirmation != "":
                            self.robot_say_phrase("Got it.", self.talker_pub)

                            self.pub.publish(speech_command)
                            confirmed = True
                            self.mic_listener.mute_microphone()
                            break
                    if not confirmed and confirmation != "":
                        self.robot_say_phrase("Oops, ok, let's try again.", self.talker_pub)

            for word in ["socio bot", "social bot", "social", "bot", "hello social bot"]:
                if word in speech_command:
                    self.robot_say_phrase("Yes? What can I do for you?", self.talker_pub)
                    self.dialogue_mode = True
                    self.reset_last_activity_time()
                    break

            speech_command = ""

            self.rate.sleep()



    # Intermediary between the speech_text_broadcaster method and the talk_function, gives us
    # the option of using the topic or using direct access to the voice object to make the robot
    # talk.
    def robot_say_phrase(self, phrase, talker_publisher, use_local_package_for_voice = False):
        if use_local_package_for_voice:
            voice.say_something(self.speech_filler + phrase)
        else:
            talker_publisher.publish(phrase)




    # Callback function for the robot_talk topic. It's a bit duplicative because if the robot
    # is already talking when another message comes in, the else clause puts the request into
    # a loop state until the robot is done talking, at which point it begins the talking
    # function. Note that this function controls microphone muting and unmuting. This is 
    # important so that the robot does not hear itself talking.
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



    # Simple function that resets last activity time. Used to manage the timeout feature.
    def reset_last_activity_time(self):
        self.last_activity_time = rospy.get_time()



    # Action to perform when a task/start = True message is received on the task/start topic.
    # This initiates dialogue with a guest that the robot has just approached.
    def task_start_callback(self, data):
        if data.data == True:
            self.dialogue_mode = True
            self.robot_say_phrase(
                "Hello, I'm social bot. I'm here to see if there's anything I can do for you." +
                " If you'd like to hear your options, just say what can you do for me? or what are my options."
                , self.talker_pub)


if __name__ == '__main__':
    try:
        dc = DialogueController()
        dc.speech_text_broadcaster()
    except rospy.ROSInterruptException:
        pass

