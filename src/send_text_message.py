import rospy
from std_msgs.msg import String

class TextMessageSender:

    def __init__(self):

        self.speech_command_listener = rospy.Subscriber('speech_commands', String, self.speech_command_handler)
        self.robot_talk_publisher = rospy.Publisher('robot_talk', String, queue_size = 5)
        self.waiting_for_text_command = True
        self.ask_for_message = False
        self.waiting_for_message = False
        self.ask_for_confirmation = False
        self.waiting_for_confirmation = False
        self.text_message_content = ""
        self.confirmation_phrase = ""
        

        rospy.init_node('text_message_sender', anonymous = True)

        self.speech_command_phrases = [
            "send a text message",
            "send a message",
            "send message",
            "send text"
        ]

        self.rate = rospy.Rate(10)

    def wait_for_text_command(self):
        while not rospy.is_shutdown():

            if self.ask_for_message:
                rospy.loginfo("Starting text message creation dialogue with guest")
                self.robot_talk_publisher.publish("Alright, what would you like your message to say?")
                self.ask_for_message = False
                self.waiting_for_message = True

            elif self.ask_for_confirmation:
                rospy.loginfo("Just confirming that the text message you want to send is: " + self.text_message_content)
                self.robot_talk_publisher.publish("Just confirming that the text message you want to send is " + self.text_message_content)
                self.ask_for_confirmation = False
                self.waiting_for_confirmation = True
                
            elif len(self.confirmation_phrase) > 0:
                rospy.loginfo("The confirmation phrase we got from the topic is: " + self.confirmation_phrase)
                self.robot_talk_publisher.publish("Ok, thanks for confirming. I'm sending the message now.")
                rospy.loginfo("THIS IS WHERE WE SUBMIT THE MESSAGE TO THE API: " + self.text_message_content)
                self.text_message_content = ""
                self.confirmation_phrase = ""
                self.do_text_message_function = False


    def speech_command_handler(self, data):
        if self.waiting_for_text_command:
            for phrase in self.speech_command_phrases:
                if phrase in data.data:
                    rospy.loginfo("Text message initiation phrase pulled from topic: " + data.data)
                    self.waiting_for_text_command = False
                    self.ask_for_message = True
                    break
        elif self.waiting_for_message:
            rospy.loginfo("Message phrase pulled from topic: " + data.data)
            self.text_message_content = data.data
            self.waiting_for_message = False
            self.ask_for_confirmation = True
        elif self.waiting_for_confirmation:
            rospy.loginfo("Confirmation phrase pulled from topic: " + data.data)
            self.confirmation_phrase = data.data
            self.waiting_for_confirmation = False



if __name__ == '__main__':
    try:
        tms = TextMessageSender()
        tms.wait_for_text_command()
    except rospy.ROSInterruptException:
        pass