#!/usr/bin/env python
"""
 Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.

 Permission is hereby granted, free of charge, to any person obtaining a copy of this
 software and associated documentation files (the "Software"), to deal in the Software
 without restriction, including without limitation the rights to use, copy, modify,
 merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 permit persons to whom the Software is furnished to do so.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""


"""
  This script allows you to send text to your voice interaction robot.
  It publishes to the /text_input topic and listens to the /text_output topic for further instructions.
  
  It automatically keeps the robot awake by publishing a message to /wake_word every n seconds (5 by default)
"""

import rospy
import boto3
import time
import threading
import json
from std_msgs.msg import String
from lex_common_msgs.srv import AudioTextConversation
from audio_common_msgs.msg import AudioData
from voice_interaction_robot_msgs.msg import FulfilledVoiceCommand
from lex_common_msgs.srv import AudioTextConversationResponse
from lex_common_msgs.msg import KeyValue

text_input_publisher = rospy.Publisher("/text_input", String, queue_size=5)
wake_publisher = rospy.Publisher("/wake_word", String, queue_size=5)

class AlexaInput:
    wake_words = ("rot", "robot")

    def __init__(self, node_name, text_output_topic, audio_output_topic, fulfilled_command_topic, wake_word_topic, wake_publish_rate=5):
        self.queue_name = "RobotOfThings"
        # Open AWS SQS
        self.sqs = boto3.resource('sqs')
        self.queue = self.sqs.get_queue_by_name(QueueName=self.queue_name)# pole for messages
        rospy.init_node(node_name, disable_signals=True)
        rospy.loginfo("Initialized node %s" % node_name)
        rospy.Subscriber("/text_output", String, self.display_response)
        self.text_output_publisher = rospy.Publisher("/" + node_name + text_output_topic, String, queue_size=1)
        self.audio_output_publisher = rospy.Publisher("/" + node_name + audio_output_topic, AudioData, queue_size=1)
        self.fulfilled_command_publisher = rospy.Publisher("/" + node_name + fulfilled_command_topic, FulfilledVoiceCommand, queue_size=5)
        self.wake_publish_rate = wake_publish_rate
        wake_thread = threading.Thread(name='wake', target=self.keep_robot_awake)
        wake_thread.daemon = True
        wake_thread.start()

    def keep_robot_awake(self):
        if self.wake_publish_rate == 0:
            return
        while True:
            wake_publisher.publish(self.wake_words[0])
            time.sleep(self.wake_publish_rate)

    def display_response(self, data):
        text = data.data
        print(text)

    def get_input(self):
        while 1:
            messages = self.queue.receive_messages(MaxNumberOfMessages=1,WaitTimeSeconds=5)
            # Message received
            if len(messages) > 0:
                rospy.loginfo("Received Message From Alexa: {}".format(messages[0].body))
                # place received mesage on ros message bus
                json_response = json.loads(messages[0].body)
                response = AudioTextConversationResponse()
                response.intent_name = json_response['intentName']
                response.message_format_type = json_response['messageFormat']
                response.dialog_state = json_response['dialogState']
                for slot in json_response['slots']:
                    slotitem = KeyValue(slot.keys()[0],slot.values()[0])
                    response.slots.append(slotitem)

                self.handle_alexa_response(response)
                # remove message from sqs
                messages[0].delete()

    def handle_alexa_response(self, alexa_response):
        rospy.loginfo("Performing intent: %s" % alexa_response.intent_name)
        fulfilled_command = FulfilledVoiceCommand()
        fulfilled_command.intent_name = alexa_response.intent_name
        fulfilled_command.slots = alexa_response.slots
        self.fulfilled_command_publisher.publish(fulfilled_command)
        self.publish_alexa_response(alexa_response)

    def publish_alexa_response(self, alexa_response):
        if len(alexa_response.text_response) > 0:
            self.text_output_publisher.publish(alexa_response.text_response)
        if len(alexa_response.audio_response.data) > 0:
            self.audio_output_publisher.publish(alexa_response.audio_response)


def main():
    usage = """
Usage:
alexa ask rot               - Wake up the robot
move <direction> <distance> - Move in a direction at <distance> centimeters
turn <direction>            - Turn left/right
stop                        - Stop all movement
location                    - GPS location
look <direction>            - Look in a direction left/righ/up/down
music mode <mode>           - Turn on/off the interactive lights 
play <song>                 - Play one of RoT's songs (queen/haka)
reset                       - Reset everything to default
introduction                - RoT introduces itself
scan <mode>                 - LIDAR scan mode on/off
shut down                   - Shutdown RoS

"""
    print(usage)
    alexa_input = AlexaInput(
            node_name="voice_interaction_node",
            text_output_topic="/text_output",
            audio_output_topic="/audio_output",
            fulfilled_command_topic="/fulfilled_command",
            wake_word_topic="/wake_word")
    
    alexa_input.get_input()


if __name__ == "__main__":
    main()
