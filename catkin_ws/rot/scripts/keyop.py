#!/usr/bin/env python

'''
RoT_keyop.py:
    A ros keyboard teleoperation script for RoT 
    based on the keyop code from the ackermann_drive_teleop
     ROS package by George Kouros (gkourosg@yahoo.gr).
'''

__author__ = 'Shaun Price'
__copyright__   = "Copyright 2019, Shaun Price"
__license__ = "GPLv3"
__version__ = "0.1.0"
__maintainer__ = "Shaun Price"
__email__ = "shaun@priceconsulting.biz"

import roslib
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Bool
import sys, select, termios, tty
import thread
from numpy import clip
import subprocess

control_keys = {
    'up'    : '\x41',
    'down'  : '\x42',
    'right' : '\x43',
    'left'  : '\x44',
    'space' : '\x20',
    'tab'   : '\x09',
    'head_right' : '\x73', # s
    'head_left'  : '\x61', # a
    'head_up'    : '\x77', # w
    'head_down'  : '\x7a', # z
    'music'   : '\x6d', # m
    'reset'   : '\x72', # r
    'action1' : '\x31', # 1
    'action2' : '\x32', # 2
    'action3' : '\x33', # 3
    'action4' : '\x34', # 4
    'action5' : '\x35', # 5
    'action6' : '\x36', # 6
    'action7' : '\x37', # 7
    'action8' : '\x38', # 8
    'action9' : '\x39'} # 9

key_bindings = {
    '\x41' : ( 1.0 , 0.0 , 0.0 , 0.0),
    '\x42' : (-1.0 , 0.0 , 0.0 , 0.0),
    '\x43' : ( 0.0 ,-1.0 , 0.0 , 0.0),
    '\x44' : ( 0.0 , 1.0 , 0.0 , 0.0),
    '\x20' : ( 0.0 , 0.0 , 0.0 , 0.0),
    '\x09' : ( 0.0 , 0.0 , 0.0 , 0.0),
    '\x73' : ( 0.0 , 0.0 , 0.0 , 1.0),
    '\x61' : ( 0.0 , 0.0 , 0.0 ,-1.0),
    '\x77' : ( 0.0 , 0.0 , 1.0 , 0.0),
    '\x7a' : ( 0.0 , 0.0 ,-1.0 , 0.0)}

# Keeps track of subprocess

class RoTKeyop:

    def __init__(self, args):
        max_speed = 0.25
        max_steering_angle = 0.35
        max__head_tilt_angle = 100 # %
        max__head_pan_angle = 100  # %

        cmd_run_topic = 'ackermann_cmd'
        cmd_head_tilt_topic = 'headTilt'
        cmd_head_pan_topic = 'headPan'
        cmd_music_topic = 'musicLights'

        self.process = None

        self.new_message = False
        self.speed_range = [-float(max_speed), float(max_speed)]
        self.steering_angle_range = [-float(max_steering_angle), float(max_steering_angle)]
        self.head_tilt_range = [-float(max__head_tilt_angle), float(max__head_tilt_angle)]
        self.head_pan_range = [-float(max__head_pan_angle), float(max__head_pan_angle)]
        
        for key in key_bindings:
            key_bindings[key] = \
                    (key_bindings[key][0] * float(max_speed) / 5,
                     key_bindings[key][1] * float(max_steering_angle) / 5,
                     key_bindings[key][2] * float(max__head_tilt_angle) / 5,
                     key_bindings[key][3] * float(max__head_pan_angle) / 5)

        self.speed = 0
        self.steering_angle = 0
        self.head_tilt = 0
        self.head_pan = 0
        self.musicLights = False # False = Off, True = On
        self.reset = False
        self.action = 0 # integer from 1-9 of what to say. Get's reset to 0 when completed
        self.motors_pub = rospy.Publisher(cmd_run_topic, AckermannDriveStamped, queue_size=1)
        self.head_tilt_pub = rospy.Publisher(cmd_head_tilt_topic, Int16, queue_size=1)
        self.head_pan_pub = rospy.Publisher(cmd_head_pan_topic, Int16, queue_size=1)
        self.music_pub = rospy.Publisher(cmd_music_topic, Bool, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/5.0), self.pub_messages_callback, oneshot=False)
        rospy.Timer(rospy.Duration(1.0/5.0), self.pub_actions_callback, oneshot=False)
        self.print_state()
        self.key_loop()

    def pub_messages_callback(self, event):
        if self.new_message is True:
            # Ackermann Messages
            ackermann_cmd_msg = AckermannDriveStamped()
            ackermann_cmd_msg.drive.speed = self.speed
            ackermann_cmd_msg.drive.steering_angle = self.steering_angle
            self.motors_pub.publish(ackermann_cmd_msg)

            # Head tilt message
            head_tilt_cmd_msg = Int16()
            head_tilt_cmd_msg.data = self.head_tilt
            self.head_tilt_pub.publish(head_tilt_cmd_msg)

            # Head pan message
            head_pan_cmd_msg = Int16()
            head_pan_cmd_msg.data = self.head_pan
            self.head_pan_pub.publish(head_pan_cmd_msg)

            # Music and lights
            music_cmd_msg = Bool()
            music_cmd_msg.data = self.musicLights
            self.music_pub.publish(music_cmd_msg)

            if self.reset == True:
                try:
                    self.process.terminate()
                except:
                    pass
                self.reset = False

            self.new_message = False

    def pub_actions_callback(self, event):
        # Actions

        # if were running an action kill any previous actions
        if self.action > 0:
            try:
                self.process.terminate()
            except:
                pass

        if self.action == 1:
            self.process = subprocess.Popen(["rosrun","tts","voicer.py","'Hello! My name is RoT. I am the robot of things. I am an experimental platform for developing robots'"], shell=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        elif self.action == 2:
            self.process = subprocess.Popen(["rosrun","tts","voicer.py","'Hello! My name is RoT'"], shell=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        elif self.action == 3:
            self.process = subprocess.Popen(["ogg123","/home/rot/Music/We_Will_Rock_You.ogg"], shell=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        elif self.action == 4:
            self.process = subprocess.Popen(["ogg123","/home/rot/Music/haka.ogg"], shell=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        elif self.action == 5:
            self.process = subprocess.Popen(["rosrun","tts","voicer.py","'Sorry! There is no action"], shell=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        elif self.action == 6:
            self.process = subprocess.Popen(["rosrun","tts","voicer.py","'Sorry! There is no action"], shell=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        elif self.action == 7:
            self.process = subprocess.Popen(["rosrun","tts","voicer.py","'Sorry! There is no action"], shell=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        elif self.action == 8:
            self.process = subprocess.Popen(["rosrun","tts","voicer.py","'Sorry! There is no action"], shell=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        elif self.action == 9:
            self.process = subprocess.Popen(["rosrun","tts","voicer.py","'Sorry! There is no action"], shell=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        self.action = 0 # reset
    
    def print_state(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        rospy.loginfo('\x1b[1M\r*********************************************')
        rospy.loginfo('\x1b[1M\rUse arrows to change speed and steering angle')
        rospy.loginfo('\x1b[1M\rUse "a" and "s" for head pan')
        rospy.loginfo('\x1b[1M\rUse "w" and "z" for head tilt')
        rospy.loginfo('\x1b[1M\rUse "m" to turn on/off music and lights mode')
        rospy.loginfo('\x1b[1M\rUse "1" for RoT introduction')
        rospy.loginfo('\x1b[1M\rUse "2" for hello! my name is RoT')
        rospy.loginfo('\x1b[1M\rUse "3" to play We Will Rock You')
        rospy.loginfo('\x1b[1M\rUse "4" to play Haka')
        rospy.loginfo('\x1b[1M\rUse space to brake and tab to align wheels')
        rospy.loginfo('\x1b[1M\rUse "r" to reset everything to 0')
        rospy.loginfo('\x1b[1M\rPress <ctrl-c> or <q> to exit')
        rospy.loginfo('\x1b[1M\r*********************************************')
        rospy.loginfo('\x1b[1M\r'
                      '\033[34;1mSpeed: \033[32;1m%0.2f m/s, '
                      '\033[34;1mSteer Angle: \033[32;1m%0.2f rad\033[0m',
                      self.speed, self.steering_angle)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    def key_loop(self):
        self.settings = termios.tcgetattr(sys.stdin)
        while 1:
            key = self.get_key()
            self.new_message = True
            if key in key_bindings.keys():
                if key == control_keys['space']:
                    self.speed = 0.0
                elif key == control_keys['tab']:
                    self.steering_angle = 0.0
                else:
                    self.speed = self.speed + key_bindings[key][0]
                    self.steering_angle = self.steering_angle + key_bindings[key][1]
                    self.head_tilt = self.head_tilt + key_bindings[key][2]
                    self.head_pan = self.head_pan + key_bindings[key][3]
                    self.speed = clip(self.speed, self.speed_range[0], self.speed_range[1])
                    self.steering_angle = clip(self.steering_angle, self.steering_angle_range[0], self.steering_angle_range[1])
                    self.head_tilt = clip(self.head_tilt, self.head_tilt_range[0], self.head_tilt_range[1])
                    self.head_pan = clip(self.head_pan, self.head_pan_range[0], self.head_pan_range[1])
                self.print_state()
            elif key == control_keys['music']:
                self.musicLights = not self.musicLights
            elif key == control_keys['reset']:
                    self.reset = True
                    self.speed = 0
                    self.steering_angle = 0
                    self.head_tilt = 0
                    self.head_pan = 0
                    self.musicLights = False
                    self.print_state()
            elif key == control_keys['action1']:
                self.action = 1
            elif key == control_keys['action2']:
                self.action = 2
            elif key == control_keys['action3']:
                self.action = 3
            elif key == control_keys['action4']:
                self.action = 4
            elif key == control_keys['action5']:
                self.action = 5
            elif key == control_keys['action6']:
                self.action = 6
            elif key == control_keys['action7']:
                self.action = 7
            elif key == control_keys['action8']:
                self.action = 8
            elif key == control_keys['action9']:
                self.action = 9
            elif key == '\x03' or key == '\x71':  # ctr-c or q
                break
            else:
                continue
        self.finalize()

    def finalize(self):
        rospy.loginfo('Halting motors, aligning wheels and head, turning off music mode and exiting...')
        self.settings = termios.tcgetattr(sys.stdin)
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.drive.speed = 0
        ackermann_cmd_msg.drive.steering_angle = 0
        self.motors_pub.publish(ackermann_cmd_msg)

        head_tilt_cmd_msg = Int16()
        head_tilt_cmd_msg.data = 0
        self.head_tilt_pub.publish(head_tilt_cmd_msg)

        head_pan_cmd_msg = Int16()
        head_pan_cmd_msg.data = 0
        self.head_pan_pub.publish(head_pan_cmd_msg)

        music_cmd_msg = Bool()
        music_cmd_msg.data = False
        self.music_pub.publish(music_cmd_msg)
        
        try:
            self.process.terminate()
        except:
            pass

        sys.exit()
    
if __name__ == '__main__':
    rospy.init_node('rot_keyop_node')
    keyop = RoTKeyop(sys.argv[1:len(sys.argv)])
