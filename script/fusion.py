#!/usr/bin/env python3

import rospy, rospkg
import os, torch

# Import ROS Messages
from std_msgs.msg import Int32MultiArray
from multimodal_fusion.msg import VoiceCommand, FusedCommand

# Import Neural Network
from network.neural_classifier_training import LitNeuralNet, DEVICE
from network.neural_classifier_training import input_size, hidden_size, output_size
from network.neural_classifier_training import train_percent, val_percent, test_percent

# Import Utilities
from utils.move_robot import UR10e_RTDE_Move
from utils.utils import MyThread, count_occurrences, classifierNetwork
from utils.command_list import *

"""

Labelling:

    0: Empty Command
    1: Place Object + Given Area (Voice Only)
    2: Place Object + Given Area + Point-At
    3: Place Object + Point-At
    4: Point-At (Gesture Only)
    5: Stop (Gesture Only)
    6: Pause (Gesture Only)

    7: Start Experiment
    8: Object Moved
    9: User Moved
    10: User Can't Move
    11: Replan Trajectory
    12: Wait for Command
    13: Can Go
    14: Wait Time

Voice Commands:

    0:  NULL
    1:  EXPERIMENT_START
    2:  MOVED_OBJECT
    3:  PUT_OBJECT_IN_AREA
    4:  PUT_OBJECT_IN_GIVEN_AREA
    5:  PUT_OBJECT_IN_AREA_GESTURE
    6:  USER_MOVED
    7:  USER_CANT_MOVE
    8:  REPLAN_TRAJECTORY
    9:  WAIT_FOR_COMMAND
    10: CAN_GO
    11: WAIT_TIME

Gestures:

    0: No Gesture
    1: Pause
    2: Point-At
    3: Stop

"""

class Fusion:

    # Received Message Flags
    voiceCheck, gestureCheck, gesture_msgCheck, areaCheck = False, False, False, False

    # Received Messages
    voice_msg, gesture_msg, area_msg, voice_area = None, None, None, None

    # Parameters
    delay, recognition_time, recognition_percentage = 4, 2, 80

    # Variables
    gesture_vector, last_area, command = None, None, None
    network_input = (0,0)

    # Thread Flags
    thread_active, recognition_thread_active = False, False

    def __init__(self):

        # Init ROS Node
        rospy.init_node('multimodal_fusion', anonymous=True)

        # Get Package Path
        package_path = rospkg.RosPack().get_path('multimodal_fusion')

        # Instance Robot Movement Class
        self.robot = UR10e_RTDE_Move()

        # Publishers
        self.fusionPub = rospy.Publisher('/fused_command', FusedCommand, queue_size=1)

        # Subscribers
        rospy.Subscriber('/voice_command',   VoiceCommand,    self.voiceCallback)
        rospy.Subscriber('/gesture_command', Int32MultiArray, self.gestureCallback)
        rospy.Subscriber('/point_area',      Int32MultiArray, self.areaCallback)

        # Init Classifier Neural Network
        self.model = LitNeuralNet(train_percent, val_percent, test_percent, input_size, hidden_size, output_size).to(DEVICE)
        self.model.load_state_dict(torch.load(os.path.join(package_path, 'model/fusion_model.pth')))
        self.model.eval()

        # Init Thread -> Temporal Window, Recognition
        self.temporal_window_thread = MyThread()
        self.recognition_thread = MyThread()

        rospy.logwarn('Multimodal Fusion Initialized')

    def voiceCallback(self, data:VoiceCommand):

        # Save Voice Message
        self.voice_msg = data.command
        self.voice_area = data.area

        rospy.logwarn(f'Received Voice Command: {self.voice_msg} | {data.info}')

        # Set Voice Message Flag
        self.voiceCheck = True

    def gestureCallback(self, data:Int32MultiArray):

        # Save Gesture Message
        self.gesture_msg = data.data

        # Open Recognition Thread
        if self.recognition_thread_active == False:

            self.recognition_thread_active = True
            self.recognition_thread = MyThread()
            self.recognition_thread.start()
            rospy.loginfo('Gesture Recognition Started')

        # If Mono-dimensional: Overwrite | else: Append
        if self.gesture_vector is None: self.gesture_vector = self.gesture_msg
        else: self.gesture_vector = self.gesture_vector + self.gesture_msg

    def areaCallback(self, data:Int32MultiArray):

        # Save Area Message
        self.area_msg = data.data

        # Set Area Flag
        self.areaCheck = True

    def reset(self):

        # Reset Flags
        self.voiceCheck, self.gestureCheck, self.areaCheck = False, False, False

        # Reset Variables
        self.gesture_vector, self.last_area = None, None
        self.network_input = (0,0)

        # Reset Messages
        self.voice_msg, self.gesture_msg, self.area_msg, self.voice_area = None, None, None, None

    def checkArea(self):

        """ Check if Area Message is Received """

        if self.area_msg[0] != self.last_area:

            # Reset Flags
            self.areaCheck, self.area_msg = False, None

            # Save Last Area
            self.last_area = self.area_msg[0]
            print(f'Area: {self.last_area}')

    def checkGesture(self):

        """ Check if Gesture Message is Received """

        # Assert Gesture Recognized is in the Available Gestures
        assert self.gesture_recognized in available_gestures.keys(), 'Gesture Not Recognized'

        # Avoid `No Gesture`
        if self.gesture_recognized != 0:

            # Gesture Different from the Previous One -> Restart Thread Timer
            if self.network_input[0] != self.gesture_recognized:

                # Update Network Input
                self.network_input = (self.gesture_recognized, self.network_input[1])

                # Stop Thread Timer
                if self.thread_active == True:

                    self.temporal_window_thread.stop()
                    self.temporal_window_thread.join()
                    self.thread_active = False

                # Restart Thread Timer
                self.thread_active = True
                self.temporal_window_thread = MyThread()
                self.temporal_window_thread.start()

        # Reset Flags
        self.gestureCheck, self.gesture_recognized = False, None

    def checkVoice(self):

        """ Check if Voice Message is Received """
        
        # Assert Voice Command is in the Available Voice Commands
        assert self.voice_msg in available_voice_commands.keys(), 'Voice Command Not Recognized'

        # Avoid `Null Command`
        if self.voice_msg != 0:

            # Voice Command Different from the Previous One -> Restart Thread Timer
            if self.network_input[1] != self.voice_msg:

                # Update Network Input
                self.network_input = (self.network_input[0], self.voice_msg)

                # Stop Thread Timer
                if self.thread_active == True:

                    self.temporal_window_thread.stop()
                    self.temporal_window_thread.join()
                    self.thread_active = False

                # Restart Thread Timer
                self.thread_active = True
                self.temporal_window_thread = MyThread()
                self.temporal_window_thread.start()

        # Reset Flags
        self.voiceCheck = False

    def timeManager(self):

        # Gesture Recognition Thread -> Out of Time
        if int(self.recognition_thread.time) >= int(self.recognition_time):

            # Gesture Recognized -> Check Percentage
            if self.gesture_vector is not None:

                print(f'Gesture Vector: {self.gesture_vector}')

                # Get Most Recognized Gesture in the Vector if it's been published at least 3 times
                self.gesture_recognized = count_occurrences(self.gesture_vector, self.recognition_percentage) if len(self.gesture_vector) > 3 else None

                # If Gesture is Recognized -> Gesture Check = True
                if self.gesture_recognized is not None: 

                    print(f'Gesture Recognized: {self.gesture_recognized}')
                    self.gestureCheck = True

                else: print('Gesture Not Recognized')

            # Clear Gesture Vector and Message
            self.gesture_vector, self.gesture_msg = None, None

            # Stop Recognition Thread
            self.recognition_thread.stop()
            self.recognition_thread.join()
            self.recognition_thread_active = False

        # Check Area, Gesture and Voice
        if self.gestureCheck == True: self.checkGesture()
        if self.voiceCheck   == True: self.checkVoice()
        if self.areaCheck    == True: self.checkArea()

        # Thread -> Out of Time
        if int(self.temporal_window_thread.time) >= int(self.delay) and self.recognition_thread_active == False:

            # Stop Thread
            self.temporal_window_thread.stop()
            self.temporal_window_thread.join()
            self.thread_active = False

            # Get Fused Command from the Classifier Neural Network
            command = classifierNetwork(self.network_input, self.model)
            print(f'Fused Command: {command} | Network Input: {self.network_input}')

            # Empty Command
            if command == 0: print('Null Command')

            # Command in Fused Command List
            elif command in available_fused_commands.keys():

                # Pass Fused Command to the FusedCommand Message
                msg = FusedCommand()
                msg.fused_command = command
                msg.info = fused_command_info[command]

                # Place Object in Given Area
                if command in [1,2]: msg.area = self.voice_area
    
                # Place Object in Point-At Area
                if command == 3: msg.area = self.last_area

            # Command in Voice-Only Command List
            elif command in available_voice_only_commands.keys():

                # Pass Voice-Only Command to the FusedCommand Message
                msg = FusedCommand()
                msg.fused_command = command
                msg.info = voice_only_command_info[command - len(fused_command_info)]

                # Wait Time Command -> Pass Wait Time (in `voice_area` Field)
                if command == 14: msg.wait_time = self.voice_area

            # Publish Fused Command
            self.fusionPub.publish(msg)

            # Reset Thread
            if self.thread_active == True:
                self.temporal_window_thread.stop()
                self.temporal_window_thread.join()
                self.thread_active = False

            # Reset Variables
            self.reset()

if __name__ == '__main__':

    # Initialize Multimodal Fusion Node
    multimodal_fusion = Fusion()

    # While ROS is Running
    while not rospy.is_shutdown():

        # Run Time Manager
        multimodal_fusion.timeManager()
