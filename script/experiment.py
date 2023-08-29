#!/usr/bin/env python3

import rospy, time

# Import ROS Messages
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from multimodal_fusion.msg import FusedCommand, TrajectoryError

# Move Robot Utilities
from utils.move_robot import UR10e_RTDE_Move, GRIPPER_OPEN, GRIPPER_CLOSE
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

"""

class ExperimentManager():

    # Robot Positions
    HOME = [1.4624409675598145, -1.614187856713766, 1.8302066961871546, -1.795131345788473, -1.5152104536639612, -0.09420425096620733]
    PLACE = [-0.01490956941713506, -2.0275737247862757, 2.4103458563434046, -1.953010698358053, -1.5686352888690394, -0.05916053453554326]

    # TTS Error Messages
    OBSTACLE_DETECTED_ERROR = 'obstacle detected'
    MOVE_TO_USER_ERROR = 'move to user'

    # Flags
    experiment_started = False

    # Received Data
    received_command = EMPTY_COMMAND
    received_error   = NO_ERROR

    def __init__(self) -> None:

        # Init ROS Node
        rospy.init_node('Experiment_Manager')

        # Instance Robot Movement Class
        self.robot = UR10e_RTDE_Move()

        # Publishers
        self.ttsPub   = rospy.Publisher('/tts', String, queue_size=1)
        self.eventPub = rospy.Publisher('/alexa_events', String, queue_size=1)

        # Subscribers
        rospy.Subscriber('/fused_command', FusedCommand, self.commandCallback)
        rospy.Subscriber('/trajectory_error', TrajectoryError, self.trajectoryErrorCallback)

    def commandCallback(self, data:FusedCommand):

        # Save Received Command
        self.received_command = data
        rospy.logwarn(f"Received Command: {self.received_command}")

        # Start Experiment Command -> Set Flag
        if self.received_command.fused_command == START_EXPERIMENT: self.experiment_started = True

    def trajectoryErrorCallback(self, data:TrajectoryError):

        # Save Received Error
        self.received_error = data
        rospy.logwarn(f"Received Error: {self.received_error}")

    def handover(self, pick_position, place_position):

        # Move Gripper to Starting Position
        if not self.robot.move_gripper(GRIPPER_OPEN): return False
        rospy.loginfo('Open Gripper')

        # Forward Kinematic -> Increase z + 20cm
        pick_position_up: Pose = self.robot.FK(pick_position)
        pick_position_up.position.z += 0.20
        rospy.loginfo('Forward Kinematic')

        # Move 20cm Over the Object
        if not self.robot.move_joint(pick_position_up): return False
        rospy.loginfo('Move Over the Object')

        # Move to Object
        if not self.robot.move_joint(pick_position): return False
        rospy.loginfo('Move To the Object')
        time.sleep(1)

        # Grip Object
        if not self.robot.move_gripper(GRIPPER_CLOSE): return False
        rospy.loginfo('Close Gripper')

        # Move 20cm Over the Object
        if not self.robot.move_joint(pick_position_up): return False
        rospy.loginfo('Move Over the Object')
        time.sleep(0.5)

        # Forward Kinematic -> Increase z + 20cm
        place_position_up: Pose() = self.robot.FK(place_position)
        place_position_up.position.z += 0.20
        rospy.loginfo('Forward Kinematic')

        # Move 20cm Over the Place Position
        if not self.robot.move_joint(place_position_up): return False
        rospy.loginfo('Move Over the Place Position')
        time.sleep(1)

        # Move to Place Position
        if not self.robot.move_joint(place_position): return False
        rospy.loginfo('Move To the Place Position')

        # Release Object
        if not self.robot.move_gripper(GRIPPER_OPEN): return False
        rospy.loginfo('Open Gripper')

        # Move 20cm Over the Place Position
        if not self.robot.move_joint(place_position_up): return False
        rospy.loginfo('Move Over the Place Position')

        # Move to Home
        if not self.robot.move_joint(self.HOME): return False
        rospy.loginfo('Move To Home')

    def run(self):

        # Wait for Experiment Start
        while not self.experiment_started: pass

if __name__ == '__main__':

    # Initialize Experiment Manager Node
    exp = ExperimentManager()

    # Run the Experiment
    while not rospy.is_shutdown():
        exp.run()
