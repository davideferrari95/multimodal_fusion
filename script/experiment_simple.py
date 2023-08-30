#!/usr/bin/env python3

import rospy, time

# Import ROS Messages
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from multimodal_fusion.msg import FusedCommand, TrajectoryError

# Move Robot Utilities
from utils.move_robot import UR10e_RTDE_Move, GRIPPER_OPEN, GRIPPER_CLOSE
from utils.command_list import *
from utils.object_list import object_list, area_list

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

    # TTS Error Messages
    OBSTACLE_DETECTED_ERROR_STRING = "There is an obstacle where I have to place the object"
    MOVE_TO_USER_ERROR_STRING = "I have to get there, if you don't move I'll have to slow down too much"

    # Flags
    experiment_started, stop = False, False
    wait_for_command, error_stop = False, False

    # Received Data
    error_handling_command, received_error = None, None

    def __init__(self) -> None:

        # Init ROS Node
        rospy.init_node('Experiment_Manager')

        # Instance Robot Movement Class
        self.robot = UR10e_RTDE_Move()

        # Publishers
        self.ttsPub   = rospy.Publisher('/tts', String, queue_size=1)

        # Subscribers
        rospy.Subscriber('/fused_command', FusedCommand, self.commandCallback)
        rospy.Subscriber('/trajectory_error', TrajectoryError, self.trajectoryErrorCallback)

    def commandCallback(self, data:FusedCommand):

        rospy.logwarn(f"Received Command: {self.received_command}")

        # Start / Stop Experiment Command -> Set Flags
        if   data.fused_command == START_EXPERIMENT: self.experiment_started = True; return
        elif data.fused_command in [STOP, PAUSE]: self.stop = True; return

        # Empty or Unused Commands -> Do Nothing
        elif data.fused_command in [EMPTY_COMMAND, POINT_AT, PLACE_OBJECT_POINT_AT]: return

        # Place Object Command -> Save Place Command
        elif data.fused_command in [PLACE_OBJECT_GIVEN_AREA, PLACE_OBJECT_GIVEN_AREA_POINT_AT]: self.error_handling_command = data

        # Voice-Only Commands -> Save Error Handling Command
        elif data.fused_command in [OBJECT_MOVED, USER_MOVED, WAIT_FOR_COMMAND, CAN_GO]: self.error_handling_command = data

    def trajectoryErrorCallback(self, data:TrajectoryError):

        # Save Received Error
        self.received_error = data.error
        rospy.logwarn(f"Received Error: {data.error} | {data.info}")

    def handover(self, pick_position, place_position) -> bool:

        """ Handover Object """

        # Move Gripper to Starting Position | Negative Error Handling -> Return
        if not self.robot.move_gripper(GRIPPER_OPEN):
            if not self.errorHandling(OPEN_GRIPPER_ERROR): return False
        rospy.loginfo('Open Gripper')

        # Forward Kinematic -> Increase z + 20cm
        pick_position_up: Pose = self.robot.FK(pick_position)
        pick_position_up.position.z += 0.20
        rospy.loginfo('Forward Kinematic')

        # Move 20cm Over the Object | Negative Error Handling -> Return
        if not self.robot.move_joint(pick_position_up):
            if not self.errorHandling(MOVE_OVER_OBJECT_ERROR, pick_position_up): return False
            rospy.loginfo('Move Over the Object')

        # Move to Object | Negative Error Handling -> Return
        if not self.robot.move_joint(pick_position):
            if not self.errorHandling(MOVE_TO_OBJECT_ERROR, pick_position): return False
        rospy.loginfo('Move To the Object')

        # Grip Object | Negative Error Handling -> Return
        if not self.robot.move_gripper(GRIPPER_CLOSE):
            if not self.errorHandling(CLOSE_GRIPPER_ERROR): return False
        rospy.loginfo('Close Gripper')
        time.sleep(1)

        # Move 20cm Over the Object | Negative Error Handling -> Return
        if not self.robot.move_joint(pick_position_up):
            if not self.errorHandling(MOVE_OVER_OBJECT_AFTER_ERROR, pick_position_up): return False
        rospy.loginfo('Move Over the Object')

        # Forward Kinematic -> Increase z + 20cm
        place_position_up: Pose() = self.robot.FK(place_position)
        place_position_up.position.z += 0.20
        rospy.loginfo('Forward Kinematic')

        # Move 20cm Over the Place Position | Negative Error Handling -> Return
        if not self.robot.move_joint(place_position_up):
            if not self.errorHandling(MOVE_OVER_PLACE_ERROR, place_position_up): return False
        rospy.loginfo('Move Over the Place Position')

        # Move to Place Position | Negative Error Handling -> Return
        if not self.robot.move_joint(place_position):
            if not self.errorHandling(MOVE_TO_PLACE_ERROR, place_position): return False
        rospy.loginfo('Move To the Place Position')

        # Release Object | Negative Error Handling -> Return
        if not self.robot.move_gripper(GRIPPER_OPEN):
            if not self.errorHandling(OPEN_GRIPPER_AFTER_ERROR): return False
        rospy.loginfo('Open Gripper')
        time.sleep(1)

        # Move 20cm Over the Place Position | Negative Error Handling -> Return
        if not self.robot.move_joint(place_position_up):
            if not self.errorHandling(MOVE_OVER_PLACE_AFTER_ERROR, place_position_up): return False
        rospy.loginfo('Move Over the Place Position')

        # Move to Home | Negative Error Handling -> Return
        if not self.robot.move_joint(self.HOME):
            if not self.errorHandling(MOVE_HOME_ERROR, self.HOME): return False
        rospy.loginfo('Move To Home')

        return True

    def placeObject(self, place_area) -> bool:

        """ Place Object in Area """

        # Get Area Place Position
        for area in area_list:
            if area.name == place_area: place_position = area.position

        # Forward Kinematic -> Increase z + 20cm
        place_position_up: Pose() = self.robot.FK(place_position)
        place_position_up.position.z += 0.20
        rospy.loginfo('Forward Kinematic')

        # Move 20cm Over the Place Position | Error -> Return
        if not self.robot.move_joint(place_position_up): return False
        rospy.loginfo('Move Over the Place Position')

        # Move to Place Position | Error -> Return
        if not self.robot.move_joint(place_position): return False
        rospy.loginfo('Move To the Place Position')

        # Release Object | Error -> Return
        if not self.robot.move_gripper(GRIPPER_OPEN): return False
        rospy.loginfo('Open Gripper')
        time.sleep(1)

        # Move 20cm Over the Place Position | Error -> Return
        if not self.robot.move_joint(place_position_up): return False
        rospy.loginfo('Move Over the Place Position')

        # Move to Home | Error -> Return
        if not self.robot.move_joint(self.HOME): return False
        rospy.loginfo('Move To Home')

        return True

    def errorHandling(self, handover_error, goal_position=None) -> bool:

        """ Error Handling """

        # Gripper Movement Error -> Stop Handover
        if handover_error in [OPEN_GRIPPER_ERROR, CLOSE_GRIPPER_ERROR, OPEN_GRIPPER_AFTER_ERROR]:

            rospy.logwarn('ERROR: An exception occurred during Gripper Movement')
            return False

        # Pick Object Movement Error -> Stop Handover
        elif handover_error in [MOVE_OVER_OBJECT_ERROR, MOVE_TO_OBJECT_ERROR, MOVE_OVER_OBJECT_AFTER_ERROR]:

            rospy.logwarn('ERROR: An exception occurred during Pick Object Movement')
            return False

        # Place Object Movement Error -> Check Error Type
        elif handover_error in [MOVE_OVER_PLACE_ERROR, MOVE_TO_PLACE_ERROR, MOVE_OVER_PLACE_AFTER_ERROR]:

            # Obstacle Detected -> Move to User
            if self.received_error == OBSTACLE_DETECTED_ERROR:

                # Publish Error Message
                error_msg = String()
                error_msg.data = self.OBSTACLE_DETECTED_ERROR_STRING
                self.ttsPub.publish(error_msg)
                rospy.logwarn('ERROR: Obstacle Detected')

                # Wait for Error Handling Command
                while self.error_handling_command is None:
                    rospy.loginfo_throttle(5, 'Waiting for Error Handling Command')

                # Object Moved -> Move to Place -> Restart Handover
                if self.error_handling_command.fused_command == OBJECT_MOVED:

                    rospy.loginfo('Object Moved Command -> Retry Place Object')

                    # Move to Position -> Positive Error Handling -> Continue Handover
                    if self.robot.move_joint(goal_position): return True

                    else:

                        # Negative Error Handling -> Stop Handover
                        rospy.logerr('ERROR: An exception occurred during Object Moved Error Handling')
                        self.error_stop = True
                        return False

                # Put Object in Place Area -> Place Object -> Stop Handover
                elif self.error_handling_command.fused_command in [PLACE_OBJECT_GIVEN_AREA, PLACE_OBJECT_GIVEN_AREA_POINT_AT]:

                    # Place Object in Area Error Handling -> Stop Handover
                    if not self.placeObject(place_area=self.error_handling_command.area):

                        # Negative Error Handling -> Stop Handover
                        rospy.logerr('ERROR: An exception occurred during Place Object in Area Error Handling')
                        self.error_stop = True

                    # Stop Handover
                    return False

            # Move to User Error -> Stop Handover
            elif self.received_error == MOVE_TO_USER_ERROR:

                # Publish Error Message
                error_msg = String()
                error_msg.data = self.MOVE_TO_USER_ERROR_STRING
                self.ttsPub.publish(error_msg)
                rospy.logwarn('ERROR: Move to User Error')

                # Wait for Error Handling Command
                while self.error_handling_command is None:
                    rospy.loginfo_throttle(5, 'Waiting for Error Handling Command')

                # User Moved -> Move to Place -> Restart Handover
                if self.error_handling_command.fused_command in [USER_MOVED, CAN_GO]:

                    rospy.loginfo('User Moved Command -> Retry Place Object')

                    # Move to Position -> Positive Error Handling -> Continue Handover
                    if self.robot.move_joint(goal_position): return True

                    else:

                        # Negative Error Handling -> Stop Handover
                        rospy.logerr('ERROR: An exception occurred during User Moved Error Handling')
                        self.error_stop = True
                        return False

                # Put Object in Place Area -> Place Object -> Stop Handover
                elif self.error_handling_command.fused_command in [PLACE_OBJECT_GIVEN_AREA, PLACE_OBJECT_GIVEN_AREA_POINT_AT]:

                    # Place Object in Area Error Handling -> Stop Handover
                    if not self.placeObject(place_area=self.error_handling_command.area):

                        # Negative Error Handling -> Stop Handover
                        rospy.logerr('ERROR: An exception occurred during Place Object in Area Error Handling')
                        self.error_stop = True

                    # Stop Handover
                    return False

        # Clear Error Handling Messages
        self.error_handling_command = None
        self.received_error = None

    def run(self):

        """ Run the Experiment """

        # Wait for Experiment Start
        while not self.experiment_started: rospy.loginfo_throttle(5, 'Waiting for Experiment Start')

        # Start Experiment
        rospy.loginfo('Start Experiment - Move to Home')
        self.robot.move_joint(self.HOME)

        # Handover Object
        for object in object_list:

            # Break if Stop Signal Received
            if self.error_stop: break

            if not self.handover(object.pick_position, object.place_position):

                # Error Handling
                print('ERROR: An exception occurred during Handover')

if __name__ == '__main__':

    # Initialize Experiment Manager Node
    exp = ExperimentManager()

    # Run the Experiment
    while not rospy.is_shutdown():
        exp.run()
