#!/usr/bin/env python3

import rospy
from typing import List

# Import ROS Messages
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from multimodal_fusion.msg import TrajectoryError
from ur_rtde_controller.msg import CartesianPoint

# Import ROS Services
from std_srvs.srv import Trigger, TriggerRequest
from ur_rtde_controller.srv import RobotiQGripperControl, RobotiQGripperControlRequest
from ur_rtde_controller.srv import GetForwardKinematic, GetForwardKinematicRequest, GetForwardKinematicResponse
from ur_rtde_controller.srv import GetInverseKinematic, GetInverseKinematicRequest, GetInverseKinematicResponse

# Import Command List
from utils.command_list import *

GRIPPER_OPEN = 100
GRIPPER_CLOSE = 0

class UR10e_RTDE_Move():

    trajectory_execution_received = False
    trajectory_executed = False
    too_slow_error = False
    robot_stopped_scaling_error = False

    def __init__(self):

        # Publishers
        self.ur10Pub = rospy.Publisher('/desired_joint_pose', JointState, queue_size=1)
        self.errorPub = rospy.Publisher('/trajectory_error', TrajectoryError, queue_size=1)
        self.cartesianPub = rospy.Publisher('/ur_rtde/controllers/cartesian_space_controller/command', CartesianPoint, queue_size=1)

        # Subscribers
        self.trajectory_execution_sub = rospy.Subscriber('/trajectory_execution', Bool, self.trajectory_execution_callback)
        self.too_slow_error_sub = rospy.Subscriber('/too_slow_error', Bool, self.too_slow_error_callback)
        self.stopped_scaling_error_sub = rospy.Subscriber('/stopped_scaling_error', Bool, self.stopped_scaling_error_callback)

        # Init Gripper Service
        self.gripper_srv = rospy.ServiceProxy('/ur_rtde/robotiq_gripper/command', RobotiQGripperControl)

        # IK, FK Services
        self.get_FK_srv = rospy.ServiceProxy('ur_rtde/getFK', GetForwardKinematic)
        self.get_IK_srv = rospy.ServiceProxy('ur_rtde/getIK', GetInverseKinematic)

        # Stop Robot Service
        self.stop_service = rospy.ServiceProxy('/ur_rtde/controllers/stop_robot', Trigger)
        self.stop_req = TriggerRequest()

    def trajectory_execution_callback(self, msg:Bool):

        """ Trajectory Execution Callback """

        # Set Trajectory Execution Flags
        self.trajectory_execution_received = True
        self.trajectory_executed = msg.data

    def too_slow_error_callback(self, msg:Bool):

            """ Too Slow Error Callback """

            # Set Too Slow Error Flag
            self.too_slow_error = msg.data

    def stopped_scaling_error_callback(self, msg:Bool):

            """ Stopped Scaling Error Callback """

            # Set Stopped Scaling Error Flag
            self.robot_stopped_scaling_error = msg.data

    def move_joint(self, joint_positions:List[float], forced=False) -> bool:

        """ Joint Space Movement """

        assert type(joint_positions) is list, f"Joint Positions must be a List | {type(joint_positions)} given | {joint_positions}"
        assert len(joint_positions) == 6, f"Joint Positions Length must be 6 | {len(joint_positions)} given"
        self.trajectory_execution_received = False

        # Destination Position (if `time_from_start` = 0 -> read velocity[0])
        pos = JointState()
        pos.position = joint_positions

        # Publish Joint Position
        self.ur10Pub.publish(pos)

        # Check Planning Error
        planning_error_flag: Bool = rospy.wait_for_message('/planning_error', Bool, timeout=10)

        # Return False if Planning Error
        if not forced and planning_error_flag.data:

            # Publish Planning Error -> Obstacle Detected
            msg = TrajectoryError()
            msg.error = OBSTACLE_DETECTED_ERROR
            msg.info = 'Obstacle Detected during Planning'
            self.errorPub.publish(msg)

            return False

        # Wait for Trajectory Execution
        while not self.trajectory_execution_received and not rospy.is_shutdown():

            # Debug Print
            rospy.loginfo_throttle(5, 'Waiting for Trajectory Execution')

            # Check for Too Slow Error
            if not forced and self.too_slow_error:

                # Publish Too Slow Error -> Move To User Error
                msg = TrajectoryError()
                msg.error = MOVE_TO_USER_ERROR
                msg.info = 'Too Slow Movement while Moving to User'
                self.errorPub.publish(msg)

                # Reset Too Slow Error
                self.too_slow_error = False

                return False

            elif not forced and self.robot_stopped_scaling_error:

                # Publish Stopped Scaling Error
                msg = TrajectoryError()
                msg.error = ROBOT_STOPPED_SCALING_ERROR
                msg.info = 'Robot Stopped due to Scaling while Moving to User'
                self.errorPub.publish(msg)

                # Reset Stopped Scaling Error
                self.robot_stopped_scaling_error = False

        # Reset Trajectory Execution Flag
        self.trajectory_execution_received = False

        # Exception with Trajectory Execution
        if not self.trajectory_executed: print("ERROR: An exception occurred during Trajectory Execution"); return False
        else: return True

    def move_cartesian(self, tcp_position:Pose) -> bool:

        """ Cartesian Movement """

        assert type(tcp_position) is Pose, f"Joint Positions must be a Pose | {type(tcp_position)} given | {tcp_position}"

        # Destination Position (if `time_from_start` = 0 -> read velocity[0])
        pos = CartesianPoint()
        pos.cartesian_pose = tcp_position
        pos.velocity = 0.02

        # Publish Cartesian Position
        rospy.logwarn('Cartesian Movement')
        self.cartesianPub.publish(pos)

        # Wait for Trajectory Execution
        while not self.trajectory_execution_received and not rospy.is_shutdown():

            # Debug Print
            rospy.loginfo_throttle(5, 'Waiting for Trajectory Execution')

        # Reset Trajectory Execution Flag
        self.trajectory_execution_received = False

        # Exception with Trajectory Execution
        if not self.trajectory_executed: print("ERROR: An exception occurred during Trajectory Execution"); return False
        else: return True

    # def move_cartesian(self, tcp_position:Pose) -> bool:

    #     """ Cartesian Movement -> Converted in Joint Movement with IK """

    #     # Call Inverse Kinematic
    #     joint_position = self.IK(tcp_position)
    #     rospy.loginfo('Inverse Kinematic')

    #     # Joint Space Movement
    #     return self.move_joint(joint_position)

    def FK(self, joint_positions:List[float]) -> Pose:

        # Set Forward Kinematic Request
        req = GetForwardKinematicRequest()
        req.joint_position = joint_positions

        # Call Forward Kinematic
        rospy.wait_for_service('ur_rtde/getFK')
        res: GetForwardKinematicResponse = self.get_FK_srv(req)

        return res.tcp_position

    def IK(self, pose:Pose, near_pose:List[float]=None) -> List[float]:

        # Set Inverse Kinematic Request
        req = GetInverseKinematicRequest()
        req.tcp_position = pose

        if near_pose is not None and len(near_pose) == 6: req.near_position = near_pose
        else: req.near_position = []

        # Call Inverse Kinematic
        rospy.wait_for_service('ur_rtde/getIK')
        res: GetInverseKinematicResponse = self.get_IK_srv(req)

        return list(res.joint_position)

    def move_gripper(self, position, gripper_enabled=True) -> bool:

        """ Open-Close Gripper Function """

        # Return True if Gripper is not Enabled
        if not gripper_enabled: return True

        # Set Gripper Request
        req = RobotiQGripperControlRequest()
        req.position, req.speed, req.force = position, 100, 100

        # Call Gripper Service
        rospy.wait_for_service('/ur_rtde/robotiq_gripper/command')
        res = self.gripper_srv(req)

        return True

    def stopRobot(self) -> bool:

        rospy.wait_for_service('/ur_rtde/controllers/stop_robot')
        self.stop_req = TriggerRequest()
        self.stop_response = self.stop_service(self.stop_req)
        return True
