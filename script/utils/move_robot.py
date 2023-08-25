#!/usr/bin/env python3

import rospy
from typing import List

# Import ROS Messages
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from ur_rtde_controller.msg import CartesianPoint

# Import ROS Services
from std_srvs.srv import Trigger, TriggerRequest 
from ur_rtde_controller.srv import RobotiQGripperControl, RobotiQGripperControlRequest
from ur_rtde_controller.srv import GetForwardKinematic, GetForwardKinematicRequest, GetForwardKinematicResponse
from ur_rtde_controller.srv import GetInverseKinematic, GetInverseKinematicRequest, GetInverseKinematicResponse

class UR10e_RTDE_Move():
    
    def __init__(self):

        # Publishers
        # self.ur10Pub=rospy.Publisher('/ur_rtde/controllers/joint_space_controller/command',JointTrajectoryPoint,queue_size=10)
        # self.ur10PubCartesian=rospy.Publisher('/ur_rtde/controllers/cartesian_space_controller/command',CartesianPoint,queue_size=10)
        self.ur10Pub=rospy.Publisher('/desired_joint_pose', JointState, queue_size=1)
        # self.ur10PubCartesian=rospy.Publisher('/desired_tcp_pose', Pose, queue_size=1)

        # Init Gripper Service
        self.gripper_srv = rospy.ServiceProxy('/ur_rtde/robotiq_gripper/command', RobotiQGripperControl)

        # IK, FK Services
        self.get_FK_srv = rospy.ServiceProxy('ur_rtde/getFK', GetForwardKinematic)
        self.get_IK_srv = rospy.ServiceProxy('ur_rtde/getIK', GetInverseKinematic)

        # Stop Robot Service
        self.stop_service = rospy.ServiceProxy('/ur_rtde/controllers/stop_robot', Trigger)
        self.stop_req = TriggerRequest()

    def move_joint(self, joint_positions:List[float]):

        """ Joint Space Movement """

        # Destination Position (if `time_from_start` = 0 -> read velocity[0])
        pos = JointState()
        pos.position = joint_positions

        # pos = JointTrajectoryPoint()
        # pos.time_from_start = rospy.Duration(0)
        # pos.velocities = [0.4]

        # Publish Joint Position
        self.ur10Pub.publish(pos)

        # flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
        flag = rospy.wait_for_message('/trajectory_execution', Bool)

        # Exception with Trajectory Execution
        if flag.data is not True: raise Exception("ERROR: An exception occurred during Trajectory Execution")

    def move_cartesian(self, tcp_position:Pose):

        """ Cartesian Movement -> Converted in Joint Movement with IK """

        # Call Inverse Kinematic
        joint_position = self.IK(tcp_position)
        rospy.loginfo('Inverse Kinematic')

        # Joint Space Movement
        self.move_joint(joint_position)

    def FK(self, joint_positions:List[float]) -> Pose:

        # Set Forward Kinematic Request
        req = GetForwardKinematicRequest()
        req.joint_position = joint_positions

        # Call Forward Kinematic
        rospy.wait_for_service('ur_rtde/getFK')
        res: GetForwardKinematicResponse = self.get_FK_srv(req)

        return res.tcp_position

    def IK(self, pose:Pose) -> List[float]:

        # Set Inverse Kinematic Request
        req = GetInverseKinematicRequest()
        req.tcp_position = pose

        # Call Inverse Kinematic
        rospy.wait_for_service('ur_rtde/getIK')
        res: GetInverseKinematicResponse = self.get_IK_srv(req)

        return res.joint_position

    def move_gripper(self, position):

        """ Open-Close Gripper Function """

        # Set Gripper Request
        req = RobotiQGripperControlRequest()
        req.position, req.speed, req.force = position, 100, 25

        # Call Gripper Service
        rospy.wait_for_service('/ur_rtde/robotiq_gripper/command')
        res = self.gripper_srv(req)

        '''if gripper_response.status != "2":
        raise Exception("Sorry, An exception occurred")'''
        
    def stopRobot(self):

        rospy.wait_for_service('/ur_rtde/controllers/stop_robot')
        self.stop_req = TriggerRequest()
        self.stop_response = self.stop_service(self.stop_req)
