#!/usr/bin/env python3

import rospy, rospkg
import os, threading, time
import torch

from typing import List
from collections import Counter

# Import ROS Messages
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray, String, Bool 
from trajectory_msgs.msg import JointTrajectoryPoint
from ur_rtde_controller.msg import CartesianPoint

# Import ROS Services
from std_srvs.srv import Trigger, TriggerRequest 
from ur_rtde_controller.srv import RobotiQGripperControl, RobotiQGripperControlRequest
from ur_rtde_controller.srv import GetForwardKinematic, GetForwardKinematicRequest, GetForwardKinematicResponse
from ur_rtde_controller.srv import GetInverseKinematic, GetInverseKinematicRequest, GetInverseKinematicResponse

from neural_classifier import LitNeuralNet
from utils import OBJECTS, RIGHT_AREA, LEFT_AREA
from utils import defaultPos, placePos, intermediatePos

class Fusion:

    def __init__(self):

        # Init ROS Node
        rospy.init_node('multimodal_fusion', anonymous=True)

        # Get Package Path
        package_path = rospkg.RosPack().get_path('fusion')

        # Publishers
        # self.ur10Pub=rospy.Publisher('/ur_rtde/controllers/joint_space_controller/command',JointTrajectoryPoint,queue_size=10)
        # self.ur10PubCartesian=rospy.Publisher('/ur_rtde/controllers/cartesian_space_controller/command',CartesianPoint,queue_size=10)
        self.ur10Pub=rospy.Publisher('/desired_joint_pose', JointState, queue_size=1)
        # self.ur10PubCartesian=rospy.Publisher('/desired_tcp_pose', Pose, queue_size=1)
        self.ttsPub = rospy.Publisher('/tts',String,queue_size=1)

        # Init Gripper Service
        self.gripper_srv = rospy.ServiceProxy('/ur_rtde/robotiq_gripper/command', RobotiQGripperControl)

        # IK, FK Services
        self.get_FK_srv = rospy.ServiceProxy('ur_rtde/getFK', GetForwardKinematic)
        self.get_IK_srv = rospy.ServiceProxy('ur_rtde/getIK', GetInverseKinematic)

        # Stop Robot Service
        self.stop_service = rospy.ServiceProxy('/ur_rtde/controllers/stop_robot', Trigger)
        self.stop_req = TriggerRequest()

        # Subscribers
        rospy.Subscriber("voice",   Int32MultiArray, self.voiceCallback)
        rospy.Subscriber("gesture", Int32MultiArray, self.gestureCallback)
        rospy.Subscriber("area",    Int32MultiArray, self.areaCallback)

        # init network
        self.model = LitNeuralNet(0.8,0.1,0.1,2,32,64,35)
        state_dict = torch.load(os.path.join(package_path, 'model/fusion_model.pth'))
        self.model.load_state_dict(state_dict)
        self.model.eval()

        # Init Thread -> Temporal Window, Recognition
        self.temporal_window_thread = MyThread()
        self.recognition_thread = MyThread()

        # Init Variables
        self.init_variables()

        rospy.logwarn('Multimodal Fusion Initialized')

    def init_variables(self):

        self.voiceCheck = False
        self.gestureCheck = False
        self.gesture_msgCheck = False
        self.areaCheck = False

        self.last_area = None

        self.delay = 4
        self.recognition_time = 2
        self.recognition_percentage = 80

        self.gesture_vector = None

        self.voice_msg = None
        self.gesture_msg = None
        self.area_msg = None

        self.network_input = (0,0)

        self.command = None

        self.thread_active = False
        self.thread_paused = False
        self.recognition_thread_active = False

    def voiceCallback(self, data:Int32MultiArray):

        # Save Voice Message
        self.voice_msg = data.data
        #self.voice_time_stamp = datetime.datetime.now()
        rospy.logwarn(f"Received Voice Command: {self.voice_msg}")

        # Set Voice Message Flag
        self.voiceCheck = True

    def gestureCallback(self, data:Int32MultiArray):

        # Save Gesture Message
        self.gesture_msg = data.data
        #self.gesture_time_stamp = datetime.datetime.now()
        # rospy.logwarn(f"Received Gesture Command: {self.gesture_msg}")

        # Open Recognition Thread
        if self.recognition_thread_active == False:

            self.recognition_thread_active = True
            self.recognition_thread = MyThread()
            self.recognition_thread.start()
            rospy.loginfo("Gesture Recognition Started")

        # If Mono-dimensional: Overwrite | else: Append
        if self.gesture_vector is None: self.gesture_vector = self.gesture_msg
        else: self.gesture_vector = self.gesture_vector + self.gesture_msg

    def areaCallback(self, data:Int32MultiArray):

        # Save Area Message
        self.area_msg = data.data 
        # rospy.logwarn(f"Received Area Command: {self.area_msg}")

        # Set Area Flag
        self.areaCheck = True

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

    def handover(self, tts_start, position, start_grip, end_grip, tts_end):

        # Node-RED Start TTS
        self.ttsPub.publish(tts_start)

        # Move Gripper to Starting Position (Open/Close for Internal/External Gripping)
        self.move_gripper(start_grip)

        # Move to Intermediate Position
        self.move_joint(intermediatePos)

        # Forward Kinematic -> Increase z + 40cm
        cartesian_pose: Pose() = self.FK(position)
        cartesian_pose.position.z += 0.40

        # cartesian_pose = CartesianPoint()
        # cartesian_response.tcp_position.position.z += 0.40
        # cartesian_pose.cartesian_pose = cartesian_response.tcp_position
        # cartesian_pose = cartesian_response.tcp_position
        # cartesian_pose.velocity = 0.2

        # Cartesian Movement -> 40cm Over the Object
        self.move_cartesian(cartesian_pose)

        # Move to Object
        self.move_joint(position) 
        time.sleep(1)

        # Grip Object
        self.move_gripper(end_grip)

        # Cartesian Movement -> 40cm Over the Object
        self.move_cartesian(cartesian_pose)
        time.sleep(0.5)

        # Move to Intermediate Position
        self.move_joint(intermediatePos)

        # Forward Kinematic -> Increase z + 40cm
        cartesian_pose: Pose() = self.FK(placePos)
        cartesian_pose.position.z += 0.40
        
        # Cartesian Movement -> 40cm Over the Object
        self.move_cartesian(cartesian_pose)
        time.sleep(1)

        # Move to Place Position
        self.move_joint(placePos) 

        # Release Object
        self.move_gripper(start_grip)

        # Forward Kinematic -> Increase z + 20cm
        cartesian_pose: Pose() = self.FK(placePos)
        cartesian_pose.position.z += 0.20

        # Cartesian Movement -> 20cm Over the Object
        self.move_cartesian(cartesian_pose)

        # Node-RED TTS Response
        self.ttsPub.publish(tts_end)
        time.sleep(1)

        # Move to Home
        self.move_joint(defaultPos)

    def stopRobot(self):

        rospy.wait_for_service('/ur_rtde/controllers/stop_robot')
        self.stop_req = TriggerRequest()
        self.stop_response = self.stop_service(self.stop_req)

    def cleaner(self):
         
        self.voiceCheck = False
        self.gestureCheck = False
        self.areaCheck = False

        self.gesture_vector = None
        self.command = None
        self.last_area = None

        self.voice_msg = None
        self.gesture_msg = None
        self.area_msg = None

        self.network_input = (0,0)

    def Counter(self):

        # Conta le occorrenze di ciascun elemento nella tupla
        conteggio = Counter(self.gesture_vector)
    
        # Trova l'elemento più frequente e il numero di volte che appare
        elemento_piu_frequente, frequenza_piu_alta = conteggio.most_common(1)[0]
    
        # Calcola la percentuale di frequenza rispetto alla lunghezza della tupla
        percentuale_frequenza = (frequenza_piu_alta / len(self.gesture_vector)) * 100
    
        # Verifica se l'elemento più frequente è presente per almeno l'80% dei casi
        if percentuale_frequenza >= self.recognition_percentage:
            print("Ho riconosciuto il gesto {}".format(elemento_piu_frequente))
            return elemento_piu_frequente

        else:
            print("Comando Gestuale Non Riconosciuto")
        
        self.gesture_vector = None

    def classificatorNetwork(self, input):

        input = torch.tensor(input, dtype= torch.float32)     # 
        output = self.model(input)
        prediction = output.argmax(dim=0)
        command = prediction.item()
        print("Numero del comando:", command)

        return command 

    def timeManager(self):
        
        #Se il timer è scaduto
        if int(self.recognition_thread.time) >= int(self.recognition_time):

            self.gesture_msg = None

            #Se il gesto è stato riconosciuto
            if self.gesture_vector is not None:

                print("Il vettore gesto è: {}".format(self.gesture_vector))

                #Se il gesto è stato pubblcato un numero di volte
                if len(self.gesture_vector) > 3:

                    #Prendi il più riconosciuto
                    self.gesture_recognised = self.Counter()

                    #Se il gesto è stato riconosciuto
                    if self.gesture_recognised is not None:

                        self.gesture_vector = None
                        self.gestureCheck = True

                #Se il gesto è stato riconosciuto una sola volta
                else:
                    print("Comando Gestuale pubblicato meno di tre volte")
                    self.gesture_vector = None

 
            self.recognition_thread.stop()
            self.recognition_thread.join()
            self.recognition_thread_active = False

        #Arriva l'area indicata
        if self.areaCheck == True:

            if self.area_msg[0] != self.last_area:

                self.areaCheck = False
                self.last_area = self.area_msg[0]
                print("Area Indicata: {}".format(self.last_area))
                self.area_msg = None

        #Arriva il comando gestuale 
        if self.gestureCheck == True:
                
            #Se il comando non appartiene a quelli per manipolare il time manager:
            if ((self.gesture_recognised != 3) and (self.gesture_recognised != 2)) or ((self.gesture_recognised == 3) and (self.gesture_recognised == 2)) : #A xor B

                #Se il vettore è vuoto o sto sovrascrivendo un vettore già esistente
                if ((self.network_input == (0,0)) and (self.network_input[0] == 0 )) or ((self.network_input != (0,0)) and (self.network_input[0] != 0)):

                    #Se il comando è diverso da quello precedente
                    if self.network_input[0] != self.gesture_recognised:

                        #Se il timer stava già andando interrompilo e ricomincia
                        if self.thread_active == True:

                            self.temporal_window_thread.stop()
                            self.temporal_window_thread.join()
                            self.thread_active = False

                            #Apri un nuovo timer da zero 
                            self.thread_active = True
                            self.temporal_window_thread = MyThread()
                            self.temporal_window_thread.start()
                            print("Timer Interrotto e Ripartito")

                        elif self.thread_active == False:

                            if self.thread_paused == True:

                                self.temporal_window_thread.resume()
                                self.thread_paused = False
                                self.thread_active = True
                                print("Timer Ripartito")

                            if self.thread_paused == False:

                                #Apri un nuovo timer da zero 
                                self.thread_active = True
                                self.temporal_window_thread = MyThread()
                                self.temporal_window_thread.start()
                                print("Timer Partito")

                    elif self.network_input[0] == self.gesture_recognised and self.thread_paused == True:
                            
                        self.temporal_window_thread.resume()
                        self.thread_paused = False
                        self.thread_active = True
                        print("Timer Ripartito anche se il gesto era lo stesso")

                    else:
                        print("Gesto già catturato prima")

                #Aggiorna il comando se appartiene a quelli predisposti alla fusione
                self.network_input = (self.gesture_recognised,) + self.network_input[1:]

            #Se il comando appartiene a quelli per manipolare il time manager: in questo caso Pause
            elif self.gesture_recognised == 3:

                #Se il timer stava già andando interrompilo 
                if self.thread_active == True:

                    self.temporal_window_thread.pause()
                    self.thread_active = False
                    self.thread_paused = True
                    print("Timer messo in pausa Manualmente")

                elif self.thread_active == False:

                    print("Il Timer non stava Andando, non puoi metterlo in pausa")

            #Se il comando appartiene a quelli per manipolare il time manager: in questo caso stop
            elif self.gesture_recognised == 2:

                #Se il timer stava già andando interrompilo 
                if self.thread_active == True:

                    self.temporal_window_thread.stop()
                    self.temporal_window_thread.join()
                    self.thread_active = False
                    # self.stopRobot()                     #TODO DEVEFUNZIONARE ALTRIMENTI TOGLI
                    print("Timer Stoppato Manualmente")
                    self.cleaner()  #Questo è fondamentale, è la differenza fra stop e pause
                
                elif self.thread_active == False:
                        
                        print("Il Timer non stava Andando, non puoi fermarlo")

            else:
                print("Comando Gestuale Non Riconosciuto")

            self.gestureCheck = False
            self.gesture_recognised = None

        #Arriva un comando vocale
        if self.voiceCheck == True:

            if ((self.network_input == (0,0)) and (self.network_input[1] == 0 )) or ((self.network_input != (0,0)) and (self.network_input[1] != 0)):

                #Se il comando che arriva è
                if self.network_input[1] != self.voice_msg:

                    #Se il timer stava già andando interrompilo e ricomincia
                    if self.thread_active == True:
                    
                        #Interrompo il vecchio
                        self.temporal_window_thread.stop()
                        self.temporal_window_thread.join()

                        #Riapro uno nuovo
                        self.temporal_window_thread = MyThread()
                        self.temporal_window_thread.start()
                        print("Timer Interrotto e Ripartito")

                    elif self.thread_active == False:
                    
                        if self.thread_paused == True:
                        
                            #Fai ripartire il nuovo timer
                            self.temporal_window_thread.resume()
                            self.thread_paused = False
                            print("Timer Ripartito")

                        elif self.thread_paused == False:
                        
                            #Apri un nuovo timer da zero 
                            self.thread_active = True
                            self.temporal_window_thread = MyThread()
                            self.temporal_window_thread.start()
                            print("Timer Partito")

                else:
                    print("Comando vocale già catturato prima")

            #Ora alloco il nuovo comando vocale
            self.network_input = self.network_input[:1] + self.voice_msg
            self.voiceCheck = False

        #When the timer is finished -> recognition = fusion
        if int(self.temporal_window_thread.time) >= int(self.delay) and self.recognition_thread_active == False:

            #Interrompi il timer
            print("Timer finito")
            self.temporal_window_thread.stop()
            self.temporal_window_thread.join()
            self.thread_active = False
    
            self.command = self.classificatorNetwork(self.network_input)
            print("\nComando ricevuto: {}, dal vettore {} ".format(self.command, self.network_input))

            #Vettore nullo, non so nemmeno come sia partito il timer
        
            if self.command == 0:

                print("Il vettore era nullo")

            #point at + oggetto generico
            if self.command >= 1 and self.command <= 5: 

                if self.last_area is not None:

                    objecttoTake = int(self.network_input[1]) + int(self.last_area)

                    #Puntato l'oggetto, controllo se si trova nell'area destra
                    if objecttoTake in RIGHT_AREA.values():

                        for key, values in RIGHT_AREA.items():
                            if values == objecttoTake:
                                objecttoTake = key

                                tts_inziale = "Ti prendo {} nell'area destra".format(objecttoTake)
                                tts_finale = "Ecco {} che mi hai chiesto".format(objecttoTake)
                                print(tts_inziale)
                                self.handover(tts_inziale, OBJECTS[key][0], OBJECTS[key][1], OBJECTS[key][2], tts_finale)
                                print(tts_finale)

                    #Puntato l'oggetto, controllo se si trova nell'area sinistra
                    elif objecttoTake in LEFT_AREA.values():

                        for key, values in LEFT_AREA.items():
                            if values == objecttoTake:
                                objecttoTake = key

                                tts_inziale = "Ti prendo {} nell'area sinistra".format(objecttoTake)
                                tts_finale = "Ecco {} che mi hai chiesto".format(objecttoTake)
                                print(tts_inziale)
                                self.handover(tts_inziale, OBJECTS[key][0], OBJECTS[key][1], OBJECTS[key][2], tts_finale)
                                print(tts_finale)
                                
                    else: 
                        tts = "Mi dispiace, hai indicato un'area che non conosco"
                        print(tts)
                        self.ttsPub.publish(tts)
                
                else:
                    tts = "Mi dispiace, non hai indicato nessuna area"
                    print(tts)
                    self.ttsPub.publish(tts)

            #0 + oggetto generico
            if self.command >= 6 and self.command <= 10:

                tts = "Mi dispiace, c'è più di un oggetto che corrisponde alla tua descrizione, ricorda che puoi sempre chiedermi informazioni sugli OBJECTS per tipo o per area, oppure puoi indicarmi l'oggetto che vuoi!"
                print(tts)
                self.ttsPub.publish(tts)

            #0 + oggetto specifico
            if self.command >= 11 and self.command <= 21:

                objecttoTake = int(self.network_input[1])

                for key, values in RIGHT_AREA.items():
                            #Se l'oggetto specifico nominato si trova lì prendilo
                            if values == objecttoTake:
                                objecttoTake = key
                                object_check = True

                                tts_inziale = "Ti prendo {} nell'area destra".format(objecttoTake)
                                print(tts_inziale)
                                tts_finale = "Ecco {} che mi hai chiesto".format(objecttoTake)
                                self.handover(tts_inziale, OBJECTS[key][0], OBJECTS[key][1], OBJECTS[key][2], tts_finale)
                                print(tts_finale)


                for key, values in LEFT_AREA.items():

                            #Se l'oggetto specifico nominato si trova lì prendilo
                            if values == objecttoTake:
                                objecttoTake = key
                                object_check = True

                                tts_inziale = "Ti prendo {} nell'area sinistra".format(objecttoTake)
                                print(tts_inziale)
                                tts_finale = "Ecco {} che mi hai chiesto".format(objecttoTake)
                                self.handover(tts_inziale, OBJECTS[key][0], OBJECTS[key][1], OBJECTS[key][2], tts_finale)
                                print(tts_finale)


            #point at + oggetto specifico
            if self.command >= 22 and self.command <= 32:

                objecttoTake = int(self.network_input[1])
                object_check = False

                if self.last_area is not None:

                    if self.last_area == 1:

                        for key, values in RIGHT_AREA.items():
                            #Se l'oggetto specifico nominato si trova lì prendilo
                            if values == objecttoTake:
                                objecttoTake = key
                                object_check = True

                                tts_inziale = "Ti prendo {} nell'area destra".format(objecttoTake)
                                print(tts_inziale)
                                tts_finale = "Ecco {} che mi hai chiesto".format(objecttoTake)
                                self.handover(tts_inziale, OBJECTS[key][0], OBJECTS[key][1], OBJECTS[key][2], tts_finale)
                                print(tts_finale)

                    #Se ho indicato l'area di sinistra
                    elif self.last_area == 2:

                        for key, values in LEFT_AREA.items():

                            #Se l'oggetto specifico nominato si trova lì prendilo
                            if values == objecttoTake:
                                objecttoTake = key
                                object_check = True

                                tts_inziale = "Ti prendo {} nell'area sinistra".format(objecttoTake)
                                print(tts_inziale)
                                tts_finale = "Ecco {} che mi hai chiesto".format(objecttoTake)
                                self.handover(tts_inziale, OBJECTS[key][0], OBJECTS[key][1], OBJECTS[key][2], tts_finale)
                                print(tts_finale)

                    else:
                        
                        tts = "Mi dispiace, non conosco l'area che hai indicato"
                        print(tts)
                        self.ttsPub.publish(tts)

                else:

                    tts = "Mi dispiace, non hai indicato nessuna area"
                    print(tts)
                    self.ttsPub.publish(tts)

                if object_check == False:

                    if self.last_area is not None:

                        if self.last_area == 1:

                            tts = "Mi dispiace, l'oggetto che cerchi non si trova nell'area destra"
                            print(tts)
                            self.ttsPub.publish(tts)

                        elif self.last_area == 2:
                            
                            tts = "Mi dispiace, l'oggetto che cerchi non si trova nell'area sinistra"
                            print(tts)
                            self.ttsPub.publish(tts)
                            
            #point at + 0
            if self.command == 33:

                if self.last_area is not None:

                    if self.last_area == 1:

                        print("Mi hai indicato qualcosa nell'area destra, ma non capisco cosa tu voglia fare")

                    elif self.last_area == 2:

                        print("Mi hai indicato qualcosa nell'area sinistra, ma non capisco cosa tu voglia fare")
                    else:
                        print("Mi hai indicato qualcosa ma non conosco quest'area")
                else:
                    print("Hai provato a indicarmi qualcosa ma non ho capito nè cosa vuoi fare nè dove hai puntato")

            if self.command == 34:

                print(1)
                if self.last_area is not None:
                    print(2)
                    if self.last_area == 1:

                        tts = "Nell'area che mi hai indicato sono presenti {}".format(', '.join(RIGHT_AREA))
                        print(tts)
                        self.ttsPub.publish(tts)

                    elif self.last_area == 2:

                        tts = "Nell'area che mi hai indicato sono presenti {}".format(', '.join(LEFT_AREA) )

                    else:

                        tts = "Mi dispiace, non conosco quest'area"
                        print(tts)
                        self.ttsPub.publish(tts)

                else:

                    tts = "Mi dispiace, non hai indicato nessuna area"
                    print(tts)
                    self.ttsPub.publish(tts)

            if self.thread_active == True:
                print("Timer finito")
                self.temporal_window_thread.stop()
                self.temporal_window_thread.join()
                self.thread_active = False

            self.cleaner()

class MyThread(threading.Thread):

    def __init__(self):
        super().__init__()
        self.stop_event = threading.Event()
        self.paused = threading.Event()
        self.paused.set()
        self.time = 0

    def run(self):

        while not self.stop_event.is_set() and not rospy.is_shutdown():
            self.paused.wait()
            self.time += 1
            rospy.sleep(1)
            if not self.time  == 0: print("Counter", self.time)

    def resume(self):
        self.paused.set()

    def pause(self):
        self.paused.clear()

    def stop(self):
        self.stop_event.set()
        self.time = 0

if __name__ == '__main__':

    # Initialize Multimodal Fusion Node
    multimodal_fusion = Fusion()

    # Run Time Manager
    while not rospy.is_shutdown():
        multimodal_fusion.timeManager()
