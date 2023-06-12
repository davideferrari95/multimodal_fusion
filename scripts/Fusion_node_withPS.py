#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray, String, Bool 
import datetime 
import torch
from Rete_neurale_classificatrice import LitNeuralNet
import threading, time
from geometry_msgs.msg import Pose
#from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from ur_rtde_controller.msg import CartesianPoint
from ur_rtde_controller.srv import RobotiQGripperControl, RobotiQGripperControlRequest
from ur_rtde_controller.srv import GetForwardKinematic, GetForwardKinematicRequest
from collections import Counter
from std_srvs.srv import Trigger, TriggerRequest 



class Classificator:

    def __init__(self):


        # Inizializzazione del nodo
        rospy.init_node('classificatore', anonymous=True)

        #init variabili
        self.gripper_aperto = 100
        self.gripper_chiuso = 0
        self.gripper_70 = 70

        self.defaultPos=[2.531209945678711, -1.8816501102843226, 1.7585914770709437, -1.4168628019145508, 4.700905799865723, 0.7452919483184814]
        self.placePos=[1.4981036186218262, -1.8520351848998011, 2.422215286885397, -2.14617981533193, 4.746311187744141, -0.03622609773744756]
        self.intermediatePos = [2.8941173553466797, -1.249845342045166, 1.0492914358722132, -1.3367853921702881, 4.713375091552734, 1.1106750965118408]

        self.oggetti={
            
            "il sale grosso":[
            [2.5016584396362305, -0.8632076543620606, 1.2751339117633265, -1.963386674920553, 4.741418838500977, 0.9712827205657959],
            self.gripper_aperto,
            self.gripper_chiuso,
            ],

            "il sale iodato":[
            [3.387843608856201, -1.0682671827128907, 1.6490314642535608, -2.1364666424193324, 4.772143363952637, 1.8173348903656006],
            self.gripper_aperto,
            self.gripper_chiuso,
            ],

            "il bicchiere bianco":[
            [2.3846733570098877, -1.0724294942668458, 1.6509855429278772, -2.178251882592672, 4.730436325073242, 0.85353684425354],
            self.gripper_chiuso,
            self.gripper_aperto,
            ],

            "la passata":[
            [2.3381829261779785, -0.8568876546672364, 1.3088882605182093, -2.0508209667601527, 4.731287956237793, 0.8080871105194092],
            self.gripper_chiuso,
            self.gripper_aperto,
            ], 

            "il coltello":[
            [2.2267518043518066, -1.1784547132304688, 2.059087578450338, -2.477241178552145, 4.735124588012695, 0.6946213245391846],
            self.gripper_aperto,
            self.gripper_chiuso,
            ], 

            "la forchetta di legno":[
            [2.293628692626953, -1.0718673032573243, 1.85292894044985, -2.3308073482909144, 4.7392730712890625, -0.8873665968524378],
            self.gripper_aperto,
            self.gripper_chiuso,
            ], 

            "le pennette":[
            [2.1489624977111816, -1.074343041782715, 1.6118515173541468, -2.0838972530760707, 4.7358078956604, -1.0315120855914515],
            self.gripper_aperto,
            self.gripper_chiuso,
            ], 

            "la forchetta di plastica":[
            [3.3018717765808105, -1.2413643163493653, 2.0898597876178187, -2.3902908764281214, 4.729753017425537, -1.270754639302389],
            self.gripper_aperto,
            self.gripper_chiuso,
            ], 

            "i pelati":[
            [3.3865585327148438, -1.4877928060344239, 2.2749279181109827, -2.3304363689818324, 4.7321391105651855, -1.1869919935809534],
            self.gripper_chiuso,
            self.gripper_aperto,
            ], 

            "il bicchiere trasparente":[
            [3.4324843883514404, -1.2404654783061524, 1.9565780798541468, -2.2602154217162074, 4.73328971862793, -1.139892880116598],
            self.gripper_chiuso,
            self.gripper_aperto,
            ],

            "gli spaghetti":[
            [3.595914840698242, -1.1588075918010254, 1.9754837195025843, -2.4072934589781703, 4.74080753326416, 0.44310879707336426],
            self.gripper_aperto,
            self.gripper_chiuso,
            ]
                    }
    
        self.area_destra = {

            "la passata": 11,
            "il sale grosso": 21, 
            "il bicchiere bianco": 61,
            "le pennette": 31,
            "il coltello": 51,
            "la forchetta di legno": 41,
        }

        self.area_sinistra = {

            "il sale iodato": 22,
            "il bicchiere trasparente": 62,
            "gli spaghetti": 32,
            "la forchetta di plastica": 42,
            "i pelati": 12, 
            
        }

        #init oggetto posizione in giunti(se time_from_start=0 allora si muove in velocità)
        self.destinationPos=JointTrajectoryPoint()
        self.destinationPos.time_from_start = rospy.Duration(0)
        self.destinationPos.velocities = [0.4]

        #Publisher per i movimenti in posa e in cartesiano 
        self.ur10Pub=rospy.Publisher('/ur_rtde/controllers/joint_space_controller/command',JointTrajectoryPoint,queue_size=10)
        self.ur10PubCartesian=rospy.Publisher('/ur_rtde/controllers/cartesian_space_controller/command',CartesianPoint,queue_size=10)

        #init service per il gripper
        self.gripper_srv = rospy.ServiceProxy('/ur_rtde/robotiq_gripper/command', RobotiQGripperControl)
        self.gripper_req = RobotiQGripperControlRequest()

        #init service per cinematica inversa 
        self.cartesian_srv = rospy.ServiceProxy('ur_rtde/getFK', GetForwardKinematic)

        self.stop_service = rospy.ServiceProxy('/ur_rtde/controllers/stop_robot', Trigger)
        self.stop_req = TriggerRequest()

        # Inizializzazione dei topic
        rospy.Subscriber("voice", Int32MultiArray, self.voiceCallback)
        rospy.Subscriber("gesture", Int32MultiArray, self.gestureCallback)
        rospy.Subscriber("area",Int32MultiArray, self.areaCallback)
        self.ttsPub = rospy.Publisher('/tts',String,queue_size=1)

        # TODO: cambia path
        # init network
        self.model = LitNeuralNet(0.8,0.1,0.1,2,32,64,35)
        state_dict = torch.load(r"/home/davide/ROS/niryo_ws/src/Multimodal Fusion/fusion/model/fusion_model.pth")
        self.model.load_state_dict(state_dict)
        self.model.eval()

        # Apri thread finestra temporale e recognition
        self.my_thread = MyThread()
        self.recognition_thread = MyThread()

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

    def voiceCallback(self,data):

       self.voice_msg = data.data 
       #self.voice_time_stamp = datetime.datetime.now()
       print("\nComando vocale {} ricevuto".format(self.voice_msg))
       self.voiceCheck = True

    def gestureCallback(self, data):

       self.gesture_msg = data.data 
       #self.gesture_time_stamp = datetime.datetime.now()
       #print("\nComando gestuale: {} ricevuto".format(self.gesture_msg))

       if self.recognition_thread_active == False:

        self.recognition_thread_active = True
        self.recognition_thread = MyThread()
        self.recognition_thread.start()
        print("Riconoscimento Partito")

       if self.gesture_vector is None: self.gesture_vector = self.gesture_msg  #Se è monodimensionale, sostituisci
       else: self.gesture_vector = self.gesture_vector + self.gesture_msg     #Se è più grande appendi

    def areaCallback(self, data):

        self.area_msg = data.data 
        # print("\nArea: {} ricevuto".format(self.area_msg))
        self.areaCheck = True

    #function for close/open gripper
    def gripping(self,position):

        # self.gripper_req.position, self.gripper_req.speed, self.gripper_req.force = 100, 100, 25
        self.gripper_req.position = position
        rospy.wait_for_service('/ur_rtde/robotiq_gripper/command')
        self.gripper_response = self.gripper_srv(self.gripper_req)
        '''if gripper_response.status != "2":
        raise Exception("Sorry, An exception occurred")'''

    def handover(self,tt_inziale,position,start_grip, end_grip, tts_finale):

        #pubblico il messaggio vocale
        self.ttsPub.publish(tt_inziale)

        #posizione iniziale (aperto o chiuso in modo da gripp interno o esterno)
        self.gripping(start_grip)

        #vado a mettermi in una posizione centrale
        self.intermidatePosition()

        #chiamata servise cinemarica inversa per andare sopra l'oggetto
        cartesian_req= GetForwardKinematicRequest()
        cartesian_req.joint_position=position
        rospy.wait_for_service('ur_rtde/getFK')
        cartesian_response = self.cartesian_srv(cartesian_req)
        print(cartesian_response)

        #init posizione cartesiana
        cartesian_pose = CartesianPoint()

        #alzo la z di 40cm
        cartesian_response.tcp_position.position.z+=0.40
        cartesian_pose.cartesian_pose = cartesian_response.tcp_position

        #velocita di salita e discesa in cartesiano
        cartesian_pose.velocity = 0.2

        #pubblico come prima posizione la posizione in cartesiano alzata di 40 cm 
        self.ur10PubCartesian.publish(cartesian_pose)
        flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
        if flag.data is not True:
            raise Exception("Sorry, An exception occurred") 
        #time.sleep(1)

        #destinazione in giunti dell'oggetto
        self.destinationPos.positions=position
        self.ur10Pub.publish(self.destinationPos)
        flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
        if flag.data is not True:
            raise Exception("Sorry, An exception occurred") 
        time.sleep(1)

        #chiusura/ apertura del gripper
        self.gripping(end_grip)

        #ripubblico la posizione alzata di 40 cm in cartesiano
        self.ur10PubCartesian.publish(cartesian_pose)
        flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
        if flag.data is not True:
            raise Exception("Sorry, An exception occurred") 
        time.sleep(0.5)

        self.intermidatePosition()

        #chiamata servise cinemarica inversa
        cartesian_req= GetForwardKinematicRequest()
        cartesian_req.joint_position= self.placePos
        rospy.wait_for_service('ur_rtde/getFK')
        cartesian_response = self.cartesian_srv(cartesian_req)
        print(cartesian_response)

        #init posizione cartesiana
        cartesian_pose = CartesianPoint()

        #alzo la z di 40cm
        cartesian_response.tcp_position.position.z+=0.40
        cartesian_pose.cartesian_pose = cartesian_response.tcp_position

        #velocita di salita e discesa in cartesiano
        cartesian_pose.velocity = 0.4

        #pubblico come prima posizione la posizione in cartesiano alzata di 40 cm 
        self.ur10PubCartesian.publish(cartesian_pose)
        flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
        if flag.data is not True:
            raise Exception("Sorry, An exception occurred") 
        time.sleep(1)


        self.destinationPos.positions=self.placePos
        self.ur10Pub.publish(self.destinationPos)
        flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
        if flag.data is not True:
            raise Exception("Sorry, An exception occurred")
        
        self.gripping(start_grip)

        #chiamata servise cinemarica inversa
        cartesian_req= GetForwardKinematicRequest()
        cartesian_req.joint_position= self.placePos
        rospy.wait_for_service('ur_rtde/getFK')
        cartesian_response = self.cartesian_srv(cartesian_req)
        print(cartesian_response)

        #init posizione cartesiana
        cartesian_pose = CartesianPoint()

        #alzo la z di 40cm
        cartesian_response.tcp_position.position.z+=0.20
        cartesian_pose.cartesian_pose = cartesian_response.tcp_position

        #velocita di salita e discesa in cartesiano
        cartesian_pose.velocity = 0.4

        #pubblico come prima posizione la posizione in cartesiano alzata di 40 cm 
        self.ur10PubCartesian.publish(cartesian_pose)
        flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
        if flag.data is not True:
            raise Exception("Sorry, An exception occurred")

        #Risposta con Nodered
        self.ttsPub.publish(tts_finale)
        time.sleep(1)

        self.returningPosition()

    def stopRobot(self):

        rospy.wait_for_service('/ur_rtde/controllers/stop_robot')
        self.stop_req = TriggerRequest()
        self.stop_response = self.stop_service(self.stop_req)

    def returningPosition(self):

        self.destinationPos.positions=self.defaultPos
        self.destinationPos.velocities = [0.8]
        self.ur10Pub.publish(self.destinationPos)
        flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
        if flag.data is not True:
            raise Exception("Sorry, An exception occurred")
        
    def intermidatePosition(self):

        self.destinationPos.positions=self.intermediatePos
        self.destinationPos.velocities = [0.7]
        self.ur10Pub.publish(self.destinationPos)
        flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
        if flag.data is not True:
            raise Exception("Sorry, An exception occurred")
        
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

                            self.my_thread.stop()
                            self.my_thread.join()
                            self.thread_active = False

                            #Apri un nuovo timer da zero 
                            self.thread_active = True
                            self.my_thread = MyThread()
                            self.my_thread.start()
                            print("Timer Interrotto e Ripartito")

                        elif self.thread_active == False:

                            if self.thread_paused == True:

                                self.my_thread.resume()
                                self.thread_paused = False
                                self.thread_active = True
                                print("Timer Ripartito")

                            if self.thread_paused == False:

                                #Apri un nuovo timer da zero 
                                self.thread_active = True
                                self.my_thread = MyThread()
                                self.my_thread.start()
                                print("Timer Partito")

                    elif self.network_input[0] == self.gesture_recognised and self.thread_paused == True:
                            
                        self.my_thread.resume()
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

                    self.my_thread.pause()
                    self.thread_active = False
                    self.thread_paused = True
                    print("Timer messo in pausa Manualmente")

                elif self.thread_active == False:

                    print("Il Timer non stava Andando, non puoi metterlo in pausa")

            #Se il comando appartiene a quelli per manipolare il time manager: in questo caso stop
            elif self.gesture_recognised == 2:

                #Se il timer stava già andando interrompilo 
                if self.thread_active == True:

                    self.my_thread.stop()
                    self.my_thread.join()
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
                        self.my_thread.stop()
                        self.my_thread.join()

                        #Riapro uno nuovo
                        self.my_thread = MyThread()
                        self.my_thread.start()
                        print("Timer Interrotto e Ripartito")

                    elif self.thread_active == False:
                    
                        if self.thread_paused == True:
                        
                            #Fai ripartire il nuovo timer
                            self.my_thread.resume()
                            self.thread_paused = False
                            print("Timer Ripartito")

                        elif self.thread_paused == False:
                        
                            #Apri un nuovo timer da zero 
                            self.thread_active = True
                            self.my_thread = MyThread()
                            self.my_thread.start()
                            print("Timer Partito")

                else:
                    print("Comando vocale già catturato prima")

            #Ora alloco il nuovo comando vocale
            self.network_input = self.network_input[:1] + self.voice_msg
            self.voiceCheck = False

        #When the timer is finished -> recognition = fusion
        if int(self.my_thread.time) >= int(self.delay) and self.recognition_thread_active == False:

            #Interrompi il timer
            print("Timer finito")
            self.my_thread.stop()
            self.my_thread.join()
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
                    if objecttoTake in self.area_destra.values():

                        for key, values in self.area_destra.items():
                            if values == objecttoTake:
                                objecttoTake = key

                                tts_inziale = "Ti prendo {} nell'area destra".format(objecttoTake)
                                tts_finale = "Ecco {} che mi hai chiesto".format(objecttoTake)
                                print(tts_inziale)
                                self.handover(tts_inziale, self.oggetti[key][0], self.oggetti[key][1], self.oggetti[key][2], tts_finale)
                                print(tts_finale)

                    #Puntato l'oggetto, controllo se si trova nell'area sinistra
                    elif objecttoTake in self.area_sinistra.values():

                        for key, values in self.area_sinistra.items():
                            if values == objecttoTake:
                                objecttoTake = key

                                tts_inziale = "Ti prendo {} nell'area sinistra".format(objecttoTake)
                                tts_finale = "Ecco {} che mi hai chiesto".format(objecttoTake)
                                print(tts_inziale)
                                self.handover(tts_inziale, self.oggetti[key][0], self.oggetti[key][1], self.oggetti[key][2], tts_finale)
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

                tts = "Mi dispiace, c'è più di un oggetto che corrisponde alla tua descrizione, ricorda che puoi sempre chiedermi informazioni sugli oggetti per tipo o per area, oppure puoi indicarmi l'oggetto che vuoi!"
                print(tts)
                self.ttsPub.publish(tts)

            #0 + oggetto specifico
            if self.command >= 11 and self.command <= 21:

                objecttoTake = int(self.network_input[1])

                for key, values in self.area_destra.items():
                            #Se l'oggetto specifico nominato si trova lì prendilo
                            if values == objecttoTake:
                                objecttoTake = key
                                object_check = True

                                tts_inziale = "Ti prendo {} nell'area destra".format(objecttoTake)
                                print(tts_inziale)
                                tts_finale = "Ecco {} che mi hai chiesto".format(objecttoTake)
                                self.handover(tts_inziale, self.oggetti[key][0], self.oggetti[key][1], self.oggetti[key][2], tts_finale)
                                print(tts_finale)


                for key, values in self.area_sinistra.items():

                            #Se l'oggetto specifico nominato si trova lì prendilo
                            if values == objecttoTake:
                                objecttoTake = key
                                object_check = True

                                tts_inziale = "Ti prendo {} nell'area sinistra".format(objecttoTake)
                                print(tts_inziale)
                                tts_finale = "Ecco {} che mi hai chiesto".format(objecttoTake)
                                self.handover(tts_inziale, self.oggetti[key][0], self.oggetti[key][1], self.oggetti[key][2], tts_finale)
                                print(tts_finale)


            #point at + oggetto specifico
            if self.command >= 22 and self.command <= 32:

                objecttoTake = int(self.network_input[1])
                object_check = False

                if self.last_area is not None:

                    if self.last_area == 1:

                        for key, values in self.area_destra.items():
                            #Se l'oggetto specifico nominato si trova lì prendilo
                            if values == objecttoTake:
                                objecttoTake = key
                                object_check = True

                                tts_inziale = "Ti prendo {} nell'area destra".format(objecttoTake)
                                print(tts_inziale)
                                tts_finale = "Ecco {} che mi hai chiesto".format(objecttoTake)
                                self.handover(tts_inziale, self.oggetti[key][0], self.oggetti[key][1], self.oggetti[key][2], tts_finale)
                                print(tts_finale)

                    #Se ho indicato l'area di sinistra
                    elif self.last_area == 2:

                        for key, values in self.area_sinistra.items():

                            #Se l'oggetto specifico nominato si trova lì prendilo
                            if values == objecttoTake:
                                objecttoTake = key
                                object_check = True

                                tts_inziale = "Ti prendo {} nell'area sinistra".format(objecttoTake)
                                print(tts_inziale)
                                tts_finale = "Ecco {} che mi hai chiesto".format(objecttoTake)
                                self.handover(tts_inziale, self.oggetti[key][0], self.oggetti[key][1], self.oggetti[key][2], tts_finale)
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

                        tts = "Nell'area che mi hai indicato sono presenti {}".format(', '.join(self.area_destra))
                        print(tts)
                        self.ttsPub.publish(tts)

                    elif self.last_area == 2:

                        tts = "Nell'area che mi hai indicato sono presenti {}".format(', '.join(self.area_sinistra) )

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
                self.my_thread.stop()
                self.my_thread.join()
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

        while not self.stop_event.is_set():
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

    classificatore = Classificator()

    print("-- Nodo Multimodale Pronto --")
    
    while not rospy.is_shutdown():
        classificatore.timeManager()
