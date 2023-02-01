#!/usr/bin/env python3

import rospy
#from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import datetime 
import torch
from Rete_neurale import LitNeuralNet

"""def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data) """
    


class Classificator():

    voce_ricevuto = False
    gesto_ricevuto = False
    t = 2 
    timestamp1 = None
    timestamp2 = None

    def __init__(self):
    
        # Inizializzazione del nodo
        rospy.init_node('classificatore', anonymous=True)
    
        # Inizializzazione dei topic
        rospy.Subscriber("voce", Float32MultiArray, self.callback_voce)
        rospy.Subscriber("gesto", Float32MultiArray, self.callback_gesto)
    
        self.model = LitNeuralNet(0.7,0.15,0.15,6,32,64,9)
        state_dict = torch.load("/home/alberto/catkin_ws/src/fusion/scripts/Rete_allenata.pth")
        self.model.load_state_dict(state_dict)
        self.model.eval()
        
        #Aspetta che abbia finito 
        #rospy.spin()

    def callback_voce(self,data):
    
        #Dichiaro la variabile globale vettorevoce
    
        # Funzione di callback per il topic "voce"
        self.vettorevoce = data.data 
        self.timestamp1 = datetime.datetime.now()

        self.voce_ricevuto = True
    
    def callback_gesto(self,data):
    
        # Funzione di callback per il topic "gesto"
    
        self.vettoregesto = data.data 
        self.timestamp2 = datetime.datetime.now()

        self.gesto_ricevuto = True
    
    def classify(self,input): 
    
        inputtensor = torch.tensor(input)
        output = self.model(inputtensor)
        prediction = output.argmax(dim=0)
        return prediction
        
         
    
    def combina_input(self):
    
        #Time manager  
        if self.timestamp1 is not None and self.timestamp2 is not None:

             differenza = self.timestamp2 - self.timestamp1
             differenza_in_secondi = differenza.seconds
             differenza_in_microsecondi = differenza.microseconds
             differenza_tot = differenza_in_secondi + differenza_in_microsecondi/1000000
    
             if self.gesto_ricevuto and self.voce_ricevuto:
                 if differenza_tot < self.t:
         
                     combinazione =  self.vettoregesto + self.vettorevoce 
                     print("vettori sommati:", combinazione)
         
                     prediction = self.classify(combinazione)
                     print("Numero del comando:", prediction.item())
         
                     self.gesto_ricevuto = False
                     self.voce_ricevuto = False
        
        
     
if __name__ == '__main__':

    classificatore = Classificator()
    
    while not rospy.is_shutdown():
        classificatore.combina_input()