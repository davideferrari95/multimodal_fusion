#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32MultiArray
import torch 
from Rete_neurale import LitNeuralNet

"""def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data) """
    

class Classificatore():

    voce_ricevuto = False
    gesto_ricevuto = False
    t = 2 

    def __init__(self):

        # Inizializzazione del nodo
        rospy.init_node('classificatore', anonymous=True)

        # Inizializzazione dei topic
        rospy.Subscriber("voce", Float32MultiArray, self.callback_voce)
        rospy.Subscriber("gesto", Float32MultiArray, self.callback_gesto)


    def callback_voce(self, data):

        # Funzione di callback per il topic "voce"
        self.vettorevoce = data.data 
        #print("Dati ricevuti dal topic voce:", vettorevoce)
        # self.combina_input(self.vettoregesto, self.vettorevoce)
        self.voce_ricevuto = True

    def callback_gesto(self, data):

        # Funzione di callback per il topic "gesto"
        self.vettoregesto = data.data 

        #print("Dati ricevuti dal topic gesto:", vettoregesto)
        # self.combina_input(self.vettoregesto, self.vettorevoce)
        self.gesto_ricevuto = True

    def classificatore(self, input):

        #Inizializzo la rete neurale classificatrice
        model = LitNeuralNet(0.7,0.15,0.15,6,32,64,9)

        #Carico i pesi della rete neurale
        state_dict = torch.load("/home/alberto/catkin_ws/src/fusion/scripts/Rete_allenata.pth")
        model.load_state_dict(state_dict)

        #Eseguo la predizione in modalità valutazione e non allenamento 
        model.eval()
        
        output = model(input)
        return output
        

    def combina_input(self):

        if self.gesto_ricevuto and self.voce_ricevuto:

            combinazione =  self.vettoregesto + self.vettorevoce

            print("vettori sommati:", combinazione)

            tensore_di_combinazione = torch.tensor(combinazione)
            
            output = self.classificatore(tensore_di_combinazione)

            prediction = output.argmax(dim=0)

            print("Numero del comando:", prediction.item()) 

            self.gesto_ricevuto = False
            self.voce_ricevuto = False


if __name__ == '__main__':
    
    classificatore = Classificatore()

    while not rospy.is_shutdown():
        classificatore.combina_input()
