#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import datetime 



voce_ricevuto = False
gesto_ricevuto = False
t = 2 

def callback_voce(data):

    #Dichiaro la variabile globale vettorevoce
    global vettorevoce, voce_ricevuto, timestamp1

    # Funzione di callback per il topic "voce"
    vettorevoce = data.data 
    timestamp1 = datetime.datetime.now()

    #print("Dati ricevuti dal topic voce:", vettorevoce)
    combina_input(vettoregesto, vettorevoce)
    voce_ricevuto = True

def callback_gesto(data):

    #Dichiaro la variabile globale vettoregesto
    global vettoregesto, gesto_ricevuto, timestamp2

    # Funzione di callback per il topic "gesto"

    vettoregesto = data.data 
    timestamp2 = datetime.datetime.now()

    #print("Dati ricevuti dal topic gesto:", vettoregesto)
    combina_input(vettoregesto, vettorevoce)
    gesto_ricevuto = True

def combina_input(vettorevoce, vettoregesto):

    global gesto_ricevuto,voce_ricevuto, timestamp1, timestamp2
    
    #Calcolo della differenza giista dei tempi 
    differenza = timestamp2 - timestamp1
    differenza_in_secondi = differenza.seconds
    differenza_in_microsecondi = differenza.microseconds
    differenza_tot = differenza_in_secondi + differenza_in_microsecondi/1000000

    if gesto_ricevuto and voce_ricevuto:
        if differenza_tot < t:
            combinazione = vettorevoce + vettoregesto
            print("vettori sommati:", combinazione)
            gesto_ricevuto = False
            voce_ricevuto = False

def _init_():

    # Inizializzazione del nodo
    rospy.init_node('classificatore', anonymous=True)

    # Inizializzazione dei topic
    rospy.Subscriber("voce", Float32MultiArray, callback_voce)
    rospy.Subscriber("gesto", Float32MultiArray, callback_gesto)

    #Aspetta che abbia finito 
    rospy.spin()


if __name__ == '__main__':
    _init_()
    combina_input()