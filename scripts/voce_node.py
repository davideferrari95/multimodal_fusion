#!/usr/bin/env python3
import rospy 
from std_msgs.msg import String, Float32MultiArray


def manda_comando():

    rospy.init_node('Comando_vocale', anonymous=True)
    pub = rospy.Publisher('voce', Float32MultiArray, queue_size=10)

 # Loop di pubblicazione del messaggio
    while not rospy.is_shutdown():

        # Creazione del messaggio vuoto
        msg = Float32MultiArray()
        msg.data = [0,0,1]  # Inserimento dei dati nel messaggio
        pub.publish(msg) # Pubblicazione del messaggio
        rospy.sleep(1.05)
    

if __name__ == '__main__':
    try:
        manda_comando()
    except rospy.ROSInterruptException:
        pass