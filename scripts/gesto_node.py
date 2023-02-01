#!/usr/bin/env python3
import rospy 
from std_msgs.msg import Float32MultiArray, String 


def manda_comando():

    rospy.init_node('Comando_gestuale', anonymous=True)
    pub = rospy.Publisher('gesto', Float32MultiArray, queue_size=1)

 # Loop di pubblicazione del messaggio
    while not rospy.is_shutdown():
        
        # Creazione del messaggio vuoto
        msg = Float32MultiArray()
        msg.data = [0.8,0.1,0.1] 
        pub.publish(msg) # Pubblicazione del messaggio
        rospy.sleep(1)
    

if __name__ == '__main__':
    try:
        manda_comando()
    except rospy.ROSInterruptException:
        pass