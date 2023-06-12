#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32MultiArray


class FakeVoice():

    def __init__(self):
        rospy.init_node('voice_fake', anonymous=True)
        self.pub = rospy.Publisher('gesture', Int32MultiArray, queue_size=1)
        self.msg = Int32MultiArray()
    def sendMessage(self):

        command = input("\n - Premi 1 per il solo point at \n - Premi 2 per il comando misto \n - Premi 3 per il Pause \n - Premi 4 per lo Stop \n - Premi 5 per Il comando dubbio \n Digita il comando che vuoi inviare: ")

        if command == '1': #Point at

            for i in range(5):
                self.msg.data = [1]
                self.pub.publish(self.msg)
                rospy.sleep(0.1) 
        
        elif command == '2': #Point at sporco

            for i in range(5):
                self.msg.data = [1]
                self.pub.publish(self.msg)
                rospy.sleep(0.1) 
            for i in range(2):
                
                self.msg.data = [2]
                self.pub.publish(self.msg)
                rospy.sleep(0.1)

            for i in range(3):

                self.msg.data = [1]
                self.pub.publish(self.msg)
                rospy.sleep(0.1)

        elif command == '3': #Pause

            for i in range(8):
                self.msg.data = [3]
                self.pub.publish(self.msg)
                rospy.sleep(0.1) 
            
        elif command == "4": #Stop
              for i in range(10):
                
                self.msg.data = [2]
                self.pub.publish(self.msg)
                rospy.sleep(0.1)

        elif command == "5": #Comando dubbio

            for i in range(5):
                
                self.msg.data = [1]
                self.pub.publish(self.msg)
                rospy.sleep(0.1)

            for i in range(5):

                self.msg.data = [2]
                self.pub.publish(self.msg)
                rospy.sleep(0.1)

        else:
            print("Comando non valido")
   


        
if __name__ == '__main__':
    try:
        F = FakeVoice()
        while not rospy.is_shutdown():
            F.sendMessage()
    except rospy.ROSInterruptException:
        pass
