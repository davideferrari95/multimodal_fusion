#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32MultiArray


class FakeArea():

    def __init__(self):
        rospy.init_node('area_fake', anonymous=True)
        self.pub = rospy.Publisher('area', Int32MultiArray, queue_size=1)
        self.msg = Int32MultiArray()

    def sendMessage(self):

        command = input(" - Premi 1 per l'area destra \n - Premi 2 per l'area sinistra  \n\n Digita il comando che vuoi inviare: ")

        if command == '1':
            
            self.msg.data = [1]
            self.pub.publish(self.msg) 
        
        elif command == '2':

            self.msg.data = [2]
            self.pub.publish(self.msg)




          



if __name__ == '__main__':
    try:
        F = FakeArea()
        while not rospy.is_shutdown():
            F.sendMessage()
    except rospy.ROSInterruptException:
        pass
