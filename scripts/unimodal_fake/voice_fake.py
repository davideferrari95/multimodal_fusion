#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32MultiArray


class FakeVoice():

    def __init__(self):
        rospy.init_node('voice_fake', anonymous=True)
        self.pub = rospy.Publisher('voice', Int32MultiArray, queue_size=1)
        self.msg = Int32MultiArray()

    def sendMessage(self):

        command = (input("Digita il codice relativo all'oggetto che vuoi prendere: "),)
        command = (int(command[0]),)

        self.msg.data = command
        self.pub.publish(self.msg)

        

if __name__ == '__main__':
    try:
        F = FakeVoice()
        while not rospy.is_shutdown():
            F.sendMessage()
    except rospy.ROSInterruptException:
        pass
