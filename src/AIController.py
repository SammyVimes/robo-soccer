#!/usr/bin/env python
import rospy
import sys
from random import randint
from time import sleep
import ai
from std_msgs.msg import String
from robosoccer.srv import *

class AIController:

    def __init__(self, id):
        self.id = id
        self.goalkeeper = id == 4 or id == 9
        start_listener = rospy.Subscriber('ai_request' + str(id), String, self.goalkeeper if self.goalkeeper else self.player)
        #srv = rospy.Service('ai_service', AI, self.update)

    def goalkeeper(self):
        pass

    def player(self):
        pass

if __name__ == '__main__':
    try:
        rospy.init_node('ai')
        id = rospy.get_param('~id')
        aiController = AIController(id)
    except rospy.ROSInterruptException:
        pass