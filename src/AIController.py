#!/usr/bin/env python
import rospy
import sys
from random import randint
from time import sleep
import ai
from std_msgs.msg import String
from robosoccer.srv import *

class AIController:

    def __init__(self, quantity):
        self.quantity = quantity
        start_listener = rospy.Subscriber('start_pub', String, self.start)
        srv = rospy.Service('ai_service', AI, self.update)
        self.ais = []

    def start(self, start_data):
        # type: (str) -> None
        players = start_data['players']
        goal1 = start_data['goal1']
        goal2 = start_data['goal2']
        for idx in range(len(self.quantity) - 1):
            i = idx + 1
            _ai = None
            if i == 4 or i == 9:
                _ai = ai.GoalkeeperAi(players[i], goal1, goal2)
            else:
                _ai = ai.Ai(players[i], goal1, goal2)
            self.ais.append(_ai)

    def update(self, req):
        # type: (AIRequest) -> str
        res = ""
        for _ai in self.ais:
            ai_state = ai.State(self.ais[0], _ai.player, balls[0])
            ai_state.update()
            res += _ai.do(ai_state)
        return res

if __name__ == '__main__':
    try:
        rospy.init_node('ai')
        quantity = rospy.get_param('~quantity')
        aiController = AIController(quantity)
    except rospy.ROSInterruptException:
        pass