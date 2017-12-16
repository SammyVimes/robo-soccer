#!/usr/bin/env python
import rospy
import sys
from random import randint
from time import sleep
import ai
from std_msgs.msg import String
from robosoccer.srv import *
import general
import math
from field import Goal
import json
import pygame


class AIController:

    def __init__(self, id):
        rospy.wait_for_service('move_service')
        self.move_service = rospy.ServiceProxy('move_service', MovePlayer)

        self.id = id
        self.is_goalkeeper = id == 4 or id == 9
        self.astar_size = [30,40]
        ai_req_listener = rospy.Subscriber('ai_request' + str(id), String, self.goalkeeper if self.is_goalkeeper else self.player)
        start_pub_listener = rospy.Subscriber('start_pub', String, self.on_start)
        #srv = rospy.Service('ai_service', AI, self.update)
        rospy.spin()

    def to_rect(self, rect):
        return pygame.Rect(rect["left"], rect["top"], rect["width"], rect["height"])


    def on_start(self, start):
        start = json.loads(start.data)
        goal1_rect = start["goal1"]
        goal2_rect = start["goal2"]
        self.goal1 = self.to_rect(json.loads(goal1_rect))
        self.goal2 = self.to_rect(json.loads(goal2_rect))

    def goalkeeper(self, ai_state):
        ai_state = json.loads(ai_state.data)
        pass

    def player(self, ai_state):
        ai_state = json.loads(ai_state.data)
        pls = ai_state["players_coords"]
        ball = ai_state["ball_coords"]
        this_player = pls[self.id]
        blocked_rects = [self.goal1, self.goal2, self.to_rect(this_player[3])]
        self.astar = AStar([this_player[1], this_player[2]], ball, self.astar_size, blocked_rects)

        path = self.astar.get_shortest_path()

        if len(path) >= 2:
            self.go_to(general.Array(path[1]) - [0, 0])

    def go_to(self, param):
        self.move_service(self.id, param[0], param[1])
        pass


class AStar:
    def __init__(self, start_pixel_pos, target_pixel_pos, size, blocked_rects=[]):
        ## Number of nodes horizontally and vertically:
        self.sizex, self.sizey = size[0], size[1]
        self.unit_cost = 10
        self.unit_diagonal_cost = 14
        self.blocked_poses = []

        ## Create start and target nodes:
        start_node_pos  = self.to_node_pos(start_pixel_pos)
        target_node_pos = self.to_node_pos(target_pixel_pos)
        self.target_node = Node(pos=target_node_pos)
        self.start_node  = Node(pos=start_node_pos, g=0, h=self.calc_h(start_node_pos))

        #self.create_all_nodes() #[None] * (self.sizex * self.sizey)
        self.openlist   = []
        self.all_nodes = {}

        self.rects_to_poses(blocked_rects)
        if not (self.start_node.pos in self.blocked_poses):
            self.openlist.append(self.start_node)
        self.closedlist = []

    def rects_to_poses(self, rects):
        for rect in rects:
            topleft     = self.to_node_pos(rect.topleft)
            bottomright = self.to_node_pos(rect.bottomright)
            width  = int(abs(topleft[0] - bottomright[0]))
            height = int(abs(topleft[1] - bottomright[1]))
            for y in range(0, height+1):
                for x in range(0, width+1):
                    self.blocked_poses.append([topleft[0]+x,topleft[1]+y])

    def to_node_pos(self, pixel_pos):
        return [math.floor(self.sizex * (pixel_pos[0] / general.width )),
                math.floor(self.sizey * (pixel_pos[1] / general.height))]

    def to_pixel_pos(self, node_pos):
        return [1.0 * general.width  * node_pos[0] / self.sizex + 1.0 * (general.width/self.sizex)/2.0,
                1.0 * general.height * node_pos[1] / self.sizey + 1.0 * (general.height/self.sizey)/2.0]

    def draw_blocks(self):
        for node in self.blocked_poses:
            x = (1.0 * node[0]/self.sizex) * general.width  + 1.0 * general.width /self.sizex/2.0
            y = (1.0 * node[1]/self.sizey) * general.height + 1.0 * general.height/self.sizey/2.0
            general.drawCircle(general.surface, pygame.Color(0,0,0), [x,y], 5,0)

    def draw_grid(self):
        for i in range(0, self.sizex):
            x = (1.0 * i/self.sizex) * general.width
            general.drawLine(general.surface, pygame.Color(0,0,0), [x,0], [x,general.height])
        for i in range(0, self.sizey):
            y = (1.0 * i/self.sizey) * general.height
            general.drawLine(general.surface, pygame.Color(0,0,0), [0,y], [general.width,y])

    #def create_all_nodes(self):
    #    all_poses = [[x,y] for y in range(0,self.sizey) for x in range(0,self.sizex) ]
    #    self.all_nodes = [Node(pos=pos, g=0, h=self.calc_h(pos)) for pos in all_poses]

    def get_node(self, pos):
        pos_tuple = tuple(pos)
        pos_list  = list(pos)
        if not pos_tuple in self.all_nodes:
            self.all_nodes[pos_tuple] = Node(pos=pos_list, g=0, h=self.calc_h(pos_list))
        return self.all_nodes[pos_tuple]

    def get_adjacents(self, node):
        ret = []
        for y in range(int(node.pos[1]-1), int(node.pos[1]+1+1)):
            for x in range(int(node.pos[0]-1), int(node.pos[0]+1+1)):
                if not (x == node.pos[0] and y == node.pos[1]) and (x >= 0 and y >= 0) and (x < self.sizex and y < self.sizey) and not ([x,y] in self.blocked_poses):
                    adj_node = self.get_node([x,y])
                    if not (x == node.pos[0] or y == node.pos[1]):
                        adj_node.diagonal = True
                    ret.append(adj_node)
        return ret

    def get_astar_paths(self):
        path = []

        if self.target_node.pos in self.blocked_poses:
            return path
        while len(self.openlist) > 0:
            min_f = -1
            next_node = None
            for node in self.openlist:
                if node.f < min_f or min_f == -1:
                    next_node = node
                    min_f = node.f

            if next_node is None:
                print("node is None")
                return path

            self.openlist.remove(next_node)
            self.closedlist.append(next_node)

            for node in self.get_adjacents(next_node):
                if node in self.closedlist:
                    continue
                if not (node in self.openlist):
                    node.parent = next_node
                    node.g = node.parent.g
                    if node.diagonal: node.g += self.unit_diagonal_cost
                    else: node.g += self.unit_cost
                    node.f = node.g + node.h
                    self.openlist.append(node)
                else:
                    new_cost = next_node.g
                    if node.diagonal: new_cost += self.unit_diagonal_cost
                    else: new_cost += self.unit_cost
                    if node.g > new_cost:
                        node.parent = next_node
                        node.g = new_cost
                        node.f = node.g + node.h

            path.append(next_node)
            if next_node.pos == self.target_node.pos:
                return path
        return path

    def get_shortest_path(self):
        paths = self.get_astar_paths()
        if paths == []:
            return []
        node = paths.pop()
        shortest_path = []
        while node != None:
            shortest_path.append(self.to_pixel_pos(node.pos))
            node = node.parent
        shortest_path.reverse()
        return shortest_path

    def draw_path(self, path, node):
        if node == None:
            return path
        else:
            path.append(node)
            #general.drawCircle(general.surface, pygame.Color(255,0,0), self.to_pixel_pos(node.pos), 5,0)
            #self.draw_path(node.parent)

    def calc_f(self, node):
        node.f = node.g + node.h
        return node.f

    def calc_h(self, node_pos):
        h = self.unit_cost * (general.diff(node_pos[0], self.target_node.pos[0]) +\
                              general.diff(node_pos[1], self.target_node.pos[1]) )

        #xDistance = abs(node_pos[0]-self.target_node.pos[0])
        #yDistance = abs(node_pos[1]-self.target_node.pos[1])
        #if xDistance > yDistance:
        #     h = 14*yDistance + 10*(xDistance-yDistance)
        #else:
        #     h = 14*xDistance + 10*(yDistance-xDistance)

        #print node_pos, self.target_node.pos, ret
        return h

class Node(object):
    def __init__(self, pos, g=0, h=0 ):
        self.pos = pos
        self.parent = None
        self.diagonal = False
        self.h = h
        self.g = g
        self.f = self.h + self.g

if __name__ == '__main__':
    try:
        rospy.init_node('ai')
        id = rospy.get_param('~id')
        aiController = AIController(id)
    except rospy.ROSInterruptException:
        pass