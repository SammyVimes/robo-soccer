#!/usr/bin/env python
# {{{ Imports
import pygame, sys
import general
import random
import rospy
import json
from pygame.locals import *
from creature import *
from field import *
import Data
import ai
from Data import SAIState
from std_msgs.msg import String
from robosoccer.srv import *
import json_tricks


def on_ai_msg(d):
    players = d['players']
    goal1 = d['goal1']
    goal2 = d['goals2']
    field = d['field']
    counter = d['counter']
    for player in players:
        player.collidingGoal(goal1)
        player.collidingGoal(goal2)
        player.move(general.surface, field, counter)


if not pygame.font:  print('Note: fonts disabled')

if not pygame.mixer: print('Note: sound disabled')

class Game:

    def move_player(self, move_player_request):
        print(move_player_request)

    def shoot_ball(self, shoot_ball_request):
        print(shoot_ball_request)

    def start(self, rootNode1_acc, rootNode2_acc, showing=False, time_length=None, playing=False, online=False):
        """ Run the simulation (e.g., game) with all the creatures (e.g., players and balls). """
        number_of_balls = 1
        number_of_players = 10
        number_of_skulls = 0
        rospy.init_node('game_server')
        start_pub = rospy.Publisher('start_pub', String, queue_size=20)
        ai_listener = rospy.Subscriber('ai_pub', String, on_ai_msg)
        shoot_service = rospy.Service('shoot_service', ShootBall, self.shoot_ball)
        move_service = rospy.Service('move_service', MovePlayer, self.move_player)

        ai_pubs = []
        for idx in range(1, number_of_players):
            ai_pubs.append(rospy.Publisher('ai_request' + str(idx), String, queue_size=20))

        balls = list()
        players = list()
        skulls = list()

        if online: socket, isClient = tcp_connect(ip_address)

        if not playing:
            rootNode1_accx = rootNode1_acc[0]
            rootNode1_accy = rootNode1_acc[1]
            rootNode2_accx = rootNode2_acc[0]
            rootNode2_accy = rootNode2_acc[1]

        if showing: pygame.init()
        if showing: os.environ['SDL_VIDEO_CENTERED'] = '1'
        # if showing: pygame.mouse.set_visible(False)

        if showing:
            vis = general.getVisibleSize()
            window = pygame.display.set_mode((int(vis[0]), int(vis[1])), pygame.DOUBLEBUF)  # , pygame.FULLSCREEN)
        else:
            window = pygame.display.set_mode((100, 100))

        fitness1 = 0
        fitness2 = 0

        if showing:
            pygame.display.set_caption('Genetic Soccer')
            general.surface = pygame.display.get_surface()
            field = Field()
            goal1 = field.getGoal1()
            goal2 = field.getGoal2()

        goals = {"goal1": json.dumps(Data.Rect(goal1.rect).__dict__), "goal2": json.dumps(Data.Rect(goal2.rect).__dict__)}

        rospy.sleep(1)

        start_pub.publish(json.dumps(goals))

        for i in range(0, number_of_balls):
            balls.append(Ball(surface=general.surface, radius=6, showing=showing))

        for player_team in range(0, 2):
            for player_role in range(0, number_of_players // 2):

                player_pos = [0, 0]

                if player_role == 0:
                    player_pos = [width / 4.0, height / 6.0 * (1.0 if player_team == 0 else 5)]
                elif player_role == 1:
                    player_pos = [width / 2.0, height / 6.0 * (1.0 if player_team == 0 else 5)]
                elif player_role == 2:
                    player_pos = [width / 2.0, height / 3.0 * (1.0 if player_team == 0 else 2)]
                elif player_role == 3:
                    player_pos = [(width * 3.0) / 4.0, height / 6.0 * (1.0 if player_team == 0 else 5)]
                elif player_role == 4:
                    player_pos = [width / 2.0, height / 7 * (1.0 if player_team == 0 else 6)]

                players.append(Player(general.surface, (20, 20), player_pos, player_team + 1, showing))

        for i in range(0, number_of_skulls):
            skulls.append(CreatureBodyComputer(surface=general.surface))

        ais = []
        for idx in range(len(players) - 1):
            i = idx + 1
            _ai = None
            if i == 4 or i == 9:
                _ai = ai.GoalkeeperAi(players[i], goal1, goal2)
            else:
                _ai = ai.Ai(players[i], goal1, goal2)
            ais.append(_ai)

        if showing: clock = pygame.time.Clock()

        counter = 0

        while time_length is None or counter < time_length:
            """ Main loop of the simulation (e.g., game). """
            moveCamera(balls[0])
            stoppingx_event = False
            stoppingy_event = False
            new_key = False
            mouse_pos = pygame.mouse.get_pos()
            goal_size = goal1.image.get_size()

            if online:
                ball = balls[0]
                player1 = players[0]
                player2 = players[1]
                terminals0 = Terminals(counter, ball.pos, ball.vel, player1.pos, player1.vel, 1, [0, 0], [0, 0], 1)
                terminals1 = Terminals(counter, ball.pos, ball.vel, [width - player2.pos[0], height - player2.pos[1]],
                                       [-player2.vel[0], -player2.vel[1]], 1, [0, 0], [0, 0],
                                       1)  # width-players[1].pos[0],height-players[1].pos[1],    players[1].vel[0], players[1].vel[1],  1)

            if not playing:
                players[0].acc[0] = running_tree(terminals0).run_tree(rootNode1_accx)
                players[0].acc[1] = running_tree(terminals0).run_tree(rootNode1_accy)

                players[1].acc[0] = running_tree(terminals1).run_tree(rootNode2_accx)
                players[1].acc[1] = -running_tree(terminals1).run_tree(rootNode2_accy)

            if showing: clock.tick(120)

            counter += 1

            if showing:
                """ Background """
                if showing: field.blitBackground(general.surface)
                """ Keyboard events """
                input = pygame.event.get()
                for event in input:
                    whatkey = Keyboard(event)
                    if whatkey.isKEYDOWN():
                        if whatkey.isEscape():
                            sys.exit(0)
                        elif whatkey.isEquals():
                            general.global_zoom *= 1.1
                        elif whatkey.isMinus():
                            general.global_zoom /= 1.1
                        elif whatkey.isLeft():
                            players[0].fireLeft()
                        elif whatkey.isRight():
                            players[0].fireRight()
                        elif whatkey.isUp():
                            players[0].fireUp()
                        elif whatkey.isDown():
                            players[0].fireDown()
                        elif whatkey.isPeriod():
                            balls[0].getShooted(players[0])
                        elif whatkey.isSlash():
                            balls[0].getShooted(players[0], 0.46)
                        elif whatkey.isA():
                            players[1].fireLeft()
                        elif whatkey.isD():
                            players[1].fireRight()
                        elif whatkey.isW():
                            players[1].fireUp()
                        elif whatkey.isS():
                            players[1].fireDown()
                        elif whatkey.isBackquote():
                            balls[0].getShooted(players[1])
                        elif whatkey.isOne():
                            balls[0].getShooted(players[1], 0.46)
                    elif whatkey.isKEYUP():
                        if whatkey.isLeft() or whatkey.isRight():
                            players[0].stopLeftAndRight()
                        elif whatkey.isUp() or whatkey.isDown():
                            players[0].stopUpAndDown()
                        elif whatkey.isA() or whatkey.isD():
                            players[1].stopLeftAndRight()
                        elif whatkey.isW() or whatkey.isS():
                            players[1].stopUpAndDown()
                    if event.type == pygame.MOUSEBUTTONDOWN:
                        left, top = (x - 50 for x in event.pos)

                    if whatkey.isKEYUP() or whatkey.isKEYDOWN():
                        new_key = True
                if online: net.updatePlayers(new_key, isClient, socket, players)

            nearest_player = players[0]
            nearest_distance = general.distance(nearest_player.get_centre_pos(), balls[0].get_centre_pos())

            for player in players[1:]:
                _distance = general.distance(player.get_centre_pos(), balls[0].get_centre_pos())
                if _distance < nearest_distance:
                    nearest_player = player
                    nearest_distance = _distance

            p_coords = []
            for player in players:
                type = player.type
                coords = player.get_pos()
                p_coords.append((type, coords[0], coords[1], Data.Rect(player.rect).__dict__))

            for (idx, _ai) in enumerate(ais):
                ball_coord = balls[0].get_pos()
                ai_state = SAIState((ball_coord[0], ball_coord[1]), p_coords)
                jhi_est = json.dumps(ai_state.__dict__)
                ai_pubs[idx].publish(jhi_est)
                # _ai.do(ai_state)

            for player in players:
                player.collidingGoal(goal1)
                player.collidingGoal(goal2)
                player.move(general.surface, field, counter, cameraPos=cameraPos, goal_size=goal_size)

            for ball in balls:
                """ All balls """
                ball_centre = ball.getCentre()

                # fitness1 += 1000/distance(ball_centre, players[0].get_pos())
                # fitness2 += 1000/distance(ball_centre, players[1].get_pos())

                ball.is_under_player = False
                # players colliding with the ball
                for player in players:
                    ball.getKicked(player)
                # computer player colliding with the ball and the goal
                for cc in skulls:
                    cc.collidingGoal(goal1)
                    cc.collidingGoal(goal2)
                    ball.getKicked(cc)
                    if debug:
                        print(mouse_pos)
                    cc.acc = [(random.random() - 0.5) / 5.0, (random.random() - 0.5) / 5.0]
                    cc.move(general.surface, field, counter, cameraPos=cameraPos, goal_size=goal_size)

                # to the pipe
                is_colliding_goal_pipe_top_right = (
                    ball.vel[1] < 0 and (diff(ball_centre[1], goal_size[1]) < ball.radius) and (
                        diff(ball_centre[0], (width / 2 + goal_size[0] / 2)) < 3))
                is_colliding_goal_pipe_top_left = (
                    ball.vel[1] < 0 and (diff(ball_centre[1], goal_size[1]) < ball.radius) and (
                        diff(ball_centre[0], (width / 2 - goal_size[0] / 2)) < 3))
                is_colliding_goal_pipe_bottom_right = (
                    ball.vel[1] > 0 and (diff(ball_centre[1], (height - goal_size[1])) < ball.radius) and (
                        diff(ball_centre[0], (width / 2 + goal_size[0] / 2)) < 3))
                is_colliding_goal_pipe_bottom_left = (
                    ball.vel[1] > 0 and (diff(ball_centre[1], (height - goal_size[1])) < ball.radius) and (
                        diff(ball_centre[0], (width / 2 - goal_size[0] / 2)) < 3))

                if is_colliding_goal_pipe_top_right or is_colliding_goal_pipe_top_left or is_colliding_goal_pipe_bottom_right or is_colliding_goal_pipe_bottom_left:
                    ball.vel[1] = - ball.vel[1]
                else:
                    ball.collidingGoal(goal1)
                    ball.collidingGoal(goal2)

                if goal1.rect.contains(ball.rect):
                    fitness2 += 100
                    field.score2 += 1
                    ball.reset()
                    players[0].reset()
                    players[1].reset()
                    players[0].fire_acc += 0.01
                    players[1].fire_acc += 0.01
                    if showing: field.setMessage2()
                elif goal2.rect.contains(ball.rect):
                    fitness1 += 100
                    field.score1 += 1
                    ball.reset()
                    players[0].reset()
                    players[1].reset()
                    players[0].fire_acc += 0.01
                    players[1].fire_acc += 0.01
                    if showing: field.setMessage1()

                ball.move(counter, cameraPos=cameraPos)
                if showing:
                    field.blitField(width=width, height=height, surface=general.surface, cameraPos=cameraPos)

            vis = general.getVisibleSize()
            if debug: pygame.draw.rect(general.surface, pygame.Color(0, 0, 0), (0, 0, vis[0], vis[1]), 1)
            if showing: pygame.display.flip()
        return (1.0 * fitness1 / counter, 1.0 * fitness2 / counter)

