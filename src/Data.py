import pygame

class SAIState(object):

    def __init__(self, goal1, goal2, ball_coords, players_coords):
        self.ball_coords = ball_coords
        self.players_coords = players_coords
        self.goal1 = goal1
        self.goal2 = goal2


class Rect(object):

    def __init__(self, rect):
        """
        :type rect: pygame.Rect
        :param rect: zzz
        """
        self.left = rect.left
        self.top = rect.top
        self.width = rect.width
        self.height = rect.height