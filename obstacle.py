import pygame
import numpy as np
from defines import limiar_rep

class Obstacle:
    def __init__(self, x, y, r):
        # self.x = x
        # self.y = y
        self.position = np.array([x, y])
        self.r = r
        
    def draw(self, screen):
        pygame.draw.circle(screen, "blue", self.position, self.r)
        pygame.draw.circle(screen, "yellow", self.position, self.r + limiar_rep,2)
        
