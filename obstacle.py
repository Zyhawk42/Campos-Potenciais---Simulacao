import pygame
import numpy as np

class Obstacle:
    def __init__(self, x, y, r):
        # self.x = x
        # self.y = y
        self.position = np.array([x, y])
        self.r = r
        
    def draw(self, screen):
        pygame.draw.circle(screen, "blue", self.position, self.r)
