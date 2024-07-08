import pygame
import numpy as np

class Goal:
    def __init__(self, x, y):
        # self.x = x
        # self.y = y
        self.position = np.array([x, y])
        
    def draw(self, screen):
        pygame.draw.circle(screen, "green", self.position, 20)
