import pygame

class Obstacle:
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r
        
    def draw(self, screen):
        pygame.draw.circle(screen, "blue", (self.x, self.y), self.r)
