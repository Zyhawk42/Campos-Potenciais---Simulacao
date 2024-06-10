# Example file showing a circle moving on screen
import pygame
import random
import numpy as np
import math

# pygame setup
pygame.init()
screen_size = (1280,720)
screen = pygame.display.set_mode(screen_size)
clock = pygame.time.Clock()
running = True
dt = 0
#cell = (pygame.Vector2,pygame.Vector2, pygame.Vector2)
#player_pos = pygame.Vector2(100,100)
#goal_pos = pygame.Vector2(1100,600)
player = np.array([100.0,100.0])
goal = np.array([1100, 600])

#Força de atração simples baseada na distância ao goal
def att_force(q, goal, katt=.1): 
    return katt*(goal - q)

XX, YY = np.meshgrid(np.arange(0, screen_size[0]+.4, .4), np.arange(0, screen_size[1]+.4, .4))
XY = np.dstack([XX, YY]).reshape(-1, 2)


while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # fill the screen with a color to wipe away anything from last frame
    screen.fill("black")
    pygame.draw.rect(screen, "red", (player[0], player[1],100,100))
    pygame.draw.circle(screen, "green", goal, 10)
    pygame.draw.circle(screen, "blue",(200,50), 40)
    pygame.draw.rect(screen, "blue", (400,350,100,200))
    Fatt = att_force(XY, goal)
    Fatt_x = Fatt[:,0]
    Fatt_y = Fatt[:,1]

    player += att_force(player,goal) * dt
    # flip() the display to put your work on screen
    pygame.display.flip()

    # limits FPS to 60
    # dt is delta time in seconds since last frame, used for framerate-
    # independent physics.
    dt = clock.tick(60) / 1000

pygame.quit()