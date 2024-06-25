import pygame
import random
import numpy as np
import math
import Obstacle
import Robot
import forces

# pygame setup
pygame.init()
screen_size = (1280,720)
screen = pygame.display.set_mode(screen_size)
clock = pygame.time.Clock()
running = True
dt = 0
goal = np.array([1100, 600])

import numpy as np

obstacles_data = np.array([
    #x, y, r
    [400,350,60],  
    [600,300,50], 
    [500,400,60],
    [750,600,30] 
])

obstacles = []
for obs in obstacles_data:
    x,y,r = obs
    obs = Obstacle.Obstacle(x,y,r)
    obstacles.append(obs)

players_data = np.array([
    # x, y
    [100.,100.],
    [130.,130.],
    [100.,130.],
    [130.,100.],
]
)
players = []
for coord in players_data:
    x,y = coord
    player = Robot.Robot(x,y)
    players.append(player)


XX, YY = np.meshgrid(np.arange(0, screen_size[0]+.4, .4), np.arange(0, screen_size[1]+.4, .4))
XY = np.dstack([XX, YY]).reshape(-1, 2)
Fatt = forces.att_force(XY, goal)
Frep_obs = forces.rep_force_total(XY,obstacles)
sim_running = False

# Renderiza o primeiro quadro
screen.fill("black")
pygame.draw.circle(screen, "green", goal, 10)
for obs in obstacles:
    Obstacle.Obstacle.draw(obs,screen)
for player in players:
    Robot.Robot.draw(player,screen)

pygame.display.flip()

while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
            sim_running = not sim_running
    # fill the screen with a color to wipe away anything from last frame
    if sim_running:
        screen.fill("black")
        pygame.draw.circle(screen, "green", goal, 10)
        for obs in obstacles:
            Obstacle.Obstacle.draw(obs,screen)    
        for player in players:
            Robot.Robot.draw(player,screen)
            player_pos = np.array([player.x, player.y])
            Robot.Robot.move_player(player,player_pos,goal,obstacles,players,dt,100,50)

        # flip() the display to put your work on screen
        pygame.display.flip()

        # limits FPS to 60
        # dt is delta time in seconds since last frame, used for framerate-
        # independent physics.
        dt = clock.tick(60) / 1000
    else:
        dt = clock.tick(60) / 1000

pygame.quit()