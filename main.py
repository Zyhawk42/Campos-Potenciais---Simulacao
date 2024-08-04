import pygame
import random
import numpy as np
import math
from obstacle import Obstacle
from robot import Robot
from goal import Goal

# pygame setup
pygame.init()
screen_size = (1280,720)
screen = pygame.display.set_mode(screen_size)
clock = pygame.time.Clock()
running = True
dt = 0
i=0
# goal = np.array([1100, 600])

import numpy as np

goals_data = np.array([
    #x, y
    [1100,600],
    [100,200]
])

goals = []
for goal in goals_data:
    x,y = goal
    goal = Goal(x,y)
    goals.append(goal)

obstacles_data = np.array([
    #x, y, r
    [400,350,60],  
    [600,300,50], 
    [500,400,60],
    [200,700,30],
    [1000,300,50],
    [1200,400,30],
    [750,600,30] 
])

obstacles = []
for obs in obstacles_data:
    x,y,r = obs
    obs = Obstacle(x,y,r)
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
    player = Robot(x,y)
    players.append(player)

#print(players)

# XX, YY = np.meshgrid(np.arange(0, screen_size[0]+.4, .4), np.arange(0, screen_size[1]+.4, .4))
# XY = np.dstack([XX, YY]).reshape(-1, 2)
# Fatt = forces.att_force(XY, goals[i].position)
# Frep_obs = forces.rep_force_total(XY,obstacles)
sim_running = False

# Renderiza o primeiro quadro
screen.fill("black")
pygame.draw.circle(screen, "green", goals[0].position, 10)
for obs in obstacles:
    obs.draw(screen)
for player in players:
    player.draw(screen)
goals[0].draw(screen)

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
        player_positions = np.array([player.position[0:2] for player in players])
        #pygame.draw.circle(screen, "green", goal, 10)
        #print(player.position)
        goals[i].draw(screen)
        for obs in obstacles:
            obs.draw(screen)
        for player in players:
            player.draw(screen)
            player.move_player(goals[i],obstacles,players,dt,100,50)
        distances = np.linalg.norm(player_positions - goals[i].position, axis=1)    
        #print(distances)

        
        if all(distance <= 30 for distance in distances):
            i = (i+1)%len(goals_data)
        #print(i)
            #print(player.player_pos)
        # flip() the display to put your work on screen
        pygame.display.flip()

        # limits FPS to 60
        # dt is delta time in seconds since last frame, used for framerate-
        # independent physics.
        dt = clock.tick(60) / 1000
    else:
        dt = clock.tick(60) / 1000

pygame.quit()