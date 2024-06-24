import pygame
import random
import numpy as np
import math
import Obstacle
import Robot

# pygame setup
pygame.init()
screen_size = (1280,720)
screen = pygame.display.set_mode(screen_size)
clock = pygame.time.Clock()
running = True
dt = 0
player1 = np.array([100.0,100.0])
goal = np.array([1100, 600])

import numpy as np

# Vetor de atração para o goal, retorna um vetor unitário
def att_force(q, goal, katt=50):
    if np.all(goal-q <=10): return np.array([0,0]) 
    #print(katt*((goal - q)/(np.linalg.norm(goal - q))))
    return katt*((goal - q)/(np.linalg.norm(goal - q)))


# Vetor de repulsão do obstáculo 
# def rep_force(q, obs):

#     R=obs[2] + 5
#     dist = np.linalg.norm(q - obs[0:2])
#     if dist <=0.001: dist=0.001 # Evitar divisão por 0
#     force = (R / dist) ** 3 * (q - obs[0:2])

#     return force *1.3

def rep_force(q, obs, R=30):
    # Obstáculo: (x, y, r)
    # v: distância vetorial
    # d: módulo da distância
    #print(q)
    #print(obs)
    v = q - obs[0:2]
    if np.all(v == 0):
        return 0
    if len(obs)<3:
        d = np.linalg.norm(v) - 10
    else:
        d = np.linalg.norm(v) - obs[2]
    
    rep = (1/d**2)*((1/d)-(1/R))*(v/d)    
    
    invalid = np.squeeze(d > R)
    rep[invalid, :] = 0
    #print(rep)
    return 100000*rep

def rep_force_total(q, obstacles):
    total_force = np.zeros_like(q)  # inicializa o array com 0
    for obs in obstacles:
        force = rep_force(q, obs)  # Força de repulsão de cada obstáculo
        total_force += force
    return total_force


def move_player(player_pos, goal, katt=10, max_speed=10):
    force = att_force(player_pos[0:2], goal, katt) + rep_force_total(player_pos[0:2],obstacles)+ rep_force_total(player_pos,players) # Força total no ponto
    force_limited = np.clip(force, -max_speed, max_speed) # Limita a velocidade do player
    new_pos = player_pos + force_limited * dt
    return new_pos

obstacles_data = np.array([
    [400,350,60],  
    [600,300,50], 
    [500,400,60],
    [750,600,30] 
])

obstacles = []
for obs in obstacles_data:
    obstacles.append((obs[0], obs[1], obs[2]))

players_data = np.array([
    [100.,100.,0.],
    [130.,130.,0.],
    [100.,130.,0.],
    [130.,100.,0.],
]
)
players = []
for player in players_data:
    players.append((player[0], player[1], player[2]))


XX, YY = np.meshgrid(np.arange(0, screen_size[0]+.4, .4), np.arange(0, screen_size[1]+.4, .4))
XY = np.dstack([XX, YY]).reshape(-1, 2)
Fatt = att_force(XY, goal)
Frep = rep_force_total(XY,obstacles)

sim_running = False

# Renderiza o primeiro quadro
screen.fill("black")
pygame.draw.circle(screen, "green", goal, 10)
for obs in obstacles:
    pygame.draw.circle(screen, "blue", (obs[0], obs[1]), obs[2])
for i in range(len(players)):
    pygame.draw.polygon(screen, "red", [(players[i][0] -5 , players[i][1] -5),(players[i][0] -5 , players[i][1] +5),(players[i][0] +5 , players[i][1] +5), (players[i][0] +5 , players[i][1] -5)])
    pygame.draw.line(screen,"green",(players[i][0] +5 , players[i][1] +5), (players[i][0] +5 , players[i][1] -5))

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
            pygame.draw.circle(screen, "blue",(obs[0], obs[1]), obs[2])
        #pygame.draw.rect(screen, "blue", (400,350,100,200))
        for i in range(len(players)):
            pygame.draw.polygon(screen, "red", [(players[i][0] -5 , players[i][1] -5),(players[i][0] -5 , players[i][1] +5),(players[i][0] +5 , players[i][1] +5), (players[i][0] +5 , players[i][1] -5)])
            pygame.draw.line(screen,"green",(players[i][0] +5 , players[i][1] +5), (players[i][0] +5 , players[i][1] -5))
            player_pos = np.array(players[i][0:2])
            player = move_player(player_pos,goal,100,50)
            players[i] = player
            
        #pygame.draw.rect(screen, "red", (player1[0] - 5, player1[1] - 5,10,10))
        #player1 = move_player(player1,goal,100,50)
        # flip() the display to put your work on screen
        pygame.display.flip()

        # limits FPS to 60
        # dt is delta time in seconds since last frame, used for framerate-
        # independent physics.
        dt = clock.tick(60) / 1000
    else:
        dt = clock.tick(60) / 1000

pygame.quit()