import numpy as np
import pygame
import math
#Polígono de centro X e vértices ABCD
#  ___
# A   B
# | X |
# D___C
def imp_dt(main_dt):
    dt = main_dt

class Robot:
    r=10
    def __init__(self, x, y, theta = 0): #theta em relação ao X da origem
        self.size = 10
        # self.x = x
        # self.y = y
        self.position = np.array([x, y])
        self.theta = theta              #theta em relação à origem
        self.front = self.position + [(self.size/2)*math.cos(self.theta),(self.size/2)*math.sin(self.theta)]
        self.vertices = np.empty((4,2))
        #Usa séries para multiplicar as coordenadas por (-1,1,1,-1) e (1,1,-1,-1)
        for i in range(4):
            self.vertices[i, 0] = self.position[0] + ((-1)**((i**2 + i + 2)//2)) * (self.size/2) # X
            self.vertices[i, 1] = self.position[1] + (-1)**(i // 2) * (self.size/2)              # Y
        
    def draw(self, screen):
        pygame.draw.polygon(screen, "red", self.vertices)
        pygame.draw.line(screen,"green",(self.position),self.front)

    def move_player(self,goal,obstacles, players,dt, katt=10, max_speed=10):
        force = att_force(self.position, goal.position, katt) + rep_force_total(self.position,obstacles)+ rep_force_total(self.position,players) + rep_force_goal(self.position,goal) # Força total no ponto
        force_limited = np.clip(force, -max_speed, max_speed) # Limita a velocidade do player
        # print(force_limited.shape)
        self.theta = math.atan2(force_limited[1],force_limited[0])
        new_pos = self.position + force_limited * dt
        #print(new_pos)
        self.position = new_pos
        self.front = new_pos + [(self.size/2)*math.cos(self.theta),(self.size/2)*math.sin(self.theta)]
        for i in range(4):
            self.vertices[i, 0] = self.position[0] + ((-1)**((i**2 + i + 2)//2)) * (self.size/2)
            self.vertices[i, 1] = self.position[1] + (-1)**(i // 2) * (self.size/2)
        return new_pos

    # Vetor de atração para o goal, retorna um vetor unitário
def att_force(q, goal, katt=50):
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
    #obs = [obs.x, obs.y, obs.r]
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
        #print(q)
        #print(obs)
        x = obs.position[0]
        y = obs.position[1]
        obs = np.array([x,y, obs.r])
        #print(q)
        #print(obs)

        force = rep_force(q, obs)  # Força de repulsão de cada obstáculo
        total_force += force
    return total_force

def rep_force_goal(q, goal):
    total_force = np.zeros_like(q)  # inicializa o array com 0
    x = goal.position[0]
    y = goal.position[1]
    obs = np.array([x,y, 10])
    #print(q)
    #print(obs)

    force = rep_force(q, obs)  # Força de repulsão de cada obstáculo
    total_force += force
    return total_force



