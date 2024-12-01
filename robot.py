import numpy as np
import pygame
import math
from defines import *
#Polígono de centro X e vértices ABCD
#  ___
# A   B
# | X |
# D___C
def imp_dt(main_dt):
    dt = main_dt

class Robot:
    # r=10
    def __init__(self, x, y, z=0, theta = 0): #theta em relação ao X da origem
        self.size = 10
        # self.x = x
        # self.y = y
        self.position = np.array([x, y, theta])
        self.theta = theta              #theta em relação à origem
        self.front = self.position[0:2] + [(self.size/2)*math.cos(self.theta),(self.size/2)*math.sin(self.theta)]
        self.vertices = np.empty((4,2))
        #Usa séries para multiplicar as coordenadas por (-1,1,1,-1) e (1,1,-1,-1)
        for i in range(4):
            self.vertices[i,0] = ((-1)**((i**2 + i + 2)//2)) * (self.size/2) # X
            self.vertices[i,1] = (-1)**(i // 2) * (self.size/2)              # Y
        self.r = escala*(0.5 * (195/1000))
        self.l = escala*(0.5 * (381/1000))
        self.l = 0.5 * (381./1000)
        self.wl = np.deg2rad(10)
        self.wr = np.deg2rad(10)
        self.v = (self.r/2) * (self.wr + self.wl)#velocidade linear
        self.vmax = 100
        self.wmax = math.radians(60)
        self.w = (self.r/(2*self.l)) * (self.wl - self.wr) #velocidade angular.pygame considera y+ para baixo, então a diferença das w das rodas foram invertidas
        # self.v = self.r/2 * (self.wr + self.wl) #velocidade linear
        # self.w = self.r/(2*self.l) * (self.wr - self.wl) #velocidade angular
        self.corpo = np.array([[100   , -190.5],      
                  [227.5 , -50   ],
                  [227.5 , 50    ],
                  [100   , 190.5 ],
                  [-200  , 190.5 ],
                  [-227.5, 163   ],
                  [-227.5, -163  ],
                  [-200  , -190.5]])*(escala/1000)
        self.corpo = np.hstack((self.corpo, np.ones((self.corpo.shape[0], 1)))).T
        # print(corpo)
        self.rodaE = np.array([[ 97.5, 170.5],
                        [ 97.5, 210.5],
                        [-97.5, 210.5],
                        [-97.5, 170.5]])*(escala/1000)
        self.rodaE = np.hstack((self.rodaE, np.ones((self.rodaE.shape[0], 1)))).T

        self.rodaD = np.array([[ 97.5, -170.5],
                        [ 97.5, -210.5],
                        [-97.5, -210.5],
                        [-97.5, -170.5]])*(escala/1000)
        self.rodaD = np.hstack((self.rodaD, np.ones((self.rodaD.shape[0], 1)))).T
        
        self.RoboC = self.corpo
        self.RoboE = self.rodaE
        self.RoboD = self.rodaD


    def draw(self, screen, force = 0):
        
        # pygame.draw.polygon(screen, "red", self.vertices + self.position[0:2])
        # pygame.draw.line(screen,"green",(self.position[0:2]),self.front)
        pygame.draw.line(screen,"red",self.position[0:2],self.position[0:2] + force)
        pygame.draw.polygon(screen, "cyan", self.RoboC[:2, :].T)
        pygame.draw.polygon(screen, "blue", self.RoboE[:2, :].T)
        pygame.draw.polygon(screen, "blue", self.RoboD[:2, :].T)
        pygame.draw.circle(screen, "black", (self.position[0], self.position[1]),5)
        
    def move_player(self,goal,obstacles, players,dt, katt=10, max_speed=10):
        force = att_force(self.position[0:2], goal.position, katt) + rep_force_total(self.position[0:2],obstacles)+ rep_force_total(self.position[0:2],players) + rep_force_goal(self.position[0:2],goal) # Força total no ponto
        # force_limited = np.clip(force, -max_speed, max_speed) # Limita a velocidade do player
        # print(force_limited)
        # theta_ant = self.theta
        # # deltaTheta = self.w * dt
        # deltaTheta = math.atan2(force_limited[1],force_limited[0]) - theta_ant
        # self.theta = theta_ant + deltaTheta
        # R = np.array([[math.cos(deltaTheta), math.sin(deltaTheta)],
        #          [-math.sin(deltaTheta), math.cos(deltaTheta)]])
        # self.vertices = self.vertices@R
        # new_pos = self.position[0:2] + force_limited * dt
        # # new_pos = ht_matrix(self.position,deltaTheta,force_limited[0], force_limited[1])
        # # vertices = ht_matrix(self.vertices.T,deltaTheta,force_limited[0], force_limited[1])
        # #print(new_pos)
        # self.position[0:2] = new_pos[0:2]
        # self.front = new_pos[0:2] + [(self.size/2)*math.cos(self.theta),(self.size/2)*math.sin(self.theta)]
        # # for i in range(4):
        # #     self.vertices[i, 0] = self.position[0] + ((-1)**((i**2 + i + 2)//2)) * (self.size/2)
        # #     self.vertices[i, 1] = self.position[1] + (-1)**(i // 2) * (self.size/2)
        
        # self.v = (self.r/2) * (self.wr + self.wl)
        self.v = min(np.linalg.norm(force), self.vmax)
        # self.wl = ((self.v + self.wr)/self.r)
        # self.wr = ((self.v - self.wl)/self.r)
        # print(v)
        # self.w = (self.r/(2*self.l)) * (self.wl - self.wr)
        self.th = ajustaangulo(self.position[2])
        self.w = ajustaangulo(math.atan2(force[1], force[0])-self.th)
        self.w = np.sign(self.w)* min(abs(self.w),self.wmax)
        # print(th)
        # deltaS = self.v*dt
        # deltath = self.w*dt
        dPdt = np.array([[self.v * math.cos(self.th)],
            [self.v * math.sin(self.th)],
            [self.w]])

            
        # P = P + deltaP
        self.position = self.position + dPdt.flatten() * dt
        self.position[2] = ajustaangulo(self.position[2])
        # deltaP = np.array([deltaS*math.cos(self.th + deltath/2),
        #                deltaS*math.sin(self.th + deltath/2),
        #                deltath])
        
        # # dP = np.array([[v * math.cos(th)],
        # #             [v * math.sin(th)],
        # #             [w]])
        
            
        # self.position = self.position + deltaP
        # P = P + dP*dt
        
        self.RoboC = T2D(Rz2D(self.corpo, self.th), self.position[0], self.position[1])
        self.RoboE = T2D(Rz2D(self.rodaE, self.th), self.position[0], self.position[1])
        self.RoboD = T2D(Rz2D(self.rodaD, self.th), self.position[0], self.position[1])
        
        # print(RoboC.shape)
        # print(RoboD.shape)
        # print(RoboE.shape)
        # flip() the display to put your work on screen
        # print(RoboD[:2,:])
        # pygame.draw.polygon(screen, "red",RoboC[:2,:].T)
        # pygame.draw.polygon(screen, "red",RoboE[:2,:].T)
        # pygame.draw.polygon(screen, "red",RoboD[:2,:].T)
        # RoboC_xy = RoboC[:2, :]
        # RoboE_xy = RoboE[:2, :]
        # RoboD_xy = RoboD[:2, :]
        # print(RoboD_xy)
        return force
    # Vetor de atração para o goal, retorna um vetor unitário
def att_force(q, goal, katt=10):
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
        if(obs.r<10):
            obs.r = 10
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


