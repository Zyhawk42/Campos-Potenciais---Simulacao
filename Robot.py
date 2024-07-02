import numpy as np
import pygame
import forces
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
        self.x = x
        self.y = y
        self.player_pos = np.array([x, y])
        self.vertices = np.empty((4,2))
        #Usa séries para multiplicar as coordenadas por (-1,1,1,-1) e (1,1,-1,-1)
        for i in range(4):
            self.vertices[i, 0] = self.player_pos[0] + ((-1)**((i**2 + i + 2)//2)) * (self.size/2) # X
            self.vertices[i, 1] = self.player_pos[1] + (-1)**(i // 2) * (self.size/2)              # Y
        
        #print(self.vertices)    

            # self.Xa = self.x-5
            # self.Ya = self.y+5
            # self.Xb = self.x+5
            # self.Yb = self.y+5
            # self.Xd = self.x+5
            # self.Yd = self.y-5
            # self.Xc = self.x-5
            # self.Yc = self.y-5


    def draw(self, screen):
        pygame.draw.polygon(screen, "red", self.vertices)
        #pygame.draw.line(screen,"green",(self.Xb,self.Yb), (self.Xd, self.Yd))

    def move_player(self,goal,obstacles, players,dt, katt=10, max_speed=10):
        #print(self.player_pos)
        force = forces.att_force(self.player_pos, goal, katt) + forces.rep_force_total(self.player_pos,obstacles)+ forces.rep_force_total(self.player_pos,players) # Força total no ponto
        force_limited = np.clip(force, -max_speed, max_speed) # Limita a velocidade do player
        new_pos = self.player_pos + force_limited * dt
        #print(new_pos)
        #self.player_pos = new_pos
        for i in range(4):
            self.vertices[i, 0] = self.player_pos[0] + ((-1)**((i**2 + i + 2)//2)) * (self.size/2)
            self.vertices[i, 1] = self.player_pos[1] + (-1)**(i // 2) * (self.size/2)
        return new_pos
        #print(self.vertices)
        # self.Xa = self.x-5
        # self.Ya = self.y+5
        # self.Xb = self.x+5
        # self.Yb = self.y+5
        # self.Xc = self.x-5
        # self.Yc = self.y-5
        # self.Xd = self.x+5
        # self.Yd = self.y-5
        # #Robot.rotaciona(self)

    #def rotaciona(self):
        #dist = math.sqrt( (5)**2 + (5)**2 )
        # self.Xc +=(dist*math.cos(math.radians(self.theta)))
        # self.Yc +=(dist*math.sin(math.radians(self.theta)))
        # self.Xd -=(dist*math.cos(math.radians(self.theta+90)))
        # self.Yd -=(dist*math.sin(math.radians(self.theta+90)))
        # self.Xa +=(dist*math.cos(math.radians(self.theta+180)))
        # self.Ya +=(dist*math.sin(math.radians(self.theta+180)))
        # self.Xb -=(dist*math.cos(math.radians(self.theta+270)))
        # self.Yb -=(dist*math.sin(math.radians(self.theta+270)))
        #self.theta += 1
        #pygame.draw.polygon(screen,(255,255,0),[(self.Xa,self.Ya),(self.Xb,self.Yb),(self.Xc,self.Yc),(self.Xd,self.Yd)])
        #print(self.Xa - self.x)



