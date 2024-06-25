import numpy as np
import pygame
import forces
#Polígono de centro X e vértices ABCD
#  ___
# A   B
# | X |
# D___C
dt = 0
def imp_dt(main_dt):
    dt = main_dt

class Robot:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.Xa = self.x-5
        self.Ya = self.y+5
        self.Xb = self.x+5
        self.Yb = self.y+5
        self.Xc = self.x-5
        self.Yc = self.y-5
        self.Xd = self.x+5
        self.Yd = self.y-5

    def draw(self, screen):
        pygame.draw.polygon(screen, "red", [(self.Xa,self.Ya),(self.Xb,self.Yb),(self.Xd,self.Yd),(self.Xc,self.Yc)])
        pygame.draw.line(screen,"green",(self.Xb,self.Yb), (self.Xd, self.Yd))

    def move_player(self, player_pos, goal,obstacles, players,dt, katt=10, max_speed=10):
        force = forces.att_force(player_pos[0:2], goal, katt) + forces.rep_force_total(player_pos[0:2],obstacles)+ forces.rep_force_total(player_pos[0:2],players) # Força total no ponto
        force_limited = np.clip(force, -max_speed, max_speed) # Limita a velocidade do player
        new_pos = player_pos[0:2] + force_limited * dt
        print(new_pos)
        self.x = new_pos[0]
        self.y = new_pos[1]
        self.Xa = self.x-5
        self.Ya = self.y+5
        self.Xb = self.x+5
        self.Yb = self.y+5
        self.Xc = self.x-5
        self.Yc = self.y-5
        self.Xd = self.x+5
        self.Yd = self.y-5

    r = .5 * 10
    l = .5 * 10
    wl = np.deg2rad(30)
    wr = np.deg2rad(30)
    v = r/2 * (wr + wl)
    w = r/(2*l) * (wr - wl) 



