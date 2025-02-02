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
        self.position = np.array([x, y, ajustaangulo(theta)])        
        self.front = self.position[0:2] + [(self.size/2)*math.cos(self.position[2]),(self.size/2)*math.sin(self.position[2])]
        self.vertices = np.empty((4,2))
        #Usa séries para multiplicar as coordenadas por (-1,1,1,-1) e (1,1,-1,-1)
        # for i in range(4):
        #     self.vertices[i,0] = ((-1)**((i**2 + i + 2)//2)) * (self.size/2) # X
        #     self.vertices[i,1] = (-1)**(i // 2) * (self.size/2)              # Y
        self.r = escala*(0.5 * (195/1000))
        self.l = escala*(0.5 * (381/1000))
        self.l = 0.5 * (381./1000)
        self.wl = np.deg2rad(10)
        self.wr = np.deg2rad(10)
        self.v = (self.r/2) * (self.wr + self.wl)#velocidade linear
        self.v_old = 0
        self.vmax_abs = 100
        self.vmax_dyn = self.vmax_abs
        self.wmax = math.radians(60)
        self.w = (self.r/(2*self.l)) * (self.wl - self.wr) #velocidade angular.pygame considera y+ para baixo, então a diferença das w das rodas foram invertidas
        self.a = 0
        self.a_old = 0
        self.a_max = 20
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
        self.colisor = int(1.4*max(np.amax(self.RoboC),np.amax(self.RoboE), np.amax(self.RoboD)))
        self.colidiu = False
        # print(self.colisor)
        self.dx = 0
        self.dy = 0
        # dth = G[2] - P[2]
        # self.rho = 0
        # print(dy,dx,dth)
        self.gamma = 0 #Ângulo da posição do robô ao objetivo
        # self.alpha = 0 #Ângulo entre a frente do robô e o objetivo
        # self.krho = 1
        # self.kp = 10
        # self.ki = 0.01
        # self.kd = 1
        # self.dif_alpha = 0
        # self.int_alpha = 0
        # self.alpha_old = 0
        self.path = []
        self.conta_ciclo = 0
        self.preso = False
        self.cont_preso = 0
        self.cont_ciclo_livre = 0
        self.lado_rot_preso = 0
        self.th_old = 0
        self.temp_goal = [0,0]
        self.goal_list = []
        self.index_goal = 0

    def draw(self, screen, force = 0):
        
        # pygame.draw.polygon(screen, "red", self.vertices + self.position[0:2])
        # pygame.draw.line(screen,"green",(self.position[0:2]),self.front)
        pygame.draw.line(screen,"red",self.position[0:2],self.position[0:2] + force)
        pygame.draw.polygon(screen, "cyan", self.RoboC[:2, :].T)
        pygame.draw.polygon(screen, "blue", self.RoboE[:2, :].T)
        pygame.draw.polygon(screen, "blue", self.RoboD[:2, :].T)
        pygame.draw.circle(screen, "black", (self.position[0], self.position[1]),5)
        pygame.draw.circle(screen, "yellow", (self.position[0], self.position[1]),self.colisor,1)
        pygame.draw.circle(screen, "green", self.goal_list[self.index_goal].position, 10)
        # pygame.draw.arc(screen,"yellow",(pygame.Rect(self.position[0] - limiar_rep,self.position[1] - limiar_rep,2*limiar_rep, 2*limiar_rep)),-self.position[2]- angle_sensor,-self.position[2] + angle_sensor,1)
        # pygame.draw.line(screen, "yellow",self.position[0:2],(self.position[0] + limiar_rep * math.cos(self.position[2]- angle_sensor),self.position[1] + limiar_rep*math.sin(self.position[2]- angle_sensor)))
        # pygame.draw.line(screen, "yellow",self.position[0:2],(self.position[0] + limiar_rep * math.cos(self.position[2]+ angle_sensor),self.position[1] + limiar_rep*math.sin(self.position[2]+ angle_sensor)))
        for coord in self.path:
                pygame.draw.circle(screen, "cyan",coord,2)
        # pygame.draw.circle(screen,"pink",self.front,3)
        
    def move_player(self,goal,obstacles, players,dt):
        # print(players)
        # players = (player for player in players if player is not self)
        # print(players)
        # print(self.goal_list)
        # v1 = ((sum(player.position[0:2] for player in players)/num_robot-1) - self.position[0:2])/10
        boids_calc = boids(self, players)
        # print(boids_calc)  
        force = 10*boids_calc #+ att_force(self.position[0:2], goal.position, katt) + max(self.v, self.vmax_abs/10) * (rep_force_total(self.position,obstacles, self.colisor) + rep_force_total(self.position + [0,0,math.pi/2],obstacles, self.colisor)/10 + rep_force_total(self.position + [0,0,-math.pi/2],obstacles, self.colisor)/10) #+ rep_force_total(self.position,players, self.colisor)) #+ rep_force_goal(self.position[0:2],goal) # Força total no ponto
        # # print(self.preso, self.cont_preso,math.degrees(self.th_old), math.degrees(self.position[2]), math.degrees(abs(ajustaangulo(self.position[2])) - abs(ajustaangulo(self.th_old))))
        if self.cont_preso > limiar_preso:
            force = trat_preso(self, goal, players)
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
        # self.dx = goal.position[0] - self.position[0]
        # self.dy = goal.position[1] - self.position[1]
        # self.rho = math.sqrt(self.dx**2 + self.dy**2)
        # self.gamma = ajustaangulo(math.atan2(force[1],force[0]) - self.position[2]) #Erro entre o angulo do robo e o angulo da forca
        # self.alpha = ajustaangulo(self.gamma - self.position[2])
        # self.dif_alpha = self.alpha - self.alpha_old
        # self.alpha_old = self.alpha
        # self.int_alpha = self.int_alpha + self.alpha
        # self.a = np.clip((2*np.linalg.norm(force))/massa, -self.a_max, self.a_max)
        # self.a = self.a * math.cos(self.gamma)
        # if abs(math.atan2(force[1],force[0])) - abs(self.position[2]) > math.pi/2:
        #     self.a = -50
        # dist_obs_eff = calcula_d(self.position, obstacles)
        # # print(dist_obs_eff)
        # # print(force - att_force(self.position[0:2], goal.position, katt), len(dist_obs_eff)) 
        # # if len(dist_obs_eff) and (min(dist_obs_eff) <= self.colisor):
        # #     self.colidiu = True
        # #     sim_running = not sim_running
        # #     # print("1")
        # if len(dist_obs_eff):
        #     dist = min(dist_obs_eff)
        #     if dist <= 1.5*self.colisor:
        #         self.vmax_dyn = 0
        #         self.cont_preso += 1 
        #         self.cont_ciclo_livre = 0
        #     else:
        #         self.vmax_dyn = (self.vmax_abs/(limiar_rep/(sum(dist_obs_eff)/len(dist_obs_eff))))
        #         self.cont_ciclo_livre += 1
        # # if len(dist_obs_eff) and (min(dist_obs_eff) <= 1.5 * self.colisor):
        #     # self.vmax_dyn = 0
        #     # self.cont_preso += 1 
        #     # self.cont_ciclo_livre = 0
        #     # print("2")           
        # # elif len(dist_obs_eff) and min(dist_obs_eff) <= limiar_rep:
        #     # self.vmax_dyn = (self.vmax_abs/(limiar_rep/(sum(dist_obs_eff)/len(dist_obs_eff))))
        #     # self.cont_ciclo_livre += 1
        #     # print("3")   
        # elif np.linalg.norm(self.position[0:2] - self.goal_list[self.index_goal].position) <=limiar_rep:
        #     self.vmax_dyn = max(.2 * self.vmax_abs, (self.vmax_abs * np.linalg.norm((self.goal_list[self.index_goal].position - self.position[0:2]))/limiar_rep))
        #     self.cont_ciclo_livre += 1
        #     # print("4")   
        # else:
        #     self.vmax_dyn = self.vmax_abs
        #     self.cont_ciclo_livre += 1
        #     # print("5")   
        # a_dir = np.sign(math.cos(math.atan2(force[1],force[0]) - self.position[2]))
        self.a = np.clip((2*np.linalg.norm(force)/massa), -self.a_max, self.a_max) - self.a_old
        self.v = np.clip(self.v_old + (self.a) * dt, -self.vmax_dyn, self.vmax_dyn)
        # self.v = np.clip(self.v, -self.vmax_abs, self.vmax_abs)
        # if(self.a - self.a_old) <0:
        #     self.v = np.clip(self.v_old + (self.a) * dt, -self.vmax, self.vmax)
        # else:    
        #     self.v = np.clip(self.v_old + (self.a) * dt, -self.vmax, self.vmax)
        self.v_old = self.v
        self.a_old = self.a
        hud(self,force)
        # self.v = np.clip(np.linalg.norm(force), -self.vmax, self.vmax)
        # self.wl = ((self.v + self.wr)/self.r)
        # self.wr = ((self.v - self.wl)/self.r)
        # print(v)
        # self.w = (self.r/(2*self.l)) * (self.wl - self.wr)
        self.th = ajustaangulo(self.position[2])
        self.w = ajustaangulo(math.atan2(force[1], force[0])-self.th)
        # self.w = self.kp*self.alpha + self.ki*self.int_alpha + self.kd*self.dif_alpha
        # self.w = np.sign(self.w)* min(abs(self.w),self.wmax)
        self.w = np.clip(self.w, -self.wmax, self.wmax)
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
        
        self.conta_ciclo += 1
        x = 15  # Adiciona a posiçao atual numa lista
        if self.conta_ciclo % x == 0:
            self.path.append(self.position[0:2].copy())  
            self.conta_ciclo = 0
            
        if self.cont_ciclo_livre >=30:
            self.cont_preso = 0
            self.cont_ciclo_livre = 30
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
def att_force(q, goal, katt):
    # print((goal - q)/(np.linalg.norm(goal - q)))
    return katt*((goal - q)/(np.linalg.norm(goal - q)))


# Vetor de repulsão do obstáculo 
# def rep_force(q, obs):

#     R=obs[2] + 5
#     dist = np.linalg.norm(q - obs[0:2])
#     if dist <=0.001: dist=0.001 # Evitar divisão por 0
#     force = (R / dist) ** 3 * (q - obs[0:2])

#     return force *1.3

def rep_force(q, obs, collider, R=limiar_rep):
    # Obstáculo: (x, y, r)
    # dvec: distância vetorial
    # dist: módulo da distância
    #obs = [obs.x, obs.y, obs.r]
    dvec = q - obs[0:2]
    # if np.all(dvec == 0):
    #     return 0
    if len(obs)<3:
        dist = np.linalg.norm(dvec) - 10 - collider
    
    else:
        dist = np.linalg.norm(dvec) - obs[2] - collider
    
    rep = (1/dist**4) * (dvec/dist)#*((1/d)-(1/(obs[2])))
    # invalid = np.squeeze(d > R)
    # rep[invalid, :] = 0
    # #print(rep)
    return krep*rep

def rep_force_total(q, obstacles, collider):
    total_force = np.array([0.,0.])  # inicializa o array com 0
    for obs in obstacles:
        #print(q)
        #print(obs)
        x = obs.position[0]
        y = obs.position[1]
        if(obs.r<collider):
            obs.r = collider
        obs = np.array([x,y, obs.r])
        #print(q)
        #print(obs)
        if np.linalg.norm(q[0:2]-obs[0:2])- obs[2] - collider>limiar_rep:
            # print("skip rep")
            force = [0,0]
        else:
            # print(np.linalg.norm(q-obs[0:2]))
            angle  = calcula_gamma(q,obs[0:2])
            if angle >= 2*angle_sensor:
                force = [0,0]
            else:    
                # angle = np.clip(ajustaangulo(math.atan2(obs[1] - q[1],obs[0] - q[0])) - q[2],-math.pi/2, math.pi/2)
                # print(math.degrees(angle))
                # screen.blit(font.render(f"angle: {angle}", True, "white"), (20, 720 - 120))
                force = abs(math.cos(angle))*rep_force(q[0:2], obs, collider)  # Força de repulsão de cada obstáculo
        total_force += force
        # print(total_force)
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

def calcula_d(q, obstacles):
    distancias = []
    for obs in obstacles:
        d = np.linalg.norm(q[0:2] - obs.position[0:2]) - obs.r
        gamma = calcula_gamma(q, obs.position[0:2])
        if d <= limiar_rep and gamma < angle_sensor:
            distancias.append(int(d))
    # for player in players:
    #     if all(q[0:2]) == all(player.position[0:2]):
    #         continue
    #     d = np.linalg.norm(q[0:2] - player.position[0:2]) - 2*player.colisor
    #     gamma_p = calcula_gamma(q, player.position[0:2])
    #     if d <= 3*player.colisor and gamma_p < angle_sensor:
    #         distancias.append(int(d))
    return distancias

def calcula_gamma(q, obs):
    # print(obs)
    angle =(ajustaangulo(math.atan2(obs[1] - q[1],obs[0] - q[0])) - q[2])
    return abs(angle)

def trat_preso (self, goal, players):
    
        if self.preso is False:
            self.th_old = self.position[2]
            # Calcula um goal temporário 90º para o lado
            self.preso = True
            if math.atan2(self.position[1] - goal.position[1], self.position[0] - goal.position[0]) >=0:
                self.lado_rot_preso = 1
            else:
                self.lado_rot_preso = -1
        # Atração para o goal temporário
        side_angle = self.position[2] + self.lado_rot_preso * math.pi / 2 
        self.temp_goal = self.position[0:2] + np.array([
                50 * math.cos(side_angle),
                50 * math.sin(side_angle)
            ])
        force = att_force(self.position[0:2], self.temp_goal, katt) + 10* rep_force_total(self.position,players, self.colisor)
        
        if abs(abs(self.position[2]) - abs(self.th_old)) >= math.pi/2:
            self.cont_preso = 0
            self.preso = False
            self.temp_goal = [0,0]
        return force

def boids(self, players):
    v1 = 0
    v2 = [0,0]
    cm = [0,0] # "centro de massa" para visualização
    for player in players: #Regra 1 : Atração ao centro de massa do grupo
        cm += player.position[0:2]
        if player is not self:
            v1 += v1 + player.position[0:2] - self.position[0:2]
            dvec = player.position[0:2]- self.position[0:2]
            dist = np.linalg.norm(dvec)
            if dist <= limiar_rep_boids: #Regra 2 : Repulsão entre integrantes
                v2 = 2*(v2 - (dvec))
                if dist <= 3*self.colisor and dist> 2.5*self.colisor:
                    # print("C")
                    self.vmax_dyn = max(self.vmax_dyn * (dist/3*self.colisor), 20)
                # elif dist <= 2.5*self.colisor:
                #     self.vmax_dyn = 0
                # self.v = (self.v *dist/player.v *limiar_rep_boids)
                else:
                    self.vmax_dyn = self.vmax_abs
    v1 /= 100*(num_robot - 1)    
    cm /= num_robot
    dist_cm = np.linalg.norm([self.position[0]-cm[0],self.position[1]- cm[1]])
    if ajustaangulo(abs(math.atan2(cm[1] - self.position[1], cm[0] - self.position[0])))>math.pi/2:
        self.v = self.v * (dist_cm/limiar_rep_boids)
    pygame.draw.circle(screen,"white",cm,5)  
    # print(v2)
    pvJ = 0
    for player in players:
        if player is not self:
            pvJ += self.v
    if num_robot>1:
        pvJ /= num_robot - 1
    else:
        self.v = (pvJ - self.v)/8
    v4 = 0
    v4 = (self.goal_list[self.index_goal].position - self.position[0:2]) /25 #Regra 4 : Tendência a um local específico
    dist_goal = np.linalg.norm(self.position[0:2] - self.goal_list[self.index_goal].position)
    if dist_goal <= limiar_rep:
        self.vmax_dyn = max(self.vmax_dyn * (dist_goal/limiar_rep), 20)
    # print(v4)
    return v1 + v2 + v4