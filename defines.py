import numpy as np
import math
import pygame

pygame.init()
screen_size = (1280,720)
screen = pygame.display.set_mode(screen_size)
font = pygame.font.Font(None, 24)
sim_running = False

num_robot = 5
robot_x_max = 150
robot_y_max = 600
escala = 50
screen_dim = (1280,720)
massa = 9 #kg

limiar_rep = 75 #Limiar para calcular repulsão aos obstáculos
limiar_rep_boids = 2*limiar_rep/3
limiar_preso = 10
angle_sensor = math.radians(45)
katt = 100
krep = 100000

def T2D(M1, deltax, deltay):
    T = np.array([[1,0,deltax],
                  [0,1,deltay],
                  [0,0,1]])
    return T@M1

def Rz2D(M1, th):
    Rz = np.array([[np.cos(th), -np.sin(th),0],
                   [np.sin(th), np.cos(th), 0],
                   [0,0,1]])
    return Rz@M1

def normalize(angle):
	return np.arctan2(np.sin(angle),np.cos(angle))

def ajustaangulo(angulo):
        # angulo = abs(angulo)
        # if angulo>math.pi:
        #     angulo = angulo-2*math.pi
        # return angulo
        # def ajustaangulo(angulo):
    # Adjusts angle to range [-pi, pi]
    while angulo > math.pi:
        angulo -= 2 * math.pi
    while angulo < -math.pi:
        angulo += 2 * math.pi
    return angulo

def hud(self, force):
    # screen.blit(font.render(f"wl: {np.rad2deg(wl)}", True, "white"), (20, 720 - 100))
    # screen.blit(font.render(f"force: {force}", True, "white"), (20, 720 - 100))
    screen.blit(font.render(f"stuck_counter: {self.cont_preso}", True, "white"), (20, 720 - 100))
    screen.blit(font.render(f"vmax_dyn: {self.vmax_dyn}", True, "white"), (20, 720 - 80))
    # screen.blit(font.render(f"gamma: {self.gamma}", True, "white"), (20, 720 - 80))
    screen.blit(font.render(f"v: {self.v}", True, "white"), (20, 720 - 60))
    screen.blit(font.render(f"a: {self.a}", True, "white"), (20, 720 - 40))
    screen.blit(font.render(f"w: {self.w}", True, "white"), (20, 720-20))
        # Calculate position for speed text (adjust according to your player's position and size)
    speed_text_pos = (int(self.position[0]), int(self.position[1]) - 30)  # Adjust -20 for proper position above player
    
    # Render text surface
    # font = pygame.font.Font(None, 24)  # Choose your font and size
    speed_text = font.render(f"{self.v:.2f}", True, (255, 255, 255))  # Render speed with two decimal places
    
    # Draw speed text on the main display surface
    screen.blit(speed_text, speed_text_pos)