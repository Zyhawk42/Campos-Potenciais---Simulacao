import numpy as np
import math
import pygame

pygame.init()
screen_size = (1280,720)
screen = pygame.display.set_mode(screen_size)
font = pygame.font.Font(None, 24)

num_robot = 1
robot_x_max = 150
robot_y_max = 600
escala = 50
screen_dim = (1280,720)



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

def hud(items):
    y_offset = 20  # Initial y-coordinate
    y_spacing = 20  # Spacing between items

    for item in items:
        # Calculate the y-coordinate for this item
        item_y = 720 - y_offset
        screen.blit(font.render(f"x: {item}", True, "white"), (20, item_y))
        # Increment the y-offset for the next item
        y_offset += y_spacing