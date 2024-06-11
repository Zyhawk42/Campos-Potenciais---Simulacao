import pygame
import random
import numpy as np
import math

# pygame setup
pygame.init()
screen_size = (1280,720)
screen = pygame.display.set_mode(screen_size)
clock = pygame.time.Clock()
running = True
dt = 0
#cell = (pygame.Vector2,pygame.Vector2, pygame.Vector2)
#player_pos = pygame.Vector2(100,100)
#goal_pos = pygame.Vector2(1100,600)
player = np.array([100.0,100.0])
goal = np.array([1100, 600])

# Vetor de atração para o goal, retorna um vetor unitário
def att_force(q, goal, katt=50):
    if np.all(goal-q <=10): return np.array([0,0]) 
    return katt*((goal - q)/(np.linalg.norm(goal - q)))

def rep_force_total(q, obstacles):
    total_force = np.zeros_like(q)  # Initialize total force as zero vector
    for obs in obstacles:
        force = rep_force(q, obs)  # Calculate force for each obstacle
        total_force += force
    return total_force

# Vetor de repulsão do obstáculo 
def rep_force(q, obs):

  R=obs[2]
  # Calculate the distance between the point and the obstacle center
  distance = np.linalg.norm(q - obs[0:2])

  # Ensure the distance is greater than a small threshold to avoid division by zero
  threshold = 1e-6
  distance = np.maximum(distance, threshold)

  # Repulsion force formula (increases as distance gets smaller)
  force = (R / distance) ** 3 * (q - obs[0:2])

  return force

def move_object(player_pos, goal, katt=0.01, max_speed=1.0):
    """
    Updates the position of an object towards a goal, considering max speed.

    Args:
        player_pos (numpy.ndarray): Current position of the object (2D array).
        goal (numpy.ndarray): Goal position towards which the object moves.
        katt (float, optional): Strength of the attractive force. Defaults to 0.01.
        max_speed (float, optional): Maximum speed the object can move. Defaults to 1.0.

    Returns:
        numpy.ndarray: Updated position of the object.
    """
    force = att_force(player_pos, goal, katt) + rep_force_total(player_pos,obstacles)  # Calculate attractive force
    # Limit the force magnitude to avoid exceeding max speed
    force_limited = np.clip(force, -max_speed, max_speed)
    # Update position based on force and a small time step (dt)
    new_pos = player_pos + force_limited * dt
    return new_pos

obstacles = np.array([
    [400, 350, 60],  
    [600, 300, 50], 
])

XX, YY = np.meshgrid(np.arange(0, screen_size[0]+.4, .4), np.arange(0, screen_size[1]+.4, .4))
XY = np.dstack([XX, YY]).reshape(-1, 2)
Fatt = att_force(XY, goal)
#Fatt_x = Fatt[:,0]
#Fatt_y = Fatt[:,1]
Frep = rep_force_total(XY,obstacles)
Ft = Fatt + Frep

while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # fill the screen with a color to wipe away anything from last frame
    #screen.fill("black")
    pygame.draw.rect(screen, "red", (player[0] - 5, player[1] - 5,10,10))
    pygame.draw.circle(screen, "green", goal, 10)
    for obs in obstacles:
        pygame.draw.circle(screen, "blue",(obs[0], obs[1]), obs[2])
    #pygame.draw.rect(screen, "blue", (400,350,100,200))

    player = move_object((player),goal,100,50)
    # flip() the display to put your work on screen
    pygame.display.flip()

    # limits FPS to 60
    # dt is delta time in seconds since last frame, used for framerate-
    # independent physics.
    dt = clock.tick(60) / 1000

pygame.quit()