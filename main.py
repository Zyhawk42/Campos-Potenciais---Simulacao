import random
import numpy as np
import math
from obstacle import Obstacle
from robot import Robot
from goal import Goal
from defines import *
# pygame setup
# pygame.init()
# screen_size = (1280,720)
# screen = pygame.display.set_mode(screen_size)
clock = pygame.time.Clock()
running = True
dt = 0
i=0
# goal = np.array([1100, 600])

players = []
players_data = [[ 63 , 120 ],
[ 110, 79 ],
[ 121 , 531],
[ 69 , 310 ],
[ 56, 250 ]]

positions = []
# for j in range(num_robot):
#     x = random.randint(50, robot_x_max - 10)
#     y = random.randint(50, robot_y_max - 10)
#     if (x, y) not in positions:
#         for position in positions:
#             # print(position)
#             if abs(position[0] - x) <50 or abs(position[1] - y) <50:
#                 x = random.randint(50, robot_x_max - 10)
#                 y = random.randint(50, robot_y_max - 10)
#                 # print(j," reroll")
#                 j = j-1
#         else:
#             positions.append((x, y))
#             # print("add")
#     else: 
#         j = j-1
min_distance = 50
for _ in range(num_robot):
    while True:
        x = random.randint(50, robot_x_max - 10)
        y = random.randint(50, robot_y_max - 10)
        too_close = any(
            math.sqrt((px - x)**2 + (py - y)**2) < min_distance for px, py in positions
        )
        if not too_close:
            positions.append((x, y))
            break
    
# for k in range(num_robot):
#     player = Robot(float(positions[k][0]), float(positions[k][1]))
#     # player = Robot(float(100), float(100))
#     print("[",int(player.position[0]), ",", int(player.position[1]), "],")
#     players.append(player)

for player in players_data:
    x,y = player
    player = Robot(float(x),float(y))
    players.append(player)
    
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
    for player in players:
        player.goal_list.append(goal)

obstacles_data = np.array([
    #x, y, r
    # [400,350,60],
    # [600,300,50], 
    # [500,450,60],
    # [200,700,30], 
    # [1000,300,50],
    # [1200,400,30],
    # [750,600,30], 
    # [850,640,40]
    # [150,150,30] 
])

obstacles = []
# for j in range(num_obstacle):
#     x = random.randint(obstacle_size_max, screen_size[0] - obstacle_size_max)
#     y = random.randint(obstacle_size_max, screen_size[1] - obstacle_size_max)
#     r = random.randint(obstacle_size_min, obstacle_size_max)
#     obstacle = Obstacle(x, y, r)
#     obstacles.append(obstacle)
for obs in obstacles_data:
    x,y,r = obs
    obs = Obstacle(x,y,r)
    obstacles.append(obs)

# players_data = np.array([
#     # x, y
#     [100.,100.],
#     [130.,130.],
#     [100.,130.],
#     [130.,100.],
# ]
# )



# for coord in players_data:
#     x,y = coord
#     player = Robot(x,y)
#     players.append(player)

#print(players)

# XX, YY = np.meshgrid(np.arange(0, screen_size[0]+.4, .4), np.arange(0, screen_size[1]+.4, .4))
# XY = np.dstack([XX, YY]).reshape(-1, 2)
# Fatt = forces.att_force(XY, goals[i].position)
# Frep_obs = forces.rep_force_total(XY,obstacles)
# sim_running = False

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
    clock.tick(60)
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
        # goals[i].draw(screen)
        for obs in obstacles: 
            obs.draw(screen)
        g = 0
        for player in players:
            force = player.move_player(player.goal_list[player.index_goal],obstacles,players,dt)
            player.draw(screen, force)
            # print(player.position)
            if (math.sqrt((player.position[0] - player.goal_list[player.index_goal].position[0])**2 + (player.position[1] - player.goal_list[player.index_goal].position[1])**2))<= player.colisor:
                # player.index_goal = (player.index_goal+1)%len(goals_data)
                player.goal_reached = True
            if player.goal_reached is True:
                g +=1   
        if g == num_robot:
            for player in players:
                player.index_goal = (player.index_goal+1)%len(goals_data)
                player.goal_reached = False
        # distances = np.linalg.norm(player_positions - goals[i].position, axis=1)    
        #print(distances)        
        # if all(distance <= 60 for distance in distances):
        #     i = (i+1)%len(goals_data)
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