# Example file showing a circle moving on screen
import pygame
import random
import drawarrow

# pygame setup
pygame.init()
screen_size = (1280,720)
screen = pygame.display.set_mode(screen_size)
clock = pygame.time.Clock()
running = True
dt = 0
x_arrow = 200.0
y_arrow = 200.0
arrow_mult = 100
player_pos = pygame.Vector2(100,100)
goal_pos = pygame.Vector2(1100,600)
coord_arrow = pygame.Vector2(x_arrow,y_arrow)


while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # fill the screen with a color to wipe away anything from last frame
    screen.fill("white")
    pygame.draw.circle(screen, "red", player_pos, 40)
    pygame.draw.circle(screen, "green", goal_pos, 10)
    pygame.draw.circle(screen, "blue",(200,50), 40)
    pygame.draw.rect(screen, "blue", (400,350,100,200))
    for i in range(0, screen_size[0], 20):
        pygame.draw.line(screen, (0, 0, 0), (0, i), (screen_size[0], i))
        pygame.draw.line(screen, (0, 0, 0), (i, 0), (i, screen_size[1]))
    pygame.display.update()
    #drawarrow.draw_arrow(screen,coord_arrow, (210,210),"black")

   
    player_pos.y += (random.randrange(-100,100,1)) * dt
    player_pos.x += (random.randrange(-100,100,1)) * dt

    # flip() the display to put your work on screen
    pygame.display.flip()

    # limits FPS to 60
    # dt is delta time in seconds since last frame, used for framerate-
    # independent physics.
    dt = clock.tick(60) / 1000

pygame.quit()