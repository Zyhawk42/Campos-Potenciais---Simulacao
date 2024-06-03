import pygame

pygame.init()

Background = pygame.display.set_mode((900 ,900))

for i in range(0, 900, 50):
    pygame.draw.line(Background, (255, 255, 255), (0, i), (900, i))
    pygame.draw.line(Background, (255, 255, 255), (i, 0), (i, 900))
pygame.display.update()

while pygame.event.wait().type != pygame.QUIT:
    pass