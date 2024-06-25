import pygame
import numpy as np

# pygame setup
pygame.init()
screen_size = (1280,720)
screen = pygame.display.set_mode(screen_size)
clock = pygame.time.Clock()
running = True
dt = 0
#x,y,theta
player = np.array([100.,
                  100.,
                  1.]                               
                  )
th = player[2]
vertices = np.array([(player[0] -5 , player[1] -5),(player[0] -5 , player[1] +5),(player[0] +5 , player[1] +5), (player[0] +5 , player[1] -5)])
rot_mat = np.array(
    [[np.cos(np.deg2rad(th)), -np.sin(np.deg2rad(th))],
    [np.sin(np.deg2rad(th)), np.cos(np.deg2rad(th))]]
)

def Rz(theta):

    return np.array([[ np.cos(theta), -np.sin(theta)],
                      [ np.sin(theta), np.cos(theta) ],
                      [ 0            , 0            ]])


# def create_2d_homogeneous_transform(translation=(0, 0), scale=1, rotation=0):
#   """
#   Creates a 2D homogeneous transformation matrix.

#   Args:
#       translation (tuple, optional): The translation vector (tx, ty). Defaults to (0, 0).
#       scale (float, optional): The scaling factor. Defaults to 1.
#       rotation (float, optional): The rotation angle in radians. Defaults to 0.

#   Returns:
#       numpy.ndarray: The 3x3 homogeneous transformation matrix.
#   """
#   theta = np.radians(rotation)
#   c, s = np.cos(theta), np.sin(theta)
#   T = np.eye(3)
#   T[:2, :2] = np.array([[c, -s], [s, c]])
#   T[:2, 2] = translation
#   T[2, 2] = scale
#   return T

while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        screen.fill("black")
        #player[2] += 1
        #player_coord = player[0:2]
        #player[0:2] = np.matmul(rot_mat, player_coord)
        print(player)
        pygame.draw.polygon(screen, "red", vertices)
        transf_mat = create_2d_homogeneous_transform((10,10),1,10)
        player = np.matmul(transf_mat,player)
        vertices = np.array([(player[0] -5 , player[1] -5),(player[0] -5 , player[1] +5),(player[0] +5 , player[1] +5), (player[0] +5 , player[1] -5)])
        # flip() the display to put your work on screen
        pygame.display.flip()

        # limits FPS to 60
        # dt is delta time in seconds since last frame, used for framerate-
        # independent physics.
        dt = clock.tick(60) / 1000

pygame.quit()