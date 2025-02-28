import sys
import numpy as np
import pygame
import math

pygame.init()
screen_size = np.array([1280,720])
font = pygame.font.Font(None, 24)
num_robot = 10         
vmax_lin = 100
wmax = math.radians(60)       
limiar_rep = 75  

k_v1 = .5  # Fator de atração ao "centro de massa"
k_v2 = 10   # Fator de separação
k_v4 = 1   # Fator de atração ao goal

l_k = [k_v1, k_v2, 0, k_v4]
i = 4

r_boids = 5
cor_boids = "cyan" 
sim_running = False

class Boid:
    def __init__(self):
        self.position = np.array([
            np.random.uniform(0, screen_size[0]),
            np.random.uniform(0, screen_size[1]),
            np.random.uniform(0, 2*math.pi),  # ângulo theta
        ])
        self.tam = 10 #Tamanho do boid
        self.velocity = np.array([0., 0.])
        norm = np.linalg.norm(self.velocity)
        if norm != 0:
            self.velocity = (self.velocity / norm) * vmax_lin

    def draw(self, screen):
        # pygame.draw.circle(screen, cor_boids, (int(self.position[0]), int(self.position[1])), r_boids)
        # def draw(self, screen):
        lado = self.tam / 2.0  # metade do lado
        theta = self.position[2]
        arestas = [
            (lado, -lado),  # canto superior direito
            (lado, lado),   # canto inferior direito
            (-lado, lado),  # canto inferior esquerdo
            (-lado, -lado)  # canto superior esquerdo
        ]
        arestas_rot = []
        for (x, y) in arestas:# Rotaciona os vértices de acordo com o ângulo theta
            xr = x * math.cos(theta) - y * math.sin(theta)
            yr = x * math.sin(theta) + y * math.cos(theta)
            arestas_rot.append((self.position[0] + xr, self.position[1] + yr))
        
        pygame.draw.polygon(screen, cor_boids, arestas_rot)
        
        frente_x = self.position[0] + lado * math.cos(theta) #Calcula a "frente" do robô
        frente_y = self.position[1] + lado * math.sin(theta)
        pygame.draw.line(screen, "red", (self.position[0], self.position[1]), (frente_x, frente_y), 2) #Linha do centro até a frente


def calc_v(theta, x_dot_d, y_dot_d):
    T = np.array([
        [math.cos(theta), math.sin(theta)],
        [-math.sin(theta), math.cos(theta)]
    ])
    vw = np.array([x_dot_d, y_dot_d])
    resultado = T.dot(vw)
    return resultado  # resultado[0] = v, resultado[1] = w

def boids(self, players):
    # Cálculo das regras dos boids (atração, repulsão, alinhamento, atração ao goal)
    v1 = np.array([0., 0.])
    v2 = np.array([0., 0.])
    cm = np.array([0., 0.])  # centro de massa para visualização
    
    for player in players:
        cm += player.position[0:2]
        if player is not self:
            # Regra 1: Atração ao centro dos demais
            v1 += player.position[0:2] - self.position[0:2]
            dvec = player.position[0:2] - self.position[0:2]
            dist = np.linalg.norm(dvec)
            if dist <= limiar_rep:  # Regra 2: Repulsão se muito próximos
                v2 -= (dvec - self.tam)
                # print(np.linalg.norm(v2))
    v1 /= (num_robot - 1)
    cm /= num_robot

    pygame.draw.circle(screen, "white", (int(cm[0]), int(cm[1])), 5)  
    mouse_position = np.array(pygame.mouse.get_pos())
    pygame.draw.circle(screen, "red", (int(mouse_position[0]), int(mouse_position[1])), 10)
    
    # Regra 4: Tendência a um local específico 
    v4 = mouse_position - self.position[0:2]
    
    # Regra 3: Alinhamento (igualar velocidades)
    pvJ = np.array([0., 0.])
    for player in players:
        if player is not self:
            pvJ += player.velocity
    if num_robot > 1:
        pvJ /= (num_robot - 1)
    self.velocity = (pvJ - self.velocity) / 8 #Vetor velocidade
    
    return (l_k[0] * v1 + l_k[1] * v2 + l_k[3] * v4) / 10

screen = pygame.display.set_mode((screen_size))
pygame.display.set_caption("Teste Boids")
clock = pygame.time.Clock()

# Cria uma lista de boids
boids_list = [Boid() for j in range(num_robot)]

running = True
while running:
    dt = clock.tick(60) / 1000.0
    screen.fill("black") 
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                sim_running = not sim_running
            elif event.key == pygame.K_1:
                i = 1
            elif event.key == pygame.K_2:
                i = 2
            elif event.key == pygame.K_4:
                i = 4
            elif event.key == pygame.K_r:
                l_k[i-1] = 1
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 4:
                l_k[i-1] += 0.2
            elif event.button == 5:
                l_k[i-1] -= 0.2

    if sim_running:
        for boid in boids_list:
            v_vec = boids(boid, boids_list) #Vetor velocidade
            v_lin = np.linalg.norm(v_vec) #Velocidade linear
            if v_lin > vmax_lin:
                v_vec = (v_vec / v_lin) * vmax_lin
            theta = boid.position[2]
            v, w = calc_v(theta, v_vec[0], v_vec[1])
            np.clip(v,-vmax_lin, vmax_lin)
            np.clip(w,-wmax, wmax)
            
            boid.position[0] += v * math.cos(theta) * dt
            boid.position[1] += v * math.sin(theta) * dt
            boid.position[2] += w * dt/10

        for boid in boids_list:
            boid.draw(screen)
        screen.blit(font.render(f"kcm: {l_k[0]:.1f}", True, "white"), (20, 720 - 100))
        screen.blit(font.render(f"krep: {l_k[1]:.1f}", True, "white"), (20, 720 - 80))
        screen.blit(font.render(f"katt: {l_k[3]:.1f}", True, "white"), (20, 720 - 60))
    
    pygame.display.flip()

pygame.quit()
