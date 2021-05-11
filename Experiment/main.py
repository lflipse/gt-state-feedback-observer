import pygame
import numpy as np
import time

def generate_reference(T):

    x_d = 40*np.pi/180

    # Reference signal
    fs1 = 1 / 8
    fs2 = 1 / 20
    fs3 = 1 / 37
    fs4 = 1 / 27

    r = x_d * (np.sin(2 * np.pi * fs1 * T) + np.sin(2 * np.pi * fs2 * T) + np.sin(2 * np.pi * fs3 * T) + np.sin(2 * np.pi * fs4 * T))

    return r



# Simulation parameters
t = 20
h = 0.01
N = round(t/h) + 1
T = np.array(range(N)) * h
r = generate_reference(T)

# Dynamics
Jw = 0.22
Bw = 0.1
Kw = 1
A = np.array([[0, 1], [- Bw / Jw, -Kw / Jw]])
B = np.array([[0], [1 / Jw]])




class Experiment():
    def __init__(self, screen_width, controller, human):
        self.screen_width = screen_width
        self.controller = controller
        self.human = human

        # Initialize the entire thing
        # Initialize the game
        pygame.init()

        # I guess depends on screen dimensions, full hd for now
        # screen = pygame.display.set_mode((1920, 1080))
        self.screen = pygame.display.set_mode((800, 600))

        # Title and Icon
        pygame.display.set_caption("Experiment")
        icon = pygame.image.load("images/ufo.png")
        pygame.display.set_icon(icon)

        playerImg = pygame.image.load("images/player.png")
        self.playerImg = pygame.transform.scale(playerImg, (50, 50))
        self.playerX = 370
        self.playerY = 480
        self.bg_color = (255, 255, 255)
        self.border_color = (0, 0, 0)
        self.rect_width = 80
        self.rect_height = 50
        self.rect_surf = self.create_rect(self.rect_width, self.rect_height, 4, self.bg_color, self.border_color)

    def create_rect(self, width, height, border, color, border_color):
        surf = pygame.Surface((width + border * 2, height + border * 2), pygame.SRCALPHA)
        pygame.draw.rect(surf, color, (border, border, width, height), 0)
        for i in range(1, border):
            pygame.draw.rect(surf, border_color, (border - i, border - i, width + 5, height + 5), 1)
        return surf

    def translate_to_position(self, angle):
        # Translate from angle between -40 to 40 degrees to
        angle_deg = r * 180 * np.pi / 40
        x_r = angle_deg/80 * self.screen_width + 0.5 * self.screen_width
        return x_r

    def player(self, x, y):
        self.screen.blit(self.playerImg, (x, y))

    def rectangle(self, rect_surf, r, y):
        self.screen.blit(rect_surf, (r, y))

    def do_experiment(self, r, N, h):
        # Game loop
        running = True

        y = np.zeros((N + 1, 4))

        # Estimator vectors
        Qhhat = np.zeros((N + 1, 2, 2))
        Phhat = np.zeros((N + 1, 2, 2))
        Ph = np.zeros((N, 2, 2))
        Pr = np.zeros((N, 2, 2))
        Qh = np.zeros((N, 2, 2))
        Qr = np.zeros((N, 2, 2))
        Lhhat = np.zeros((N, 2))
        uhhat = np.zeros(N)

        # Real vectors
        Lh = np.zeros((N, 2))
        Lr = np.zeros((N, 2))
        ref = np.zeros((N, 2))
        e = np.zeros((N, 2))
        x = np.zeros((N, 2))
        xdot = np.zeros((N, 2))
        xhhat = np.zeros((N, 2))
        ur = np.zeros(N)
        uhbar = np.zeros(N)
        vh = np.zeros(N)
        uh = np.zeros(N)


        for i in range(N):
            t1 = time.time()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    exit("Quited the game")

            # Measure stuff
            # Calcuate derivative(s) of reference
            if i > 0:
                ref[i, :] = np.array([r[i], (r[i] - r[i - 1]) / h])
                xdot[i, :] = (x[i, :] - x[i - 1, :]) / h
            else:
                ref[i, :] = np.array([r[i], (r[i]) / (2 * h)])
                xdot[i, :] = (x[i, :]) / (2 * h)

            e[i, :] = x[i, :] - ref[i, :]  # Assuming x is measured

            if i > 0:
                # Calculated control inputs
                # uh[i], Lh = self.human.compute_controls()
                Pr[i, :], Phhat[i, :], Qhhat[i, :, :] = self.robot.compute_controls()

            # Integrate dynamics

            self.screen.fill(self.bg_color)
            x_r = self.translate_to_position(r[i])
            self.rectangle(self.rect_surf, x_r, self.playerY)
            self.player(self.playerX, self.playerY)
            pygame.display.update()

            d = time.time() - t1
            if d < h:
                time.sleep(h - d)
            else:
                print('having a delay')


        print("Finished the game")

