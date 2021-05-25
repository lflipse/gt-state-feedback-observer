import pygame
import numpy as np
from pygame.locals import *

class Visualize:
    def __init__(self, screen_width, screen_height):
        self.screen_width = screen_width
        self.screen_height = screen_height
        pygame.init()  # Initialize the game
        self.screen = pygame.display.set_mode((screen_width, screen_height))
        DISPLAYSURF = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

        # Title and Icon
        pygame.display.set_caption("Experiment")
        icon = pygame.image.load("images/ufo.png")
        pygame.display.set_icon(icon)

        # Images
        playerImg = pygame.image.load("images/x-wing.png")
        enemyImg = pygame.image.load("images/player.png")
        self.img_size = 100
        self.y_player = int(3*screen_height/4)
        margin = 1.4
        self.y_enemy = self.y_player - int(self.img_size * margin)
        self.player = pygame.transform.scale(playerImg, (self.img_size, self.img_size))
        self.enemy = pygame.transform.scale(enemyImg, (self.img_size, self.img_size))
        self.bg_color = (255, 255, 255)
        self.line_color = (100, 100, 100)
        self.font = pygame.font.Font('freesansbold.ttf', 32)


    def visualize_experiment(self, r, angle):
        x = self.translate_to_position(angle)
        x_r = self.translate_to_position(r)
        self.screen.fill(self.bg_color)
        pygame.draw.line(self.screen, self.line_color, [x, self.y_player + self.img_size], [x, self.y_player + self.img_size + 40], width=4)
        self.show_enemy(self.translate_to_position(r))
        pygame.draw.line(self.screen, self.line_color, [x_r, self.y_player + self.img_size],
                         [x_r, self.y_player + self.img_size + 40], width=4)
        if x > x_r:
            dx = min(0.4 * (x - x_r), 30)
        else:
            dx = max(0.4 * (x - x_r), -30)
        pygame.draw.polygon(self.screen, self.line_color,
                        ((x, self.y_player + 1.15 * self.img_size), (x, self.y_player + 1.25 * self.img_size),
                         (x_r + dx, self.y_player + 1.25 * self.img_size), (x_r + dx, self.y_player + 1.4 * self.img_size),
                         (x_r, self.y_player + 1.2 * self.img_size),
                         (x_r + dx, self.y_player + self.img_size), (x_r + dx, self.y_player + 1.15 * self.img_size)))

        self.show_player(x)

        # display_surface = pygame.display.set_mode((int(0.5*self.screen_width), (int(0.5*self.screen_height))))
        # text = self.font.render('GeeksForGeeks', False, self.line_color, self.line_color)
        # textRect = text.get_rect()
        # display_surface.blit(text, textRect)



        pygame.display.update()

    def translate_to_position(self, r):
        # Translate from angle between -30 to 30 degrees to
        angle_deg = r * 180 * np.pi / 45
        x_r = angle_deg/60 * self.screen_width + 0.5 * self.screen_width
        return x_r

    def show_player(self, x):
        # x is the center, need to move
        self.screen.blit(self.player, (x - 0.5*self.img_size, self.y_player))

    def show_enemy(self, x):
        # x is the center
        self.screen.blit(self.enemy, (x - 0.5*self.img_size, self.y_enemy))

    def check_quit(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("Quited this game")
                return
            elif event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    pygame.quit()
                    print("Quited this game")
                    return

    def quit(self):
        pygame.display.quit()
        pygame.quit()