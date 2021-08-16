import pygame
import numpy as np
from pygame.locals import *

class Visualize:
    def __init__(self, screen_width, screen_height, full_screen):
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.preview = True
        pygame.init()  # Initialize the game
        self.screen = pygame.display.set_mode((screen_width, screen_height))
        if full_screen == True:
            DISPLAYSURF = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

        # Title and Icon
        pygame.display.set_caption("Controller_Design")
        icon = pygame.image.load("../Controller_Design/images/ufo.png")
        pygame.display.set_icon(icon)

        # Images
        playerImg = pygame.image.load("../Controller_Design/images/x-wing.png")
        enemyImg = pygame.image.load("../Controller_Design/images/player.png")
        self.img_size = 100
        self.y_player = int(3*screen_height/4)
        margin = 1.4
        self.y_enemy = self.y_player - int(self.img_size * margin)
        self.player = pygame.transform.scale(playerImg, (self.img_size, self.img_size))
        self.enemy = pygame.transform.scale(enemyImg, (self.img_size, self.img_size))
        self.bg_color_light = (255, 255, 255)
        self.bg_color_dark = (0, 0, 0)
        # self.bg_color = (0, 0, 0) # Black background
        self.line_color = (100, 100, 100)
        self.preview_color = (155, 135, 12)
        self.bg_color = (0, 0, 0)
        pygame.font.init()
        # print("available fonts: ", pygame.font.get_fonts())
        self.font = pygame.font.SysFont('georgia', 60)
        self.font_top = pygame.font.SysFont('georgia', 20)

    def visualize_experiment(self, r, r_prev, angle, text, top_text, sigma, role=""):
        x = self.translate_to_position(angle)
        x_r = self.translate_to_position(r)

        self.screen.fill(self.bg_color)
        dark = True

        if self.preview:
            self.show_preview(r_prev, sigma)
        self.show_player(x)
        self.show_text(text, dark=dark)
        self.show_lower_text(top_text, dark=dark)
        self.show_top_text(role, dark=dark)

        pygame.display.update()

    def translate_to_position(self, r):
        # Translate from angle between -30 to 30 degrees to
        angle_deg = r * 180 * np.pi / 45
        x_r = angle_deg / 60 * self.screen_width + 0.5 * self.screen_width
        return x_r

    def show_player(self, x):
        # x is the center, need to move
        self.screen.blit(self.player, (x - 0.5*self.img_size, self.y_player))

    def show_text(self, text, dark):
        if dark:
            textsurface = self.font.render(text, False, (255, 255, 255))
        else:
            textsurface = self.font.render(text, False, (0, 0, 0))
        text_width, text_height = self.font.size(text)
        self.screen.blit(textsurface, (0.5*(self.screen_width-text_width), 0.5*self.screen_height))

    def show_lower_text(self, text, dark):
        if dark:
            textsurface = self.font.render(text, False, (255, 255, 255))
        else:
            textsurface = self.font.render(text, False, (0, 0, 0))
        text_width, text_height = self.font_top.size(text)
        self.screen.blit(textsurface, (0.5*(self.screen_width-text_width), 0.9*self.screen_height))

    def show_top_text(self, text, dark):
        if dark:
            textsurface = self.font.render(text, False, (255, 255, 255))
        else:
            textsurface = self.font.render(text, False, (0, 0, 0))
        text_width, text_height = self.font_top.size(text)
        self.screen.blit(textsurface, (0.5*(self.screen_width-text_width), 0.1*self.screen_height))

    def show_preview(self, r, sigma):
            pieces = len(r) - 1
            if pieces > 2:

                points = []
                dy = 3.2*(self.screen_height - self.y_player)/pieces
                for i in range(pieces):
                    # Draw incremental lines from parts of the reference trajectory
                    points.append((self.translate_to_position(r[i]), self.y_player-i*dy+0.5*self.img_size))
                # print(points)
                if points != []:
                    # If sigma > 0 draw point cloud in stead of lines
                    if sigma > 0:
                        for point in points:
                            pygame.draw.circle(self.screen, self.preview_color, point, 5)
                    else:
                        pygame.draw.circle(self.screen, self.preview_color,
                                           (self.translate_to_position(r[0]), self.y_player + 0.5 * self.img_size), 10)
                        for point in points:
                            pygame.draw.circle(self.screen, self.preview_color, point, 5)

    def draw_arrows(self, x, x_r):
        if x > x_r:
            dx = min(0.4 * (x - x_r), 30)
        else:
            dx = max(0.4 * (x - x_r), -30)
        pygame.draw.polygon(self.screen, self.line_color,
                        ((x, self.y_player + 1.15 * self.img_size), (x, self.y_player + 1.25 * self.img_size),
                         (x_r + dx, self.y_player + 1.25 * self.img_size), (x_r + dx, self.y_player + 1.4 * self.img_size),
                         (x_r, self.y_player + 1.2 * self.img_size),
                         (x_r + dx, self.y_player + self.img_size), (x_r + dx, self.y_player + 1.15 * self.img_size)))

    def check_quit(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("Quited this game")
                return True
            elif event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    pygame.quit()
                    print("Quited this game")
                    return True
            else:
                return False

    def quit(self):
        pygame.display.quit()
        pygame.quit()