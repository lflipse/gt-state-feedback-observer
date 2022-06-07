import pygame
import numpy as np
from pygame.locals import *
from pygame import mixer
import time

class Visualize:
    def __init__(self, screen_width, screen_height, full_screen):
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.preview = False
        pygame.init()  # Initialize the game
        self.screen = pygame.display.set_mode((screen_width, screen_height))
        if full_screen == True:
            DISPLAYSURF = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

        # Title and Icon
        pygame.display.set_caption("Demo")
        icon = pygame.image.load("img/ufo.png")
        pygame.display.set_icon(icon)

        # Images
        playerImg = pygame.image.load("img/x-wing.png")
        enemyImg = pygame.image.load("img/player.png")
        self.img_size = 75
        self.y_player = int(2.5*screen_height/4)
        margin = 1.4
        self.y_enemy = self.y_player - int(self.img_size * margin)
        self.player = pygame.transform.scale(playerImg, (self.img_size, self.img_size))
        self.enemy = pygame.transform.scale(enemyImg, (self.img_size, self.img_size))
        self.bg_color_light = (255, 255, 255)
        self.bg_color_dark = (0, 0, 0)
        # self.bg_color = (0, 0, 0) # Black background
        self.line_color = (100, 100, 100)
        self.player_color = (179, 171, 142)
        self.preview_color = (224, 195, 97)
        self.arrow_color = (202, 182, 119)
        # ff8c00 hex color
        self.bg_color = (0, 0, 0)
        pygame.font.init()
        # print("available fonts: ", pygame.font.get_fonts())
        self.font = pygame.font.SysFont('georgia', 60)
        self.font_top = pygame.font.SysFont('georgia', 20)
        self.flicker = 0

        # mixer.init()
        # mixer.music.load("../Controller_Design/images/background.mp3")
        # mixer.music.play(-1, 0.0)
        # time.sleep(2)
        # mixer.music.stop()
        # pygame.event.wait()

    def visualize_experiment(self, r, r_prev, angle, text, top_text, bottom_text, sigma):
        x = self.translate_to_position(angle)
        x_r = self.translate_to_position(r)

        self.screen.fill(self.bg_color)
        dark = True

        if self.preview:
            self.show_preview(r_prev, sigma)
        self.show_player(x)
        self.show_target(r)
        self.draw_arrows(x, r)
        self.show_text(text, dark=dark)
        self.show_lower_text(bottom_text, dark=dark)
        self.show_top_text(top_text, dark=dark)

        pygame.display.update()

    def show_player(self, x):
        pygame.draw.line(self.screen, self.player_color, (x - 30, self.y_player), (x + 30, self.y_player), width=8)
        pygame.draw.line(self.screen, self.player_color, (x, self.y_player - 30), (x, self.y_player + 30), width=8)


    def translate_to_position(self, r):
        # Translate from angle between -30 to 30 degrees to
        angle_deg = r * 180 * np.pi / 45
        x_r = angle_deg / 60 * self.screen_width + 0.5 * self.screen_width
        return x_r

    def show_target(self, r):
        # x is the center, need to move
        # self.screen.blit(self.player, (x - 0.5*self.img_size, self.y_player + 0.5*self.img_size))
        x_r = self.translate_to_position(r)
        pygame.draw.circle(self.screen, self.preview_color, (x_r, self.y_player), 40, width=8)

    def show_text(self, text, dark):
        if dark:
            textsurface = self.font.render(text, False, (255, 255, 255))
        else:
            textsurface = self.font.render(text, False, (0, 0, 0))
        text_width, text_height = self.font.size(text)
        self.screen.blit(textsurface, (0.5*(self.screen_width-text_width), 0.4*self.screen_height))

    def show_lower_text(self, text, dark):
        if dark:
            textsurface = self.font.render(text, False, (255, 255, 255))
        else:
            textsurface = self.font.render(text, False, (0, 0, 0))
        text_width, text_height = self.font_top.size(text)
        self.screen.blit(textsurface, (0.3*(self.screen_width-text_width), 0.9*self.screen_height))

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
            dy = 0.2*(self.screen_height - self.y_player)/pieces
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



    def draw_arrows(self, x, r):
        x_r = self.translate_to_position(r)
        if x > x_r:
            dx = min(0.4 * (x - x_r), 30)
        else:
            dx = max(0.4 * (x - x_r), -30)
        pygame.draw.polygon(self.screen, self.arrow_color,
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