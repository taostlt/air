import pygame
import matplotlib.pyplot as plt
import math, sys, random
from pygame.color import *
from pygame.locals import *

import math, sys, random

import numpy as np

import matplotlib.pyplot as plt
import matplotlib.pylab as pylab # this comes from 1. Coaxial Drone Dynamics SOLUTION and may not be necessary
                                 # it is used as pylab.rcParams['figure.figszie'] = 10, 10
from pymunk.pygame_util import DrawOptions
import webbrowser

import plotting

import pymunk
class Switch:
    def __init__(self):
        self.light_status = np.array([0.0])

    def light_on(self, screen, image):
        print('Light is ON')
        screen.blit(image, (0,0))

    def light_off(self, screen):
        print('Light is OFF')
        screen.fill((0, 0, 0))
class Card(pygame.sprite.Sprite):
    # For color codes |https://python-graph-gallery.com/196-select-one-color-with-matplotlib/
    def __init__(self, screen, font,
                 card_text, card_text_x, card_text_y, card_width, card_height,
                 card_color = THECOLORS["royalblue"], color_text = THECOLORS["white"]):
        super(Card, self).__init__()

        offset_x, offset_y = 5, 5  # padding for cards around the edges
        self.surf = pygame.Surface((card_width + offset_x, card_height + offset_y))
        self.surf.fill(card_color)
        self.rect = self.surf.get_rect()
        self.card_text = card_text
        self.screen = screen
        screen.blit(self.surf, (card_text_x - offset_x, card_text_y - offset_y))
        screen.blit(font.render(card_text, True, (color_text)), (card_text_x, card_text_y))
        # screen.blit(font.render(dynamic_text, True, (color_text)), (dynamic_text_x, dynamic_text_y))

class Monorotor:

    def __init__(self, m=1.0):
        self.m = m
        self.g = 9.81

        # note that we're no longer thinking of rotation rates.
        # We are thinking directly in terms of thrust.
        self.thrust = 0.0

        # z, z_dot
        self.X = np.array([0.0, 0.0])

    @property
    def z(self):
        return self.X[0]

    @property
    def z_dot(self):
        return self.X[1]

    @property
    def z_dot_dot(self):
        f_net = self.m * self.g - self.thrust
        return f_net / self.m

    def advance_state(self, dt):
        X_dot = np.array([
            self.z_dot,
            self.z_dot_dot])

        self.X = self.X + X_dot * dt
        return self.X

class PController:

    def __init__(self, k_p, m):
        self.k_p = k_p
        self.vehicle_mass = m
        self.g = 9.81

    def thrust_control(self, z_target, z_actual):
        # TODO - implement this method
        err = z_target - z_actual

        # u_bar is what we want vertical acceleration to be
        u_bar = self.k_p * err

        # u is the thrust command which will cause u_bar
        u = self.vehicle_mass * (self.g - u_bar)

        return u

def main():
    pygame.init()
    size = width, height = 600, 600
    screen = pygame.display.set_mode(size)
    pygame.display.set_caption("This is just a test")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont('Consolas', 25)

    space = pymunk.Space()
    space.gravity = 0, 9.800
    j = -1
    gravity_ref = space.gravity[1]

    start_ticks = pygame.time.get_ticks()                           # starter ticks

    image = pygame.image.load(r'/home/frank/Pictures/Amazon_Response_By_Seller.png')
    mySwitch = Switch()

    loop_count = 0
    MASS_ERROR = 1.0
    K_P = 10.0

    # preparation
    drone = Monorotor()
    perceived_mass = drone.m * MASS_ERROR
    controller = PController(K_P, perceived_mass)

    total_time = 20.0
    dt = 0.002
    # t = np.linspace(0.0, total_time, int(total_time / dt))

    # run simulation
    history = []
    z_target = np.array([0.0])
    # for z_target in z_path:
    #     u = controller.thrust_control(z_target, z_actual)
    #     drone.thrust = u
    #     drone.advance_state(dt)
    #     history.append(drone.X)
    # z_path = -np.ones(t.shape[0])
    z_path = 1.0

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                sys.exit(0)
            elif event.type == KEYDOWN and event.key == K_ESCAPE:
                sys.exit(0)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_UP:
                print('key UP')
                mySwitch.light_on(screen, image)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_DOWN:
                print('key DOWN')
                mySwitch.light_off(screen)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_RIGHT:
                print('key RIGHT')
                myCard = Card(screen, font, str('Right'), 300, 300, 20, 20, THECOLORS["royalblue"],
                              THECOLORS["white"])
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_LEFT:
                print('key LEFT')
                myCard = Card(screen, font, str('Left'), 300, 300, 20, 20, THECOLORS["royalblue"],
                              THECOLORS["white"])

        clock.tick(500)
        seconds = (pygame.time.get_ticks() - start_ticks) / 1000.00
        time = float(seconds)
        print(f'TIME: {int(time)}')
        loop_count += 1

        z_actual = drone.z
        print(f'drone z: {z_actual}')
        print(f'')
        z_target = 1.0
        u = controller.thrust_control(z_target,z_actual)
        drone.thrust = u
        drone.advance_state(dt)
        history.append(drone.X)

        if time > 10.00:
            # generate plots
            t = np.linspace(0.0, int(time), int(time)+1)
            z_target = np.ones((1,9))
            print(f't: {t}')
            z_actual = [h[0] for h in history]
            print(f'z actual: {np.round(z_actual,2)}')

            print(f'History: {history}')
            print(f'z target shape is: \n {z_target}')
            n = np.arange(0, 10, 1)
            print(f'n: {n, n.shape}')
            print(f'z target shape is: {z_target.shape}')

            plotting.compare_planned_to_actual(z_actual, z_path, t)
            sys.exit(0)

        pygame.display.flip()

if __name__ == '__main__':
    sys.exit(main())