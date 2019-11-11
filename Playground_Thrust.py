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

class OpenLoopController:
    # Robert Redford has a donkey high on a hill.                                            # main character, target_z
    # Ownen Wilson shows up with his donkey, only part way up the hill.                   # second character, current_z
    # Paul Newman compares Robert to Owen. "Look at the distance between."                   # third character, delta_z
    # Paul runs up the hill a short distance. "You have to go this fast."       # Paul show velocity, z_target_dot / dt
    # Saffron tells them, they need to go even faster.                 # fourth character, z_target_dot - z_current_dot
    # "You see how fast Robert is going compared to Owen?" She runs faster up the hill.                  # delta_z / dt
    # The gold becomes carrots.
    # Saffron says,                                                              # fnet_thrust = mass * z_target_dot_dot
    # "This is the motivation required to get your bags of gold up the hill."
    # "Of course your Donkeys have their weight also."                    # thrust = donkey mass * gravity - fnet_thrust
    # The line man (Van Cleeve) says "Boy, look where your Donkey is at." Owen's has moved up the hill.
    # You only have this much more to go"  he shows a gap,
    # "and you only need to do it this quickly." He zips his fingers across an imaginary space.
    #                                                                 # self.vehicle += np.array([delta_z, delta_z_dot])
    # "Here are your carrots." He hands them over.                                                       # return thrust


    def __init__(self, vehicle_mass, initial_state=np.array([0.0, 0.0])):
        self.vehicle_mass = vehicle_mass
        self.vehicle_state = initial_state
        self.g = 9.81

    def thrust_control(self, target_z, dt):
        current_z, current_z_dot = self.vehicle_state

        delta_z = target_z - current_z
        target_z_dot = delta_z / dt

        delta_z_dot = target_z_dot - current_z_dot
        target_z_dot_dot = delta_z_dot / dt

        f_net_thrust = target_z_dot_dot * self.vehicle_mass
        thrust = self.g * self.vehicle_mass - f_net_thrust

        self.vehicle_state += np.array([delta_z, delta_z_dot])

        return thrust

def main():
    # Intialize Pymunk Environment
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

    # My Code to turn the light switch on.
    image = pygame.image.load(r'/home/frank/Pictures/Amazon_Response_By_Seller.png')
    mySwitch = Switch()

    # Initialize System
    loop_count = 0
    dt = 0.1

    drone = Monorotor()
    MASS_ERROR = 1.001
    drone_start_state = drone.X
    perceived_mass = drone.m * MASS_ERROR
    controller = OpenLoopController(perceived_mass, drone_start_state)
    # controller = PController(K_P, perceived_mass)

    drone_state_history = []
    history = []
    z_path = np.array([1.0])
    z_target = np.array([0.0])
    z_actual = [np.array([0.0])]
    time_history = np.array([0.0])
    z_targPy = np.array([1.0])
    z_targPy_history = np.array([0.0])

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
        fps = 10.0
        clock.tick(fps)
        seconds = (pygame.time.get_ticks() - start_ticks) / 1000.00
        time = float(seconds)
        print(f'TIME: {int(time)}')
        loop_count += 1
        time_history = np.hstack((time_history, time))
        # z_targPy = np.cos(2 * time) - 0.5
        z_targPy = np.hstack((z_targPy, 1.0))
        z_targPy_history = np.hstack((z_targPy_history, z_targPy))
        # z_actual.append(np.array([1.0]))


        drone_state_history.append(drone.X)
        drone.thrust = controller.thrust_control(z_targPy, dt)

        # z_actual = [drone.z]
        # print(f'drone z:{z_actual}')
        # # z_actual = np.hstack((z_target, drone.z))

        # z_target_lin = np.cos(2 * time) - 0.5
        # z_target_lin_list.append(z_target_lin)

        # z_target_py = np.cos(2 * time) - 0.5
        # z_target_py_list.append(z_target_py)

        # print(f'z_target: {z_target}')
        # u = controller.thrust_control(z_target,z_actual)
        u = 1.0
        drone.thrust = u
        drone.advance_state(dt)
        history.append(drone.X)

        if time > 10.00:
            # dt = 0.002
            print(f'z_target: {z_targPy}')
            # print(f'End time: {time}')
            # t = np.linspace(0.0, time, int(time/dt))
            # z_targ_gnd = 0.5 * np.cos(2 * t) - 0.5

            # time_py = np.linspace(0.0, time, loop_count)
            print(f'time history:{time_history}')
            # time_py = time_history
            # z_targ_py = np.cos(2 * time_py) - 0.5

            plt.figure(figsize=(8,4))
            row, column = 1, 2
            # plt.subplot(row, column, 1)
            # plt.plot(t, z_targ_gnd)
            # plt.title("Lin time, z_target ground")

            plt.subplot(row, column, 2)
            plt.plot(time_history, z_targPy_history, color = "red")
            plt.title("Py time, z target Py")
            plt.show()

            print(f'dt:{dt}')
            print(f'dt as time / frames: {time/loop_count}')
            print(f'# frame: {loop_count}')
            print(f'# frame: {time/dt}')
            print(f'# frames: {fps * time}')
            print(f'Average fps: {loop_count/time}')
            print(f'Time divided by frame: {time/loop_count}')
            # print(f'z actual list: {z_actual_list}')
            # generate plots
            # print(f'fps is: {fps}')
            # print(f'Length of t is: {len(t)}')
            # print(f't:\n {t}')
            # print(f'Length of z_target_list: {len(z_target_list)}')
            # print(f'z target list: \n{z_target_list}')
            # plt.plot(t, z_target_list)
            # plt.show()
            dt = time / fps

            # print(f'Time History: \n {time_history[:,0:5]}')
            # print(f'Time History List: \n {time_history_list}')
            # frequency_pymunk_avg = total_dts / time
            # dt_pymunk_avg = 1.0 / frequency_pymunk_avg
            # print(f'dt pymunk:{dt_pymunk}')
            # plt.plot(t, np.full((int(fps+1),),1))

            # plt.show()
            # print(f'frequency pymunk avg: {frequency_pymunk_avg}')
            # print(f'dt pymunk avg: {dt_pymunk_avg}')

            # print(f'z actual: {type(z_actual)}, {z_actual}')
            # print(f'z actual shape: {z_actual.shape}')
            #
            # print(f'z path: {type(z_path)}, {z_path}')
            # print(f't: {t}')
            # plotting.compare_planned_to_actual(z_actual, z_path, t)
            sys.exit(0)

        pygame.display.flip()

if __name__ == '__main__':
    sys.exit(main())