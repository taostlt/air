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

import pymunk

class Test:
    def clear():
        screen.fill((255, 255, 255))

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

class Ground:
        def __init__(self, space, z_position):
            self.groundHeight = z_position
            self.body = pymunk.Body(0, 0, body_type=pymunk.Body.STATIC)
            width = 600
            self.shape = pymunk.Poly.create_box(self.body, (width, 50))
            # self.shape.body.position = (width//2, 144.129925)
            self.shape.body.position = (width // 2, self.groundHeight)
            space.add(self.shape, self.body)
            # print("I JUST CREATED GROUND.")

class CoaxialCopter:
    X: object

    def __init__(self,
                 name, space,
                 gravity_ref,
                 k_f = 0.1,
                 I_x = 0.1,
                 m = 1.0,
                 l = 0.5,
                 ):

        self.name = name
        self.k_f = k_f
        self.I_x = I_x
        self.l = l
        self.mass = m
        self.omega_1 = 0.0
        self.omega_2 = 0.0
        self.g = gravity_ref

                        #  z,   y,   phi, z_dot, y_dot, phi_dot
        self.X = np.array([0.0, 0.0, 0.0, 0.0,   0.0,   0.0])

        # Create drone body
        self.shape = pymunk.Poly.create_box(None, size=(50, 10))
        self.moment = pymunk.moment_for_poly(self.mass, self.shape.get_vertices())
        self.body = pymunk.Body(self.mass, self.moment)
        self.shape.body = self.body
        # leg1 = pymunk.Segment(self.body, (-20, -30), (-10, 0), 3)  # contact point 2
        # leg2 = pymunk.Segment(self.body, (20, -30), (10, 0), 3)    # contact point 1

        # Position where the drone starts in space
        # self.position = (300,0)                      # For benchmarking, start at h = 0.
        self.position = (0, 174)                 # Position = level to ground (h=174)
        # self.position = (300, 300)                   # Position = center of frame.
        self.shape.body.position = self.position
        # space.add(self.shape, self.body, leg1, leg2)
        space.add(self.shape, self.body)

    def set_rotors_angular_velocities(self, linear_acc):
        # term_1 = self.mass * (-linear_acc + self.g) / (2 * self.k_f)
        # term_1 = abs(term_1)     # This avoids a 'math domain' error when linear_acc > gravity means faster then falling.
        #                          # which should never happen unless props reverse to provide thrust downwards.
        # term_2 = self.I_z * angular_acc / (2 * self.K_m)
        # omega_1 = math.sqrt(term_1 + term_2)
        # omega_2 = math.sqrt(term_1 - term_2)

        omega = math.sqrt(self.mass * (-linear_acc + self.g) / (2 * self.k_f))

        self.omega_1 = omega
        self.omega_2 = omega
        return self.omega_1, self.omega_2

    def get_thrust_and_moment(self):
        f1 = self.k_f * self.omega_1 **2
        f2 = self.k_f * self.omega_2 **2
        c = f1 + f2
        M_x = (f1 - f2) * self.l
        return c, M_x

    @property
    def z_dot_dot(self):
        c, M_x = self.get_thrust_and_moment()
        phi = self.X[2]
        a_z = self.g - c * math.cos(phi) / self.mass
        return a_z

    @property
    def y_dot_dot(self):
        c, M_x = self.get_thrust_and_moment()
        phi = self.X[2]
        a_y = c * math.sin(phi) / self.mass
        return a_y

    # @property
    # def z_dot_dot(self):                     # This is z_dot_dot_total for system, including g.
    #     f_1 = self.k_f * self.omega_1**2
    #     f_2 = self.k_f * self.omega_2**2
    #     f_g = self.mass * self.g
    #     print('fg: ', f_g)
    #     f_total = (f_1 + f_2) - f_g
    #     print('f_total: ', f_total)
    #     vertical_acceleration = f_total / self.mass
    #     print(f'Vertical acc is: {vertical_acceleration}')
    #     return vertical_acceleration

    @property
    def phi_dot_dot(self):
        # cw_torque = self.K_m * self.omega_1 **2
        # ccw_torque = self.K_m * self.omega_2 **2
        # net_torque = cw_torque - ccw_torque
        # angular_acc = net_torque / self.I_z
        # return angular_acc
        c, M_x = self.get_thrust_and_moment()
        angular_acc = M_x / self.I_x
        return angular_acc

    def advance_state(self, dt):
        """Advances the state of the drone forward by dt seconds"""
        #
        # TODO
        #  Implement this method! Your implementation may look
        #  VERY similar to the uncontrolled version of this function.

        X_dot = np.array([
            self.X[3],
            self.X[4],
            self.X[5],
            self.z_dot_dot,
            self.y_dot_dot,
            self.phi_dot_dot])

        self.X = self.X + (X_dot * dt)
        return self.X


    # def advance_state_uncontrolled(self, dt):
    #     print('Advance State Uncontrolled.')
    #     z_dot_dot = self.g
    #     delta_z_dot = z_dot_dot * dt
    #     self.z_dot = self.z_dot + delta_z_dot
    #     # self.z_dot = round(self.z_dot, 2)
    #     print(f'self.z_dot: {self.z_dot}')
    #
    #     delta_z = self.z_dot * dt
    #     self.z = self.z + delta_z
    #     # self.z = round(self.z, 2)
    #     print(f'The z position is: {self.z}')

    def advance_state_uncontrolled(self, dt):
        """Advances the state of the drone by dt seconds.
        Note that this method assumes zero rotational speed
        for both propellers."""

        X_dot = np.array([
            self.X[3],
            self.X[4],
            self.X[5],
            self.g,
            0.0,
            0.0])

        # Change in state will be
        self.X = self.X + X_dot * dt
        return self.X

def main():
    pygame.init()
    pygame.display.set_caption("This is just a test")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont('Consolas', 25)
    size = width, height = 600, 600
    screen = pygame.display.set_mode(size)
    black = (0,0,0)
    space = pymunk.Space()
    space.gravity = 0, 9.800  # Pymunk defines gravity incorrectly by default,

    # space.gravity = 0, 9.8     # as seen here, it defines 'down' as a negative acceleration (see Slide Pin Joint demo)
    j = -1  # j is a unit vector to indicate direction (ie. Khan academy "normal force elevator")
    gravity_ref = space.gravity[1]

    image = pygame.image.load(r'/home/frank/Pictures/Amazon_Response_By_Seller.png')
    # mySwitch = Switch()
    name = 'Alex'
    dt = 0.002  # from homework

    # myCard = Card(screen, font,str(space.step(1/60.0)), 300,300, 20,20, THECOLORS["royalblue"],THECOLORS["white"])

    loop = 0
    # loop_count = np.array([0])
    start_ticks = pygame.time.get_ticks()                   # starter ticks
    loop_count = 0
    time = 0
    #                               #  loop, time, z,   z_dot_dot, z_path_actual
    # current_state         = np.array([0.0, 0.0,  0.0, 0.0,       0.0], dtype=np.float)
    # current_state_history = np.array([0.0, 0.0,  0.0, 0.0,       0.0], dtype=np.float)

    # a = np.array([[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12]])
    # b = a[:2, 1:3]    # [2, 3 ]
    # print(b)          # [6, 7 ]
    # Start counting at zero, up to 2, up to 3 but do not include upper bound.

    # This all works as expected:
 #    a = np.array([0,0])
 #    b = np.array([1,1])
 #    c = np.vstack((a,b))
 #    print(f'C: \n {c}')
 #    c = np.vstack((c,b))
 #    print(f'C: \n {c}')
 # #---------------------
 #    # x = [1,2,60]
 #    y = np.array([4,5,10])
 #    plt.plot(y, color="red")
 #    plt.show()

    z_path_target_history = np.array([0.0])

    total_time = 3.0
    dt = 0.02
    drone2 = CoaxialCopter("2", space, gravity_ref)
    t = np.linspace(0.0, total_time, int(total_time/ dt))
    drone2_state_history = drone2.X
    drone2_path_target = 0.5 * np.cos(2 * t) - 0.5
    drone2_dot_dot_path = -2.0 * np.cos(2 * t)

    for i in range(t.shape[0] - 1):
        drone2.set_rotors_angular_velocities(drone2_dot_dot_path[i])
        drone2_state = drone2.advance_state(dt)
        drone2_state_history = np.vstack((drone2_state_history, drone2_state))

    # plt.plot(t, drone2_path_target, label="actual state history", marker='o', color="red")
    # plt.title("Drone '2' planned path, lin space")
    # plt.plot(t, drone2_state_history[:,0],color="blue")
    # plt.legend('target lin', 'actual lin')
    # plt.show()

    drone = CoaxialCopter(name, space, gravity_ref)
    drone_state_history = drone.X
    time_state = np.array([0.0])
    time_state_history = np.array([0.0])
    dt_state = np.array([0.0])
    dt_state_history = np.array([0.0])
    dt_average = np.array([0.0])
    dt_average_history = np.array([0.0])

    #---------------------
    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                sys.exit(0)
            # elif event.type == KEYDOWN and event.key == K_ESCAPE:
            #     sys.exit(0)
            # elif event.type == pygame.KEYDOWN and event.key == pygame.K_UP:
            #     print('key UP')
            #     mySwitch.light_on(screen, image)
            # elif event.type == pygame.KEYDOWN and event.key == pygame.K_DOWN:
            #     print('key DOWN')
            #     mySwitch.light_off(screen)
            # elif event.type == pygame.KEYDOWN and event.key == pygame.K_RIGHT:
            #     print('key RIGHT')
            #     myCard = Card(screen, font, str('Right'), 300, 300, 20, 20, THECOLORS["royalblue"],
            #                   THECOLORS["white"])
            # elif event.type == pygame.KEYDOWN and event.key == pygame.K_LEFT:
            #     print('key LEFT')
            #     myCard = Card(screen, font, str('Left'), 300, 300, 20, 20, THECOLORS["royalblue"],
            #                   THECOLORS["white"])

        # space.step(1 / 60.0)
        clock.tick(500)

        print(f'Start ticks: {start_ticks}')
        print(f'Pygame tick difference:{((pygame.time.get_ticks()-start_ticks) / 1000.00)}')

        # Calculate Time in Seconds. DONE.

        # Increment loop data. DONE.
        # Increment time data. DONE.
        # Store loop data & time data in a current np.array. DONE.
        # Add current data on top of old data. DONE.
        # Plot column of loop data. DONE.
        # Plot column of time data. DONE.

        seconds = (pygame.time.get_ticks() - start_ticks) / 1000.00
        time = float(seconds)
        # print(f'TIME: {time}')
        loop_count += 1
        # print(f'Loop Count: {np.round(loop_count/60)}')
        # print(f't: {t.shape}')

        z_path_target = 0.5 * np.cos(2 * time) - 0.5
        z_dot_dot = -2.0 * np.cos(2 * time)
        print(f'z_dot_dot: {z_dot_dot}')

      #  z_path_target_history = np.vstack((z_path_target_history, z_path_target))
        # current_state[0] = loop_count
        # current_state[1] = time
        # current_state[2] = z_path_target
        # current_state[3] = z_dot_dot
        # current_state[4] = z_path_actual

        drone.set_rotors_angular_velocities(z_dot_dot)
        drone_state = drone.advance_state(dt)

        time_state[0] = time

        drone_state_history = np.vstack((drone_state_history, drone_state))
        time_state_history = np.vstack((time_state_history, time_state))
        print(f'Loop Count: \n {loop_count}')
        dt_state = time_state_history[loop_count] - time_state_history[loop_count-1]
        print(f'dt state: {dt_state}')
        dt_state_history = np.vstack((dt_state_history, dt_state))

        dt_average[0] = time/loop_count         # same as 1 / (loop_count/time)
        print(f'dt average: {np.round(dt_average, 3)}')
        dt_average_history = np.vstack((dt_average_history, dt_average))

        if loop_count > 6400:
            print(f'dt state history: {dt_state_history}')
            plt.plot(dt_state_history[:,0], color="blue")
            plt.plot(dt_average_history[:,0], color="brown")
            plt.legend(["dt instances", "dt average history (total time / total cycles)"])
            # plt.legend(['planned path', 'executed path'], fontsize=18)
            plt.title("dt over time")
            plt.show()
            print('AIR')
            sys.exit(0)

        # if time > 2.9999:
        #     z_position_actual = drone_state_history[:,0]
        #     # print(f'Z POSITION ACTUAL: \n {z_position_actual}')
        #     # plt.plot(time_state_history, z_position_actual, label = "z position actual", color ="red")
        #     # plt.plot(time_state_history, time_state_history, label="time state history", color = "black")
        #     plt.title('z path actual')
        #     # plt.legend()
        #     # plt.show()
        #
        #     # plt.figure(figsize=(13, 4))
        #     # row = 1
        #     # column = 1
        #     # plt.subplot(row, column, 1)     # (row, column, index)
        #     # plt.plot(time, current_state_history[:,0], color="red")
        #     # # plt.plot(s, color="blue")
        #     # plt.plot(current_state_history[:,1],current_state_history[:,0], color="red")
        #
        #     # plt.plot(current_state_history, color="orange")
        #     # plt.plot(t, current_state_history[1:,0],current_state_history[1:,1] ,color="blue")
        #     sys.exit(0)

        pygame.display.flip()

if __name__ == '__main__':
    sys.exit(main())