
# https://github.com/viblo/pymunk/blob/master/examples/breakout.py
import pygame
import pymunk
from pygame.color import *
from pygame.locals import *

import math, sys, random

import numpy as np

import matplotlib.pyplot as plt

from pymunk.pygame_util import DrawOptions

# screen = pygame.display.set_mode((128, 128))

width = 600
height = 600

class CoaxialCopter:
    def __init__(self,
                 name,
                 space,
                 K_m = 0.1,
                 K_f = 0.1,
                 mass = 0.5,
                 I_z = 0.2,
                 ):
        self.K_m = K_m
        self.K_f = K_f
        self.mass = mass
        self.I_z = I_z

        self.g = -space.gravity[1]
        print(f'The gravity is: {self.g}')
        self.omega_1 = 0.0
        self.omega_2 = 0.0
        self.name = name

        self.z = 0.0
        self.z_dot = 0.0
        self.psi = 0.0
        self.psi_dot = 0.0

        # self.X = np.array([0.0, 0.0, 0.0, 0.0])

        # Create drone body
        self.shape = pymunk.Poly.create_box(None, size=(50, 10))
        self.moment = pymunk.moment_for_poly(self.mass, self.shape.get_vertices())
        self.body = pymunk.Body(self.mass, self.moment)
        self.shape.body = self.body
        # leg1 = pymunk.Segment(self.body, (-20, -30), (-10, 0), 3)  # contact point 2
        # leg2 = pymunk.Segment(self.body, (20, -30), (10, 0), 3)

        self.position = (300, 174)  # Position drone starts in space
        self.shape.body.position = self.position
        # space.add(self.shape, self.body, leg1, leg2)
        space.add(self.shape, self.body)


    def set_rotors_angular_velocities(self, linear_acc, angular_acc):
        term_1 = self.mass * (-linear_acc + self.g) / (2 * self.K_f)
        term_1 = abs(term_1)  # This avoids a 'math domain' error when linear_acc > gravity
        term_2 = self.I_z * angular_acc / (2 * self.K_m)
        omega_1 = math.sqrt(term_1 + term_2)
        omega_2 = math.sqrt(term_1 - term_2)

        self.omega_1 = -omega_1
        self.omega_2 = omega_2
        return self.omega_1, self.omega_2

    @property
    def z_dot_dot(self):
        f_1 = self.K_f * self.omega_1**2
        f_2 = self.K_f * self.omega_2**2
        # print('Kf, f_1, f_2, inside z_dot_dot function: ', self.K_f, f_1, f_2)
        # print('f1, f2, inside function: ', f_1, f_2)
        # print('Omega 1, Omega 2, inside function: ', self.omega_1, self.omega_2)

        f_g = self.mass * self.g
        # print('fg: ', f_g)
        f_total = -f_1 - f_2 + f_g
        # print('f_total: ', f_total)
        vertical_acceleration = f_total / self.mass
        return vertical_acceleration

    @property
    def psi_dot_dot(self):
        cw_torque = self.K_m * self.omega_1 **2
        ccw_torque = self.K_m * self.omega_2 **2
        net_torque = cw_torque - ccw_torque
        angular_acc = net_torque / self.I_z
        return angular_acc

    def advance_state(self, screen, font, dt):
        """Advances the state of the drone forward by dt seconds"""
        #
        # TODO
        #  Implement this method! Your implementation may look
        #  VERY similar to the uncontrolled version of this function.
        z_dot_dot = self.z_dot_dot

        delta_z_dot = z_dot_dot * dt
        self.z_dot += delta_z_dot

        delta_z = self.z_dot * dt
        self.z += delta_z
        # print('z: ', self.z)

        # update psi state (see method "psi_dot_dot" implemented before)
        psi_dot_dot = self.psi_dot_dot

        delta_psi_dot = psi_dot_dot * dt
        self.psi_dot += delta_psi_dot

        # technically we should restrict psi so it's between
        # -pi and pi, but we're not going to worry about that now
        delta_psi = self.psi_dot * dt
        self.psi += delta_psi

        # Calculate Acceleration from Drone's POV
        dynamic_text = z_dot_dot
        # screen.blit(font.render(card_text, True, (255, 255, 255)), (10, 95))
        # screen.blit(font.render(str(round(dynamic_text, 2)), True, (155, 190, 230)), (185, 75))

        # self.shape.body.position = self.position[0], self.position[1] - self.z

        # print(f'z position: {self.z:,.2f}')

    def advance_state_uncontrolled(self, dt):
        print('Advance State Uncontrolled.')
        z_dot_dot = 9.8
        delta_z_dot = z_dot_dot * dt
        self.z_dot = self.z_dot + delta_z_dot
        # self.z_dot = round(self.z_dot, 2)
        print(f'self.z_dot: {self.z_dot}')

        delta_z = self.z_dot * dt
        self.z = self.z + delta_z
        # self.z = round(self.z, 2)
        print(f'The z position is: {self.z}')

        # self.shape.body.apply_force_at_local_point((0, -self.z), (0, 0))

        # self.shape.body.position = self.position[0], self.position[1] - self.z

class Experiment:
    def __init__(self):
    # Does nothing, just a place to log my experiments and notes:

    # 1. Choose desired acceleration, use set_rotors to calculate required: Omega 1, Omega 1
    # 2. Use z_dot_dot to calculate acceleration
    # 3. Use Advance State to calculate z_dot, z_position, psi_dot, psi_position

    # Experiment: What happens when I delegate the drone's position to the advance_state?
    #  a. Delegate drone control first to human me. DONE.
    #       i. Turn off gravity. DONE.
    #  b. Give control to advance state. DONE.

    # 4/30
    # Experiment: If I put the drone at the top of the screen, does it fall down at roughly gravity?
    # How do I put the drone at the top of the screen?
    #         self.shape.body.position = self.position[0], self.position[1] - self.z
    # How do I fly this drone? If press a key up, and set the velocity to 2.0 m/s sec, what happens?

    # 5/1
    # If I set the acc to 20.0 m/s^2, does it go up on keypress? IT WORKS! Goes up slowly at the end!
    # If I set acc to -20.0 m/s^2, does it go down on keypress? It goes up very fast. Why the difference?

    # 5/2
    # Experiment:
    # How do I let the drone fall? I guess the max I can set the desired acc to is -9.8 m/s^2
    # Can I increment the thrust by +5 m/s^2 per 'UP' button press?
    # 5/5
    # Which way does pymunk define gravity? Is up negative or positive?
    # Answer: Negative is down. Defined as positive in my Coaxial class
    # 5/6
    # I would like to be able to increment and decrement linear_acc by +/- 5 m/s^2. How?
    # Want to show Omega_2 value, show drone_acc external and internal.
    # Want to clean up card code so all cards get updated in a single function.
    # 5/7
    # Extend orange card down by 1 row.

    # 5/12
    # Question: If I turn pymunk gravity on and have gravity on in Advanced state controller, what happens?
    # Answer: Acc_external increases in either direction until the drone hits the ground.
    # Question? What happens if pass pymunk gravity to Advance State?
    # How would I do that?
    # 1. Take the parameter for gravity, space.gravity, and add it to the parameter list of Advance State(space.gravity)
    #   - Advance State gets z_dot_dot from Coaxial.z_dot_dot
    #   - I just changed the self.g = space.gravity[1] ; 9.8

    # Question: What happens if I put Uncontrolled State back in? Our my sensor readings still accurate?

    # 5/13
    # Question: Why doesn't the drone stop on the ground?

    # Hypothesis 1: Frames per second is set to low?
    # Changed both the FPS and Space.step to 560. No change.

    # Hypo 2: The advanced state is controlling the drone's position and doesn't observe the collision._
    # Experiment: Remove the position control from the Advance_state method

    # Hypo 3: The collision needs to be attached in the advanced_state method.

    # 5/15/19
    # Adding Cosine function from class
    # 5/16/19
    # Same as yesterday

    # 5/22/19
    # I think I just need to play with the Coaxial.shape.body
    # CoaxialDrone.shape.body.apply_force_at_local_point((0, 10), (0, 0))

    # 6/3/19
    # Remembered: Every loop erases the screen. So every HUD needs to be updated every loop.
    #
    # 6/8/19
    # Working on HUD class and layout
    #
    # 6/17/19
    # Updating HUD and Card classes



        pass

class Ground:
    def __init__(self, space, z_position):
        self.groundHeight = z_position
        self.body = pymunk.Body(0, 0, body_type=pymunk.Body.STATIC)
        self.shape = pymunk.Poly.create_box(self.body, (width, 50))
        # self.shape.body.position = (width//2, 144.129925)
        self.shape.body.position = (width//2, self.groundHeight)
        space.add(self.shape, self.body)

class Card(pygame.sprite.Sprite):
    # For color codes | http://www.tayloredmktg.com/rgb/
    def __init__(self, screen, font,
                 card_text, card_text_x, card_text_y, card_width, card_height,
                 card_color = THECOLORS["royalblue"], color_text = THECOLORS["white"]):
        super(Card, self).__init__()

        offset_x, offset_y = 5, 5  # padding for cards
        self.surf = pygame.Surface((card_width + offset_x, card_height + offset_y))
        self.surf.fill(card_color)
        self.rect = self.surf.get_rect()
        self.card_text = card_text
        self.screen = screen
        # dynamic_text_trunc = float(dynamic_text)
        # dynamic_text_trunc = round(dynamic_text_trunc, 2)
        # dynamic_text = str(dynamic_text_trunc)
        screen.blit(self.surf, (card_text_x - offset_x, card_text_y - offset_y))
        screen.blit(font.render(card_text, True, (color_text)), (card_text_x, card_text_y))
        # screen.blit(font.render(dynamic_text, True, (color_text)), (dynamic_text_x, dynamic_text_y))

    def id(self):
        print(self.card_text)


def draw_collision(arbiter, space, data):
    # circle(Surface, color, pos, radius, width=0) -> Recty
    # print('Collision - ouch!')
    #  to_pygame = lambda p: (int(p.x), int(-p.y+600)) ??? from PymunkCollisions.py, convert from pygame to pymunk
    print('COLLISIONS!')

    for c in arbiter.contact_point_set.points:
        # print(c)
        # print('c.point_a: ', c.point_a)
        r = max(3, abs(c.distance * 15))
        r = int(r)
        p = tuple(map(int, c.point_a))
        # print(f'P is p: {p}')
        pygame.draw.circle(data["surface"], THECOLORS["red"], p, r, 0)

def draw_plot(x, y):
    print(f't_history: {x}')
    print(f'Z_history: {y}')

    plt.plot(x, y)
    plt.xlabel('x - axis')
    plt.ylabel('y - axis')
    plt.title('Position over Time')
    plt.gca().invert_yaxis()
    plt.show()

class Rounded:
    def __init__(self, numbers):
        self.temp = list()
        for i, j in enumerate(numbers):
            self.temp.append(round(j,2))
    def value(self):
        return self.temp

class HUD:
    def __init__(self, screen, font):
        self.screen = screen
        self.font = font
        # screen.blit(font.render("text", True, (THECOLORS['white'])), (300, 300))
        self.hudList_static = ['T+', 'Velocity', 'Acc', 'Position', 'Loop', 'dt', 'Omega_1', "Omega_2"]
        self.hudList_dynamic = list()

    def display(self, seconds_float,  velocity, acceleration, position, loop, dt, omega_1, omega_2):
        # numberItems = len(self.hudList)
        screen_origin_X = 10
        screen_origin_Y = 15

        rowHeight = 20
        columnWidth = 100
        y_vertical = 0

        # Blue column headers with background                     (Xorg Yorg Width, Height)
        labelCard_Column_1 = Card(self.screen, self.font, "Pymunk", 100, 10, 80, 20)
        labelCard_Column_2 = Card(self.screen, self.font, "Drone", 185, 10, 80, 20, THECOLORS['cornflowerblue'])

        # Orange Column background:            "TEXT"(width) (length)  (background color)
        time_card = Card(self.screen, self.font, "", 10, 35, 85, 165, THECOLORS['goldenrod'])

        for item in self.hudList_static:
            # print(item)
            y_vertical += rowHeight
            self.screen.blit(self.font.render(item, True, (THECOLORS['white'])), (screen_origin_X, screen_origin_Y + y_vertical))
        list = [seconds_float, velocity[1], acceleration, position[1], loop, dt, omega_1, omega_2]

        self.hudList_dynamic = Rounded(list)
        self.hudList_dynamic = self.hudList_dynamic.value()

        y_vertical = 0                                           # reset to zero to start at top row for next list.

        for data in self.hudList_dynamic:
            # print(data)
            y_vertical += rowHeight
            self.screen.blit(self.font.render(str(data), True, (THECOLORS['white'])),(screen_origin_X + columnWidth, screen_origin_Y + y_vertical))

        y_vertical = 0                                           # reset to zero to start at top row for next list.
        columnWidth += 80
        for data in self.hudList_dynamic:
            # print(data)
            y_vertical += rowHeight
            self.screen.blit(self.font.render(str(data), True, (THECOLORS['white'])),(screen_origin_X + columnWidth, screen_origin_Y + y_vertical))


    # What does it do?
    # Description: Given a list, it is a mini machine that "paves" the HUD out on the screen for every loop.
    # Use Case: Add Angular Acc.
    #
    # - It can count the amount of items in the list.
    # - It can display the text in order from top to bottom starting from a given point origin.
    #
    # What does it know?
    # What the labels are. The text of the labels. Where the labels should go on the screen, how high they are.
    # - The list of labels.
    # - The text of the label.
    # - The total height of the graphics.
    # - The desired color of the background.
    # - The desired color of the text.
    # - The desired position of the background.
    # - The desired position of the origin of the graphics.
    # - How many items are in the list.
    # - How to display text:
    #     screen.blit(font.render(dynamic_text, True, (color_text)), (dynamic_text_x, dynamic_text_y))

def displayHud(screen, seconds_float, CoaxialDrone, acceleration, loop, dt, dronePosition):

    # print('HUD')
    color_text = (255, 255, 255)  # white, foreground text of labels
    font = pygame.font.SysFont('Consolas', 25)

    labelCard_Column_1 = Card(screen, font, "Pymunk", 100, 10, 80, 20, str(seconds_float), 40, 10)
    labelCard_Column_2 = Card(screen, font, "Drone", 185, 10, 80, 20, str(seconds_float), 40, 10,
                              THECOLORS['cornflowerblue'])

    # Orange background:                     (width)(length)
    time_card = Card(screen, font, "T+", 10, 35, 85, 165, str(seconds_float), 120, 35, THECOLORS['goldenrod'])

    # Calculate World Velocity in the y
    dynamic_text_velocity = CoaxialDrone.shape.body.velocity[1]
    card_text = 'Velocity:'
    screen.blit(font.render(card_text, True, (255, 255, 255)), (10, 55))
    screen.blit(font.render(str(round(dynamic_text_velocity, 2)), True, (color_text)), (115, 55))

    card_text = "Acc"
    screen.blit(font.render(card_text, True, (255, 255, 255)), (10, 75))
    screen.blit(font.render(str(round(acceleration, 2)), True, (color_text)), (115, 75))
    # accelerationCard = Card(screen, font, "Acc", 10,75, 143,20, dynamic_text, 95,75,THECOLORS['royalblue'])

    card_text = "loop"
    screen.blit(font.render(card_text, True, (255, 255, 255)), (10, 95))
    screen.blit(font.render(str(round(loop, 2)), True, (color_text)), (115, 95))

    card_text = "dt"
    screen.blit(font.render(card_text, True, (255, 255, 255)), (10, 115))
    screen.blit(font.render(str(round(dt, 2)), True, (color_text)), (115, 115))

    card_text = "Omega_1: "
    screen.blit(font.render(card_text, True, (255, 255, 255)), (10, 135))
    screen.blit(font.render(str(round(CoaxialDrone.omega_1, 2)), True, (color_text)), (175, 135))

    card_text = "Omega_2: "
    screen.blit(font.render(card_text, True, (255, 255, 255)), (10, 155))
    screen.blit(font.render(str(round(CoaxialDrone.omega_2, 2)), True, (color_text)), (175, 155))

    dynamic_text_pos_y = dronePosition[1]  # Z position Only
    card_text = "Position:"
    screen.blit(font.render(card_text, True, (255, 255, 255)), (10, 175))
    screen.blit(font.render(str(round(dynamic_text_pos_y, 2)), True, (color_text)), (115, 175))

class Test:
    def __init__(self, screen, font, first, last):
        self.first = first
        self.last = last
        self.screen = screen
        self.font = font
        for i in range(9):
            print(i)
    def math(self, screen, font):
        card_text = "Acc"
        acceleration = 10080.0088
        color_text = 255, 255, 255
        screen.blit(font.render(card_text, True, (255, 255, 255)), (10, 75))
        screen.blit(font.render(str(round(acceleration, 2)), True, (color_text)), (115, 75))

        return self.first + self.last

def main():

    pygame.init()

    screen = pygame.display.set_mode((width, height))
    font = pygame.font.SysFont('Consolas', 25)

    # test_1 = Test(screen, font, 'alex', 'Zan')

    pygame.display.set_caption("The ball drops")

    # Clock
    start_ticks = pygame.time.get_ticks()  # starter ticks
    clock = pygame.time.Clock()
    pygame.time.set_timer(pygame.USEREVENT, 1000)

    draw_options = DrawOptions(screen)

    space = pymunk.Space()
    space.gravity = 0, -9.800

    # Calculate Time in Seconds
    seconds = (pygame.time.get_ticks() - start_ticks) / 1000
    seconds_float = float(seconds)
    droneHUD = HUD(screen, font)

    x = random.randint(120, 380)
    ground = Ground(space, 144)   # attribute is the height of the ground

    t_history = []
    z_history = []

    # Add collision handler to object:
    ch = space.add_collision_handler(0, 0)
    ch.data["surface"] = screen
    ch.post_solve = draw_collision

    myName = 'alx'
    linear_acc_desired = 0.0
    psi_acc_desired = 0.0

    CoaxialDrone = CoaxialCopter('Alex', space)
    stable_omega_1, stable_omega_2 = CoaxialDrone.set_rotors_angular_velocities(linear_acc_desired, psi_acc_desired)
    print(f'\n omega_1: {stable_omega_1:0.4f} , omega_2: {stable_omega_2:0.3f}')

    vertical_acc = CoaxialDrone.z_dot_dot
    print(f'Given linear_acc = {linear_acc_desired}, and psi_acc = {psi_acc_desired} of a drone with a mass of {CoaxialDrone.mass} and gravity of {CoaxialDrone.g} results \n in {stable_omega_1:0.3f} , {stable_omega_2:0.3f} -> vert_acc: {vertical_acc:0.4f}')

    CoaxialDrone.omega_1 = stable_omega_1 * math.sqrt(1.1)
    CoaxialDrone.omega_2 = stable_omega_2 * math.sqrt(0.9)
    print('Omegas:')
    print(f'{CoaxialDrone.omega_1:0.3f} {CoaxialDrone.omega_2:0.3f}, vertical acc: {CoaxialDrone.z_dot_dot:0.3f}')

    ang_acc = CoaxialDrone.psi_dot_dot
    print(f'angular acc {ang_acc}')

    loop = 0

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit(0)
                
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                sys.exit(0)
        # """
        # This is the code that applies force to the body of the drone
        # """
        # if ball.shape.body.position.y < 200:
        #     CoaxialDrone.shape.body.apply_force_at_local_point((0, -400), (0, 0))

            #     """  (0, 400) means apply 400 units of force in the direction of y (0,0) is the co-ordinate to apply the force too"""

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_UP:
                        linear_acc_desired = linear_acc_desired - 0.5
                        stable_omega_1,stable_omega_2 = CoaxialDrone.set_rotors_angular_velocities(linear_acc_desired, 0.0)

                        # CoaxialDrone.shape.body.apply_force_at_local_point((0, my_vertical_acc), (0, 0))
                        CoaxialDrone.shape.body.apply_force_at_local_point((0, 100), (0, 0))

                        pygame.display.set_caption("His monk flies")


            elif event.type == pygame.KEYDOWN and event.key == pygame.K_RIGHT:
                linear_acc_desired = 0.0
                # current_omega_1, current_omega_2 = CoaxialDrone.set_rotors_angular_velocities(4.95, 0.0)
                linear_acc_desired = 0.0
                stable_omega_1, stable_omega_2 = CoaxialDrone.set_rotors_angular_velocities(linear_acc_desired, 0.0)

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_LEFT:
                draw_plot(t_history, z_history)

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_DOWN:
                linear_acc_desired = linear_acc_desired + 0.5
                # current_omega_1, current_omega_2 = CoaxialDrone.set_rotors_angular_velocities(4.95, 0.0)
                stable_omega_1, stable_omega_2 = CoaxialDrone.set_rotors_angular_velocities(linear_acc_desired, 0.0)

                # pygame.display.set_caption("His monk drop")

        screen.fill((0, 0, 0))
        space.debug_draw(draw_options)

        space.step(1/60.0)
        FPS = 60
        clock.tick(FPS)

        # Calculate Time in Seconds
        seconds = (pygame.time.get_ticks() - start_ticks) / 1000
        seconds_float = float(seconds)

        # Calculate dt, dt = 0.02 in class lessons
        loop += 1                      # loop is number of frames
        dt = seconds_float / loop      # The amount of time it takes for each frame

        # Detect Drone's Acceleration Externally
        acceleration = CoaxialDrone.shape.body.velocity[1] / seconds

        # Detect Drone's Velocity Externally
        velocity = CoaxialDrone.shape.body.velocity

        # Detect Drone's position
        position = CoaxialDrone.shape.body.position

        # Get omega values
        omega_1, omega_2 = CoaxialDrone.omega_1, CoaxialDrone.omega_2
        # print(omega_1, omega_2)

        # Display HUD:
        # displayHud(screen, seconds_float, CoaxialDrone, acceleration, loop, dt, dronePosition)

        # stable_omega_1, stable_omega_2 = CoaxialDrone.set_rotors_angular_velocities(0.0,0.0)
        # F = Kf * omega^2
        # F = 0.1 * 4.95 ^2

        # print(stable_omega_1, stable_omega_2)
        # print(f'The vertical acceleration is: {CoaxialDrone.z_dot_dot}')

        # F = ma -mg
        # F = m(a-g)
        #
        # a = 0.0
        # m = 0.5
        #
        # a = delta v / delta time

        target_z = np.cos(seconds_float) - 1
        target_z_dot_dot = -np.cos(seconds_float)

        # CoaxialDrone.advance_state_uncontrolled(dt)

        CoaxialDrone.set_rotors_angular_velocities(target_z_dot_dot, 0.0)

        CoaxialDrone.advance_state(screen, font, dt)
        print(f'Force is: {CoaxialDrone.z_dot}')

        # CoaxialDrone.shape.body.position = CoaxialDrone.position[0], CoaxialDrone.position[1] - CoaxialDrone.z
        CoaxialDrone.shape.body.apply_force_at_local_point((0, -CoaxialDrone.z_dot), (0, 0))

        t_history.append(seconds_float)
        z_history.append(CoaxialDrone.z)

        droneHUD.display(seconds_float, velocity, acceleration, position, loop, dt, omega_1, omega_2)

        pygame.display.update()

if __name__ == '__main__':
    sys.exit(main())
