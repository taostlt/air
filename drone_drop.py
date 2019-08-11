# https://github.com/viblo/pymunk/blob/master/examples/breakout.py
import pygame
import pymunk
from pygame.color import *
from pygame.locals import *

import math, sys, random

import numpy as np

import matplotlib.pyplot as plt
import matplotlib.pylab as pylab # this comes from 1. Coaxial Drone Dynamics SOLUTION and may not be necessary
                                 # it is used as pylab.rcParams['figure.figszie'] = 10, 10
from pymunk.pygame_util import DrawOptions

# screen = pygame.display.set_mode((128, 128))

width = 600 # screen size
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

        self.g = -space.gravity[1]      # Gravity in pymunk also has a horizontal component.
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
        # leg2 = pymunk.Segment(self.body, (20, -30), (10, 0), 3)    # contact point 1

        # Position where the drone starts in space
        self.position = (300,0)                       # For benchmarking, start at h = 0.
        # self.position = (300, 174)                  # Initial position on the ground (h=174)
        #self.position = (300, 300)                   # Position where the drone starts in space
        self.shape.body.position = self.position
        # space.add(self.shape, self.body, leg1, leg2)
        space.add(self.shape, self.body)

    def set_rotors_angular_velocities(self, linear_acc, angular_acc):
        term_1 = self.mass * (-linear_acc + self.g) / (2 * self.K_f)
        term_1 = abs(term_1)     # This avoids a 'math domain' error when linear_acc > gravity means faster then falling.
                                 # which should never happen unless props reverse to provide thrust downwards.
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
        f_g = self.mass * self.g
        print('fg: ', f_g)
        f_total = -f_1 - f_2 + f_g
        print('f_total: ', f_total)
        vertical_acceleration = f_total / self.mass
        print(f'Vertical acc is: {vertical_acceleration}')
        return vertical_acceleration

    @property
    def psi_dot_dot(self):
        cw_torque = self.K_m * self.omega_1 **2
        ccw_torque = self.K_m * self.omega_2 **2
        net_torque = cw_torque - ccw_torque
        angular_acc = net_torque / self.I_z
        return angular_acc

    def advance_state(self, dt):
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
    # 6/22/19
    # I just want the drone to fly to 20. Not a cos height. Just 20. How do I plug that into the homework?
    # If I set z_dot_dot = t/t
    # and I set z = t**2, then I could get Acc as a straight line, starting at linspace[1.0, 10, 11] instead of zero.

    # 6/23/19
    # The dt is the distance between intervals. So two seconds, divided by 21 segments results in a dt = 0.1

    # 6/24/19
    # https://www.sharpsightlabs.com/blog/numpy-linspace/
    # This is the clearest explanation of linspace. We need 5 ticks to mark off 4 even spaces. 0, 25, 50, 75, 100.
    # The reason we need 5 ticks is because we start at zero, we place a tick to indicate the start.
    # The reason we get drift in our graph is because our sign wave doesn't zero at its troughs, cos(3) zeros but
    # cos(3.011) doesn't.

        # This equation is my attempt at creating an equation that gets the drone to hover. target_z_dot_dot = -np.cos(time)*2 - 1
        # If I switch to uncontrolled_state(dt), does my graph agree?
        # https://www.tutorialspoint.com/python/python_matplotlib.htm shows how to create linspace with Pi
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

        offset_x, offset_y = 5, 5  # padding for cards around the edges
        self.surf = pygame.Surface((card_width + offset_x, card_height + offset_y))
        self.surf.fill(card_color)
        self.rect = self.surf.get_rect()
        self.card_text = card_text
        self.screen = screen
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

    def display(self, CoaxialDrone, time,
                Fnet_truth, thrust_truth,
                velocity, acceleration, position,
                loop, dt, omega_1, omega_2,
                ang_acc_truth):
        screen_origin_X = 10       # Screen location to begin drawing columns
        screen_origin_Y = 15

        rowHeight = 20             # How high should each row be, without padding?
        columnWidth = 100          # How wide should each column be?
        y_vertical = 0             # Each row should start at the top of the column and advance downwards

        print(f'The angular acceleration is: {CoaxialDrone.psi_dot_dot}')

        label_list = ['T+', 'Fnet', 'Thrust','Velocity', 'Acc', 'Position', 'Loop', 'dt', 'Omega_1', "Omega_2", 'Ang_Acc']
        truth_list = [time, Fnet_truth, thrust_truth, velocity[1], acceleration, position[1], loop, dt, omega_1, omega_2, ang_acc_truth]
        drone_list = [time, Fnet_truth, 2000, CoaxialDrone.z_dot, CoaxialDrone.z_dot_dot, CoaxialDrone.z, loop, dt, omega_1, omega_2, CoaxialDrone.psi_dot_dot]

        # Draw Orange Column background       "TEXT" (width) (row height) [background color]
        time_card = Card(self.screen, self.font, "", 10, 35, 85, len(label_list)*rowHeight, THECOLORS['goldenrod'])

        # Draw Blue column headers with text and background     (Xorg Yorg Width, Height)
        labelCard_Column_1 = Card(self.screen, self.font, "Truth", 100, 10, 80, 20)
        labelCard_Column_2 = Card(self.screen, self.font, "Drone", 185, 10, 80, 20, THECOLORS['cornflowerblue'])

        # Description Column from label_list
        for item in label_list:
            y_vertical += rowHeight
            self.screen.blit(self.font.render(item, True, (THECOLORS['white'])), (screen_origin_X, screen_origin_Y + y_vertical))

        # Truth Column from truth_list
        y_vertical = 0                         # reset to zero to start at top row for next list.
        hudList_truth = Rounded(truth_list)    # Create an object of type Rounded, round the values.
        hudList_truth = hudList_truth.value()  # Return the new values in the new list
        for data in hudList_truth:
            # print(data)
            y_vertical += rowHeight
            self.screen.blit(self.font.render(str(data), True, (THECOLORS['white'])),(screen_origin_X + columnWidth, screen_origin_Y + y_vertical))

        # Drone Column from drone_list
        y_vertical = 0                         # Reset to zero to start at top row for next list.
        columnWidth += 80                      # Draw 80 spaces to the right of the last column.
        hudList_drone = Rounded(drone_list)    # Round off values to two places.
        hudList_drone = hudList_drone.value()  # Return the rounded value as a list
        for data in hudList_drone:
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

class Test:
    def __init__(self, screen, font, first, last):
        self.first = first
        self.last = last
        self.screen = screen
        self.font = font
        for i in range(9):
            print(i)
    def math(self, screen, font):                           # This was just a test function
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

    pygame.display.set_caption("The ball drops")

    # Clock
    start_ticks = pygame.time.get_ticks()  # starter ticks
    clock = pygame.time.Clock()
    pygame.time.set_timer(pygame.USEREVENT, 1000)

    draw_options = DrawOptions(screen)

    space = pymunk.Space()
    space.gravity = 0, -9.800  # Pymunk defines gravity incorrectly, as seen here, it defines 'down' as a negative acceleration.
    gravity_float = float(space.gravity[1])
    print(f'Gravity float: {gravity_float}')

    # Calculate Time in Seconds
    seconds = (pygame.time.get_ticks() - start_ticks) / 1000
    time = float(seconds)
    droneHUD = HUD(screen, font)

    x = random.randint(120, 380)
    # ground = Ground(space, 144)   # attribute is the height of the ground

    t_history = []
    z_actual = []
    z_history = []

    # Add collision handler to object:
    ch = space.add_collision_handler(0, 0)
    ch.data["surface"] = screen
    ch.post_solve = draw_collision

    myName = 'alx'
    linear_acc_desired = 0.0
    psi_acc_desired = 0.0

    target_z = np.cos(time) - 1   # define target path outside of main fly loop.
    target_z_dot_dot = -np.cos(time)
    # target = np.zeros(5)
    target_z_dot_dot_history = []

    CoaxialDrone = CoaxialCopter('Alex', space)                    # Create drone that can fly
    loop = 0                                                       # Initialize frame counter
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
        time = float(seconds)

        # Calculate dt, dt = 0.02 in class lessons, or dt = t[1]-t[0]
        loop += 1                      # loop is total number of frames
        dt = time / loop               # The amount of time it takes for each frame, time needed per loop
                                       # dt is the time required to increment each step or frame.

        # stable_omega_1, stable_omega_2 = CoaxialDrone.set_rotors_angular_velocities(linear_acc_desired, psi_acc_desired)
        # print(f'\n omega_1: {stable_omega_1:0.4f} , omega_2: {stable_omega_2:0.3f}')

        vertical_acc = CoaxialDrone.z_dot_dot
        # print(
        #     f'Given linear_acc = {linear_acc_desired}, and psi_acc = {psi_acc_desired} of a drone with a mass of {CoaxialDrone.mass} and gravity of {CoaxialDrone.g} results \n in {stable_omega_1:0.3f} , {stable_omega_2:0.3f} -> vert_acc: {vertical_acc:0.4f}')

        #BENCH MARK A ----------------------------------------
        # CoaxialDrone.omega_1 = stable_omega_1 * math.sqrt(2.1)
        # CoaxialDrone.omega_2 = stable_omega_2 * math.sqrt(2.1)
        # print('Bench Mark. Controls-Lesson 1. Coaxial Drone Dynamics. TEST CODE 2: increase ang velocity of props => increase Vert_acc:')
        # print(f'An increase of ang_acc by % math.sqrt(2.1) = {CoaxialDrone.z_dot_dot}')
        # print('The answer in the hwk is -10.79, here the result is -10.78')
        #-----------------------------------------------------

        # #BENCH MARK B: Lesson 1 > Controls-Lesson_1 > TEST CODE 3
        # CoaxialDrone.omega_1 = stable_omega_1 * math.sqrt(1.1)
        # CoaxialDrone.omega_2 = stable_omega_1 * math.sqrt(0.9)
        # print(f'An increase of omega_1 = math.sqrt(1.1) and decrease of omega_2 = math.sqrt(0.9): {CoaxialDrone.psi_dot_dot}')
        # The answer given is 2.45 rad/s^2, the found answer is: 2.45 rad/s^2
        # #-----------------------------------------------------

        ang_acc_truth = CoaxialDrone.body.angular_velocity
        ang_acc = CoaxialDrone.psi_dot_dot

        print('Omegas:')
        print(f'{CoaxialDrone.omega_1:0.3f} {CoaxialDrone.omega_2:0.3f}, vertical acc: {CoaxialDrone.z_dot_dot:0.3f}')
        print(f'angular acc {ang_acc}')

        # Detect Drone's Acceleration Externally
        acceleration_truth = CoaxialDrone.shape.body.velocity[1] / seconds  # Where seconds is (pygame.time.get_ticks() - start_ticks)
                                                                            # Where velocity has an x,y component but HUD only shows velocity[1]
        # Detect Drone's Velocity Externally
        velocity_truth = CoaxialDrone.shape.body.velocity                   # Velocity and Position Vectors are parsed in Rounded fnc.

        # Detect Drone's position Externally
        position_truth = CoaxialDrone.shape.body.position                   # Vec2d(tuple) where position[1] is z height.

        # Get omega values
        omega_1, omega_2 = CoaxialDrone.omega_1, CoaxialDrone.omega_2
        # print(omega_1, omega_2)
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

        # target_z_dot_dot = -1.0

        # BENCH MARK B: Lesson 1 > Controls-Lesson_1 > 2_Advanced State for (Un)controlled drone.
        # CoaxialDrone.advance_state_uncontrolled(dt)
        # Z height homework = 490.
        # Z height PyDrone (simulator) = 558.
        # 558 / 490 is 14% out of tolerance
        # -----------------------------------------

        # CoaxialDrone.set_rotors_angular_velocities(target_z_dot_dot, 0.0)

        # CoaxialDrone.advance_state(dt)
        CoaxialDrone.advance_state_uncontrolled(dt)
        print(f'Velocity is: {CoaxialDrone.z_dot}')

        # Fnet = ma, F = m(g - a), m*g - m*a.   g = space.gravity = (0, 9.8)
        # Fnet_truth =  CoaxialDrone.mass * space.gravity[1] - CoaxialDrone.mass * drone.shape.acceleration?
        Fnet_truth = CoaxialDrone.mass * gravity_float - CoaxialDrone.mass * float(velocity_truth[1] / seconds)
        print(f'Fnet_truth = {Fnet_truth}')

        thrust_truth = CoaxialDrone.mass * gravity_float - Fnet_truth
        print(f'thrust truth: {thrust_truth}')
        print(f'target z dot dot: {target_z_dot_dot}')
        # CoaxialDrone.shape.body.apply_force_at_local_point((0,), (0, 0))  # arguments: (self._body, tuple(force), tuple(point))
        # cp.cpBodyApplyForceAtLocalPoint(self._body, tuple(force), tuple(point))

        # CoaxialDrone.shape.body.apply_force_at_local_point((0,0),(0,0))
        # CoaxialDrone.shape.body.velocity_func = 200.0  ; we are not encouraged to set velocity directly. Just Force.
        t_history.append(time)
        z_history.append(CoaxialDrone.z)
        z_actual.append(CoaxialDrone.shape.body.position)
        # target_z_dot_dot_history.append(-np.cos(time))

        droneHUD.display(CoaxialDrone, time,
                         Fnet_truth, thrust_truth,
                         velocity_truth, acceleration_truth, position_truth,
                         loop, dt, omega_1, omega_2,
                         ang_acc_truth)

        if 5.00 < time < 5.05:
            # BENCHMARK C:
            print(f'The Z height at around 10 seconds is: {z_history[-1]}')
            print(f'The time is: {t_history}')
            # plt.plot(t_history, t_history)
            # plt.plot(t_history, z_actual)

            # plt.plot(t_history, z_actual)
            # plt.plot(t_history, target_z_dot_dot_history)
            # plt.plot(np.cos(t_history))

            x1 = np.linspace(0.0, 5.0)
            x2 = np.linspace(0.0, 2.0)
            x3 = np.linspace(0.0, 5.0)

            y1 = np.cos(2 * np.pi * x1) * np.exp(-x1)
            y2 = np.cos(2 * np.pi * x2)
            y3 = np.cos(2 * np.pi * x3) * np.sin(x3)

            x = np.linspace(0, 2 * np.pi, 400)
            y = np.sin(x ** 2)

            fig, ax = plt.subplots(3)
            ax[0].plot(x,y)
            ax[1].plot(-x,y)
            ax[2].plot(x,-y)

            fig.suptitle('Vertically Stacked Subplots')

            # ax1.plot(x1, y1)
            # ax2.plot(x1, y1)
            # plt.subplot(2, 1, 1)
            # plt.plot(x1, y1, 'o-')
            # plt.title('time')
            # plt.ylabel('Plot 1')
            #
            # plt.subplot(2, 1, 2)
            # plt.plot(x2, y2, '.-')
            # plt.title('2')
            # plt.xlabel('time')
            # plt.ylabel('Plot 2')
            #
            # plt.subplot(2, 1, 2)
            # plt.plot(x3, y3, '.-')
            # plt.title('3')
            # plt.xlabel('time')
            # plt.ylabel('Plot 3')

            # plt.ylabel("Z pos. Est., Z pos Actual")
            # plt.xlabel("Time (seconds)")
            # plt.legend(["Time","Z Actual", "Z Drone", "target z dot dot"])
            # plt.gca().invert_yaxis()
            plt.show()
            sys.exit(0)

        pygame.display.update()

if __name__ == '__main__':
    sys.exit(main())

