import pymunk
import pygame
import random
import sys
import math
from pymunk.pygame_util import DrawOptions

width = 600
height = 600

class CoaxialCopter:
    def __init__(self, position, space):
        self.mass = 1.0
        self.I_z = 0.2
        self.K_m = 0.1
        self.K_f = 0.007
        self.g = 9.8

        self.shape = pymunk.Poly.create_box(None, size=(50, 10))
        self.moment = pymunk.moment_for_poly(self.mass, self.shape.get_vertices())
        self.body = pymunk.Body(self.mass, self.moment)
        self.shape.body = self.body
        leg1 = pymunk.Segment(self.body, (-20, -30), (-10, 0), 3)  # 2
        leg2 = pymunk.Segment(self.body, (20, -30), (10, 0), 3)

        self.shape.body.position = position
        space.add(self.shape, self.body, leg1, leg2)

    @property
    def z_dot_dot(self):
        f_1 = self.K_f * self.omega_1**2
        f_2 = self.K_f * self.omega_2**2
        f_g = self.mass * self.g
        f_total = -f_1 - f_2 + f_g
        acceleration = f_total / self.m
        return acceleration

    @property
    def psi_dot_dot(self):
        cw_torque = self.K_m * self.omega_1 **2
        ccw_torque = self.K_m * self.omega_2 **2
        net_torque = cw_torque - ccw_torque
        angular_acc = net_torque / self.I_z
        return angular_acc

    def set_rotors_angular_velocities(self, linear_acc, angular_acc):
        term_1 =  self.mass * (-linear_acc + self.g) /(2*self.K_f)
        term_2 = self.I_z * angular_acc / (2 * self.K_m)
        omega_1 = math.sqrt(term_1 + term_2)
        omega_2 = math.sqrt(term_1 - term_2)        
        
        self.omega_1 = -omega_1
        self.omega_2 = omega_2
        return self.omega_1, self.omega_2

class Ground:
    def __init__(self, space):
        self.body = pymunk.Body(0, 0, body_type=pymunk.Body.STATIC)
        self.shape = pymunk.Poly.create_box(self.body, (width, 10))
        self.shape.body.position = (width//2, 10)
        space.add(self.shape, self.body)


def main():

    pygame.init()
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("The ball drops")
    clock = pygame.time.Clock()
    print(clock)
    draw_options = DrawOptions(screen)

    space = pymunk.Space()
    space.gravity = 0, -9.8
    x = random.randint(120, 380)
    ground = Ground(space)
    coaxialDrone = CoaxialCopter((x, 350), space)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit(0)
                
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                sys.exit(0)

        """
        This is the code that applies force to the body of the drone
        """
       # if ball.shape.body.position.y < 200:
       #     """  (0, 400) means apply 400 units of force in the dirction of y (0,0) is the co-ordinate to apply the force too"""
       

        if event.type == pygame.KEYDOWN and event.key == pygame.K_UP:
                stable_omega_1,stable_omega_2 = coaxialDrone.set_rotors_angular_velocities(0.0, 0.0)
                print(stable_omega_1,stable_omega_2)
                pygame.display.set_caption("His monk flies")

        if event.type == pygame.KEYDOWN and event.key == pygame.K_DOWN:
            coaxialDrone.shape.body.apply_force_at_local_point((0, -400), (0, 0))
            print('time', pygame.time.Clock())
            pygame.display.set_caption("His monk drop")
            

        # ball.shape.body.apply_force_at_local_point((0, 100), (0, 0))

        screen.fill((0, 0, 0))
        space.debug_draw(draw_options)
        space.step(1/50.0)
        pygame.display.update()
        clock.tick(50)


if __name__ == '__main__':
    sys.exit(main())
