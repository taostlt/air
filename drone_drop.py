import pymunk
import pygame
import random
import sys
import math
import numpy as np
from pymunk.pygame_util import DrawOptions

screen = pygame.display.set_mode((128, 128))


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

        self.g = 9.8
        self.omega_1 = 0.0
        self.omega_2 = 0.0
        self.name = name

        self.z = 0.0
        self.z_dot = 0.0
        self.psi = 0.0
        self.psi_dot = 0.0

        # self.X = np.array([0.0, 0.0, 0.0, 0.0])

        position = (300, 500)

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

    def set_rotors_angular_velocities(self, linear_acc, angular_acc):
        term_1 =  self.mass * (-linear_acc + self.g) /(2*self.K_f)
        term_2 = self.I_z * angular_acc / (2 * self.K_m)
        omega_1 = math.sqrt(term_1 + term_2)
        omega_2 = math.sqrt(term_1 - term_2)        
        
        self.omega_1 = -omega_1
        self.omega_2 = omega_2
        return self.omega_1, self.omega_2

    def advance_state_uncontrolled(self, dt):
        z_dot_dot = 9.81

        delta_z_dot = z_dot_dot * self.dt
        self.z_dot = self.z_dot + delta_z_dot

        delta_z = self.z_dot * self.dt
        self.z = self.z + delta_z

    def advance_state(self, dt):
        """Advances the state of the drone forward by dt seconds"""
        #
        # TODO
        #  Implement this method! Your implementation may look
        #  VERY similar to the uncontrolled version of this function.
        z_dot_dot = self.z_dot_dot

        delta_z_dot = dt * z_dot_dot
        self.z_dot += delta_z_dot

        delta_z = self.z_dot * dt
        self.z += delta_z
        # print('z: ', self.z)

        # update psi state (see method "psi_dot_dot" implemented before)
        psi_dot_dot = self.psi_dot_dot

        delta_psi_dot = dt * psi_dot_dot
        self.psi_dot += delta_psi_dot

        # technically we should restrict psi so it's between
        # -pi and pi, but we're not going to worry about that now
        delta_psi = self.psi_dot * dt
        self.psi += delta_psi

        # print(f'z position: {self.z:,.2f}')

class Ground:
    def __init__(self, space):
        self.body = pymunk.Body(0, 0, body_type=pymunk.Body.STATIC)
        self.shape = pymunk.Poly.create_box(self.body, (width, 10))
        self.shape.body.position = (width//2, 10)
        space.add(self.shape, self.body)

class Player(pygame.sprite.Sprite):
    def __init__(self):
        super(Player, self).__init__()
        self.surf = pygame.Surface((100, 30))
        self.surf.fill((255, 255, 255))
        self.rect = self.surf.get_rect()

scoreCard = Player()


def main():
    pygame.init()

    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("The ball drops")

    # clock
    start_ticks = pygame.time.get_ticks()  # starter ticks
    clock = pygame.time.Clock()

    counter, text = 10, '10'.rjust(3)
    pygame.time.set_timer(pygame.USEREVENT, 1000)
    font = pygame.font.SysFont('Consolas', 30)

    draw_options = DrawOptions(screen)

    space = pymunk.Space()
    space.gravity = 0, -9.8
    x = random.randint(120, 380)
    ground = Ground(space)

    myName = 'alx'
    coaxialDrone = CoaxialCopter('Alex', space)
    stable_omega_1, stable_omega_2 = coaxialDrone.set_rotors_angular_velocities(0.0, 0.0)

    coaxialDrone.omega_1 = stable_omega_1 * math.sqrt(1.1)
    coaxialDrone.omega_2 = stable_omega_2 * math.sqrt(1.1)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit(0)
                
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                sys.exit(0)

            # elif event.type == pygame.USEREVENT:
            #     counter -= 1
            #     text = str(counter).rjust(3) if counter > 0 else 'boom!'
            #     screen.fill((255, 255, 255))
            #     screen.blit(font.render(text, True, (255, 200, 255)), (0, 0))
            #     pygame.display.flip()
            #     print(text)


        # """
        # This is the code that applies force to the body of the drone
        # """
        # if ball.shape.body.position.y < 200:
        #     """  (0, 400) means apply 400 units of force in the dirction of y (0,0) is the co-ordinate to apply the force too"""
       

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_UP:
                        stable_omega_1,stable_omega_2 = coaxialDrone.set_rotors_angular_velocities(0.0, 0.0)
                        print('Rotation of propeller 1, 2:', stable_omega_1,stable_omega_2)
                        vertical_acceleration = coaxialDrone.z_dot_dot
                        print('Vertical acceleration =', vertical_acceleration)
                        print(coaxialDrone.X)

                        coaxialDrone.shape.body.apply_force_at_local_point((0, vertical_acceleration), (0, 0))
                        pygame.display.set_caption("His monk flies")

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_DOWN:
                coaxialDrone.shape.body.apply_force_at_local_point((0, -400), (0, 0))
                print('time', pygame.time.Clock())
                pygame.display.set_caption("His monk drop")


        # ball.shape.body.apply_force_at_local_point((0, 100), (0, 0))

        screen.fill((0, 0, 0))
        space.debug_draw(draw_options)
        space.step(1/60.0)
        clock.tick(60)

        # Draw the player to the screen
        screen.blit(scoreCard.surf, (0, 0))
        # screen.blit(player2.surf, (0, 0))
        # counter += 1

        seconds = (pygame.time.get_ticks() - start_ticks) / 1000
        seconds_float = float(seconds)

        print(seconds)
        text = str(seconds_float).rjust(3)
        screen.blit(font.render(text, True, (0, 0, 0)), (0, 0))

        # Update the display
        # pygame.display.flip()

        pygame.display.update()
        print(counter)


        # coaxialDrone.advance_state(dt)


if __name__ == '__main__':
    sys.exit(main())

