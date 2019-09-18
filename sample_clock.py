import pygame
import sys
pygame.init()
screen = pygame.display.set_mode((128, 128))
clock = pygame.time.Clock()

counter, text = 10, '10'.rjust(3)
pygame.time.set_timer(pygame.USEREVENT, 1000)
font = pygame.font.SysFont('Consolas', 30)

while True:
    for event in pygame.event.get():
        if event.type == pygame.USEREVENT:
            counter -= 1
            text = str(counter).rjust(3) if counter > 0 else 'boom!'

        if event.type == pygame.QUIT: break

        elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            print('ESCAPE KEY')
            sys.exit(0)
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_DOWN:
            sys.exit(0)
    else:
        screen.fill((25, 25, 25))
        screen.blit(font.render(text, True, (255, 255, 255)), (37, 48))
        pygame.display.flip()
        clock.tick(60)
        continue
    break