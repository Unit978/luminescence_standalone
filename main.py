__author__ = 'luisl_000'

from main_room import PlatformWorld
from maze import Maze
from fibpuzzle import FibWorld
from engine import Engine

import pygame
from pygame import QUIT


class Game(object):

    def __init__(self):

        self.engine = None
        self.main_room = None
        self.maze_room = None
        self.fib_room = None

    def start(self):

        self.engine = Engine(1200, 700)

        self.engine.game = self

        screen = pygame.display.set_mode((1200, 700), pygame.HWSURFACE, 32)
        title_screen = pygame.image.load("assets/images/gui/title_screen.png").convert()

        x = 1200/2 - title_screen.get_width()/2
        y = 700/2 - title_screen.get_height()/2

        # show title screen
        exit_title_screen = False
        while not exit_title_screen:

            for event in pygame.event.get():
                if event.type == QUIT:
                    Engine.clean_up()

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_RETURN:
                        exit_title_screen = True

            screen.fill((0, 0, 0))
            screen.blit(title_screen, (x, y))

            pygame.display.update()

        self.main_room = PlatformWorld()
        self.maze_room = Maze()
        self.fib_room = FibWorld()

        self.engine.worlds.append(self.main_room)
        self.engine.worlds.append(self.maze_room)
        self.engine.worlds.append(self.fib_room)

        self.engine.set_world(self.main_room)
        self.engine.run()

    def go_to_maze(self):
        self.engine.set_world(self.maze_room)

    def go_to_fib(self):
        self.engine.set_world(self.fib_room)

    def go_to_main(self):
        self.engine.set_world(self.main_room)

    def go_to_end(self):
        screen = pygame.display.set_mode((1200, 700), pygame.HWSURFACE, 32)
        title_screen = pygame.image.load("assets/images/gui/end_screen.png").convert()

        x = 1200/2 - title_screen.get_width()/2
        y = 700/2 - title_screen.get_height()/2

        # show title screen
        exit_title_screen = False
        while not exit_title_screen:

            for event in pygame.event.get():
                if event.type == QUIT:
                    Engine.clean_up()

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_RETURN:
                        exit_title_screen = True

            screen.fill((0, 0, 0))
            screen.blit(title_screen, (x, y))

            pygame.display.update()
        self.start()


luminescence = Game()
luminescence.start()