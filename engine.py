#Nybble Game Engine v0.1 - for Python 3.2+

# Game Engine is at its infancy which means that some
# functionalities are not fully optimized such as the systems
# processing the only components they need.
# This Engine may be later optimized but for now it is
# for small 2D games and for teaching purposes.

import pygame
import sys
from pygame.locals import *

# for sound
from pygame import mixer
from pygame import font

from util_math import Vector2
from managers import IdManager
from systems import RenderSystem

# Engine processes the current world, reads input events
# and handles the main game loop


class Engine:

    # Requires screen parameters
    def __init__(self, display_w, display_h):
        pygame.init()
        mixer.init()
        font.init()

        # check if the mixer was successfully initialized
        if mixer.get_init() is None:
            print("Failed to initialize the audio mixer module.")

        if font.get_init() is None:
            print("Failed to initialize the font module.")

        self.fps = 120
        self.world = None
        self.gui = Gui(self)

        # Create screen display with 32 bits per pixel, no flags set
        self.display = pygame.display.set_mode((display_w, display_h), pygame.HWSURFACE, 32)
        self.delta_time = 0.0
        self.debug = False
        self.paused = False

        self.print_fps = False

        self.worlds = list()

        self.game = None

    def set_world(self, world, append=False):
        world.engine = self
        self.world = world
        self.world.resume()

        if append:
            self.worlds.append(world)

    def run(self):

        timer = pygame.time.Clock()
        last_frame_time = 0.0

        for world in self.worlds:

            world.engine = self

            # load the scene of the world before running
            world.start_scene_loading()

            render_system = world.get_system(RenderSystem.tag)

            # failed to obtain the render system
            if render_system is None:
                print("Error. Render system does not exist in the world.")
                return

            # construct the scene order from the initial entities
            render_system.construct_scene(world.entity_manager.entities)

        while True:

            # do not run the game if delta time is too high
            if self.delta_time >= 0.05:
                self.delta_time = 0.0
                continue

            # Get the initial time in milliseconds of the current frame
            frame_start_time = pygame.time.get_ticks()

            if self.print_fps:
                print("FPS: ", timer.get_fps(), "delta time: ", self.delta_time)

            if self.world is None:
                print("Error, the world specified is None.")
                Engine.clean_up()

            # poll input events
            for event in pygame.event.get():
                if event.type == QUIT:
                    Engine.clean_up()

                # key down presses
                elif event.type == pygame.KEYDOWN:

                    # pause the game
                    if event.key == pygame.K_p:
                        self.paused = not self.paused

                        # pause/resume audio
                        if self.paused:
                            mixer.pause()
                            mixer.music.pause()
                        else:
                            mixer.unpause()
                            mixer.music.unpause()

                    # toggle debug mode
                    elif event.key == pygame.K_F12:
                        self.debug = not self.debug

                    elif event.key == pygame.K_F11:
                        self.print_fps = not self.print_fps

                # pass input events to the world
                if not self.paused:
                    self.world._take_input(event)

            # Run the currently set world
            if not self.paused:
                self.world.run()

            # draw gui elements on top of everything
            self.gui.draw_widgets()

            pygame.display.update()

            # The time interval between this frame and the last one.
            # Convert the time from milliseconds to seconds
            self.delta_time = (frame_start_time - last_frame_time)/1000.0
            last_frame_time = frame_start_time
            timer.tick(self.fps)

    @staticmethod
    def clean_up():
        font.quit()
        mixer.quit()
        pygame.quit()
        sys.exit()


# Handles rendering the GUI elements
class Gui:

    class Widget:

        def __init__(self, image_surface=None, pos=Vector2(0, 0)):
            self.uuid = 0
            self.position = pos
            self.image = image_surface
            self.tag = ""

        def __eq__(self, other):
            return self.uuid == other.uuid

    def __init__(self, engine):
        self.widgets = list()
        self.engine = engine
        self.id_manager = IdManager()

    # draw buttons, text, labels, hud elements
    def draw_widgets(self):

        display = self.engine.display

        for widget in self.widgets:

            x = widget.position.x
            y = widget.position.y

            display.blit(widget.image, (x, y))

    # Add widget to gui and assign a uuid.
    def add_widget(self, widget):
        widget.uuid = self.id_manager.get_id()
        self.widgets.append(widget)

    # Change image of the widget.
    def update_widget_image(self, widget_tag, image_surface):
        # find widget by tag
        for widget in self.widgets:
            if widget.tag == widget_tag:
                widget.image = image_surface
                break

    # remove widget from the gui handler
    def remove_widget(self, widget):
        self.widgets.remove(widget)