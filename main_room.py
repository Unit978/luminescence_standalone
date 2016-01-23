
from world import *
from engine import *

from scripts import *
from utility import *
from state_machine import *

engine = Engine(1200, 700)

monster_appearance_sfx = mixer.Sound("assets/sound/piano_low_key.wav")


class BookShelfInteraction(BehaviorScript):

    def __init__(self):
        super(BookShelfInteraction, self).__init__("book shelf interaction")

        self.showing_hint = False

        self.mouse_rect = Rect(0, 0, 5, 5)

    def take_input(self, event):

        if event.type == pygame.MOUSEBUTTONDOWN:

            # interacted with book shelve
            if not self.showing_hint:
                x_mouse = pygame.mouse.get_pos()[0]
                y_mouse = pygame.mouse.get_pos()[1]

                camera_pos = self.entity.world.get_system(RenderSystem.tag).camera.transform.position

                # adjust to the camera
                x_mouse += camera_pos.x
                y_mouse += camera_pos.y

                # center on the mouse
                self.mouse_rect.center = (x_mouse, y_mouse)

                for book_shelf in self.entity.world.book_shelves:

                    # player must be touching book shelf
                    if PhysicsSystem.box2box_collision(self.entity.collider, book_shelf.collider):

                        # player clicked on book shelf
                        if self.mouse_rect.colliderect(book_shelf.collider.box):
                            self.showing_hint = True

                            self.entity.world.engine.gui.add_widget(self.entity.world.text)

            elif self.showing_hint:
                self.entity.world.engine.gui.remove_widget(self.entity.world.text)
                self.showing_hint = False


class ExitMainRoom(WorldScript):

    def __init__(self):
        super(ExitMainRoom, self).__init__("exit main room")

        self.puzzles_done = False

    def update(self):
        # check to see of the puzzles are finished
        if not self.puzzles_done:
            self.puzzles_done = self.world.engine.game.fib_room.puzzle_finished and self.world.engine.game.maze_room.puzzle

        # elevator hasn't been triggered yet
        elevator_cabin = self.world.get_entity_by_tag("cabin")
        if self.puzzles_done and elevator_cabin.collider.is_trigger:
            elevator_cabin = self.world.get_entity_by_tag("cabin")

            # on the elevator cabin
            if PhysicsSystem.box2box_collision(self.world.player.collider, elevator_cabin.collider):
                monster_appearance_sfx.play()
                elevator_cabin.collider.is_trigger = False
                elevator_cabin.collider.treat_as_dynamic = True


class MoveCabin(BehaviorScript):

    def __init__(self):
        super(MoveCabin, self).__init__("move cabin")
        self.speed = 30

        # ten seconds going up the elevator
        self.timer_to_end = 10.0

    def update(self):

        # move up if activated
        if not self.entity.collider.is_trigger:
            self.entity.transform.position.y -= self.speed * self.entity.world.engine.delta_time
            self.timer_to_end -= self.entity.world.engine.delta_time

        if self.timer_to_end < 0:
            self.entity.world.engine.game.go_to_end()


class MonsterMovement(BehaviorScript):

    def __init__(self):
        super(MonsterMovement, self).__init__("monster movement")
        self.speed = 290
        self.velocity = Vector2(0, 0)

        self.killed_player = False

    def update(self):

        # follow the player if he is alive
        if not self.killed_player:
            player = self.entity.world.player

            direction = player.transform.position - self.entity.transform.position
            direction.normalize()

            dt = self.entity.world.engine.delta_time

            self.entity.transform.position += direction * self.speed * dt

            # make the lamp follow the monster
            self.entity.world.monster_light.transform.position = self.entity.transform.position

    def take_input(self, event):

        # restart the game if the player died
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_r:
                if self.killed_player:
                    self.entity.world.engine.game.start()

    def collision_event(self, other_collider):

        # hit the player - player dies
        if other_collider.entity.name == "player":

            player = self.entity.world.player

            if not self.killed_player:
                # disable all player functionality
                player.disabled = True

                # display a red cross over the player to signify that he is dead
                img = pygame.image.load("assets/images/effects/blood_splatter.png").convert_alpha()
                splatter = self.entity.world.create_renderable_object(img)
                splatter.renderer.depth = -100
                splatter.transform.position = self.entity.transform.position
                self.killed_player = True


class UpdateAnimationHandler(WorldScript):

    def __init__(self, anim_state_machine):
        super(UpdateAnimationHandler, self).__init__("animation handler")
        self.anim_state_machine = anim_state_machine

    def update(self):
        self.anim_state_machine.update()

        # make the lamp source follow the player
        self.world.lamp_source.transform.position = self.world.player.transform.position


class TeleportCrate(BehaviorScript):

    def __init__(self):
        super(TeleportCrate, self).__init__("teleport crate")

    # if the crate collides with a teleporter, spawn them from the ceilings at certain points
    def collision_event(self, other_collider):

        if other_collider.entity.name == "teleport b":
            self.entity.transform.position = Vector2(540, -300)

        elif other_collider.entity.name == "teleport a":
            self.entity.transform.position = Vector2(1800, -300)


class DeactivateSaw(BehaviorScript):

    def __init__(self):
        super(DeactivateSaw, self).__init__("deactivate saw")

    def collision_event(self, other_collider):

        if other_collider.entity.tag == "saw switch":

            new_switch_image = pygame.image.load("assets/images/tiles/56x100_switchON.png").convert()

            other_collider.entity.renderer.set_image(new_switch_image)

            # get the saw
            saw = self.entity.world.get_entity_by_tag("saw")

            saw.collider.is_trigger = True
            saw.remove_component(Animator.tag)


# This will handle the dimming of lamp light and regeneration of lamp light from the other lamps
class HandleLightLife(BehaviorScript):

    def __init__(self):
        super(HandleLightLife, self).__init__("handle light life")

        self.max_lamp_life = 100.0
        self.max_time_monster = 8.0

        # lamp light life in seconds
        self.lamp_life = self.max_lamp_life

        # 10 secs are given so player can refuel lamp
        self.monster_appearance_timer = self.max_time_monster

        self.monster_spawned = False

    def update(self):
        # reduce lamp strength at every mutliple of 5
        if self.lamp_life > 0 and int(self.lamp_life) % 5 == 0:
            scale = self.lamp_life / self.max_lamp_life
            self.entity.world.lamp_source.transform.scale_by(scale, scale)

        # spawn monster
        if self.monster_appearance_timer < 0 and not self.monster_spawned:
            monster_appearance_sfx.play()
            self.entity.world.initialize_monster()
            self.monster_spawned = True

        # start secondary timer for monster to appear
        if self.lamp_life < 0 < self.monster_appearance_timer:
            self.monster_appearance_timer -= self.entity.world.engine.delta_time

        # reduce lamp life
        if self.lamp_life > 0:
            self.lamp_life -= self.entity.world.engine.delta_time

    def take_input(self, event):

        # if the player wants to take a light
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:

                # make sure we are colliding with a light source
                for lamp_light in self.entity.world.lamp_lights:

                    # player is in range of a lamp light
                    if PhysicsSystem.box2box_collision(self.entity.collider, lamp_light.collider):

                        # reset lamp life and monster appearance timer
                        self.lamp_life = self.max_lamp_life
                        self.monster_appearance_timer = self.max_time_monster

                        # disable monster
                        if self.monster_spawned:
                            self.entity.world.disable_monster()

                        # set lamp source back to max capacity
                        self.entity.world.lamp_source.transform.scale_by(1, 1)

                        # destroy the lamp light that you obtained fuel from
                        self.entity.world.destroy_entity(lamp_light)

                        # remove from lamp lights list
                        self.entity.world.lamp_lights.remove(lamp_light)

                        # remove it from the renderer
                        self.entity.world.get_system(RenderSystem.tag).light_sources.remove(lamp_light)

                        self.monster_spawned = False
                        return


class GoToOtherLevel(BehaviorScript):

    def __init__(self):
        super(GoToOtherLevel, self).__init__("go to other level")

    def collision_event(self, other_collider):

        if other_collider.entity.name == "trigger to maze":

            # player will be falling down once he comes back from the maze
            self.entity.rigid_body.velocity.zero()
            self.entity.transform.position = Vector2(1100, -320)

            self.entity.world.engine.game.go_to_maze()

        elif other_collider.entity.name == "trigger to fib":

            # shift back the player a bit
            self.entity.transform.position.x -= (self.entity.collider.box.w/2 + 10)
            self.entity.rigid_body.velocity.zero()

            self.entity.world.engine.game.go_to_fib()


class PlatformWorld(World):

    def __init__(self):
        super(PlatformWorld, self).__init__()

        self.player = None
        self.player_anim_handler = None

        self.ladders = list()
        self.lamp_source = None
        self.crates = list()

        # object that determine if the player should be grounded
        # when collided on
        self.ground = list()

        self.monster = None
        self.monster_light = None

        self.lamp_lights = list()

        self.book_shelves = list()

        self.text = None

    def resume(self):
        # load music to play in the background
        mixer.music.load("assets/music/MarysCreepyCarnivalTheme.ogg")
        mixer.music.play(-1)
        mixer.music.set_volume(0.3)

    def load_scene(self):

        img = pygame.image.load("assets/images/gui/hint.png").convert()
        self.text = self.engine.gui.Widget(img, Vector2(0, 0))

        w = self.engine.display.get_width()
        h = self.engine.display.get_height()
        self.origin = Vector2(0, -450)

        # world bounds
        self.width = 4600
        self.height = 1100

        # setup the render system for a dark environment
        self.get_system(RenderSystem.tag).simulate_dark_env = True
        self.get_system(RenderSystem.tag).blit_buffer = pygame.Surface((w, h)).convert()

        # triggers to the other worlds
        trigger_to_maze = self.create_box_collider_object(200, 200)
        trigger_to_maze.collider.is_trigger = True
        trigger_to_maze.transform.position = Vector2(-90, 70)
        trigger_to_maze.name = "trigger to maze"

        trigger_to_fib = self.create_box_collider_object(200, 200)
        trigger_to_fib.collider.is_trigger = True
        trigger_to_fib.transform.position = Vector2(4700, 370)
        trigger_to_fib.name = "trigger to fib"

        self.load_backgrounds()
        self.load_player()
        self.load_ladders()
        self.load_floors()
        self.load_ceilings()
        self.load_walls()
        self.load_platforms()
        self.load_elevators()
        self.load_boxes()
        self.load_lights()
        self.load_book_shelves()
        self.load_saw()

        # set up the foundation for the monster
        self.monster = self.create_entity()
        self.monster.add_component(Transform())
        self.monster.tag = "monster"

        # set up camera
        render = self.get_system(RenderSystem.tag)
        render.camera = self.create_entity()
        render.camera.add_component(Transform(Vector2(0, 0)))
        render.camera.add_script(CameraFollow("camera follow", self.player.transform, w, h))

        # start the background music and set it to loop forever
        mixer.music.play(-1)
        mixer.music.set_volume(0.3)

        self.get_system(PhysicsSystem.tag).gravity.y += 200

        self.add_script(UpdateAnimationHandler(self.player_anim_handler))

        # get all the lamp lights
        for e in self.entity_manager.entities:
            if e.tag == "lamp light":
                self.lamp_lights.append(e)

            elif e.tag == "book shelf":
                self.book_shelves.append(e)

            # construct ground objects list
            elif self.is_ground(e):
                self.ground.append(e)

        self.add_script(ExitMainRoom())

    def load_lights(self):

        render_sys = self.get_system(RenderSystem.tag)

        # a light source to see the monster
        lamp_light_img = pygame.image.load("assets/images/lights/lamp_light_xsmall_mask.png").convert_alpha()
        self.monster_light = self.create_renderable_object(lamp_light_img)
        self.monster_light.renderer.depth = 10000

        large_lamp_light_img = pygame.image.load("assets/images/lights/lamp_light_mask.png").convert_alpha()
        self.lamp_source = self.create_renderable_object(large_lamp_light_img)
        self.lamp_source.renderer.depth = 10000
        render_sys.light_sources.append(self.lamp_source)

        lamp_light_img = pygame.image.load("assets/images/lights/lamp_light_small_mask.png").convert_alpha()
        lamp_img = pygame.image.load("assets/images/environment/lamp.png").convert_alpha()

        # LAMP AT WALL A
        lamp = self.create_renderable_object(lamp_img)
        lamp.transform.position = Vector2(185, 160)

        lamp_light = self.create_renderable_object(lamp_light_img)
        lamp_light.transform.position = Vector2(185, 160)
        set_lamp_light_attributes(lamp_light, render_sys)

        # LAMP AT PLATFORM B
        lamp = self.create_renderable_object(lamp_img)
        lamp.transform.position = Vector2(600, 415)

        lamp_light = self.create_renderable_object(lamp_light_img)
        lamp_light.transform.position = Vector2(600, 415)
        set_lamp_light_attributes(lamp_light, render_sys)

        # LAMP AT PLATFORM F
        lamp = self.create_renderable_object(lamp_img)
        lamp.transform.position = Vector2(2000, -20)

        lamp_light = self.create_renderable_object(lamp_light_img)
        lamp_light.transform.position = Vector2(2000, -20)
        set_lamp_light_attributes(lamp_light, render_sys)

        # OVER SAW
        lamp = self.create_renderable_object(lamp_img)
        lamp.transform.position = Vector2(2800, 200)

        lamp_light = self.create_renderable_object(lamp_light_img)
        lamp_light.transform.position = Vector2(2800, 200)
        set_lamp_light_attributes(lamp_light, render_sys)

        # At right end of the level
        lamp = self.create_renderable_object(lamp_img)
        lamp.transform.position = Vector2(4550, 380)

        lamp_light = self.create_renderable_object(lamp_light_img)
        lamp_light.transform.position = Vector2(4550, 380)
        set_lamp_light_attributes(lamp_light, render_sys)

    def load_saw(self):

        img = pygame.image.load("assets/images/environment/hazards/saw.png").convert_alpha()
        saw = self.create_game_object(img)
        saw.transform.scale_by(0.75, 0.75)
        saw.renderer.depth = 70
        saw.transform.position = Vector2(3200, 480)
        saw.tag = "saw"

        animator = Animator()
        saw.add_component(animator)

        anim = Animator.Animation()
        anim.frame_latency = 0.01

        # create multiple rotating frames for the saw
        for degree in range(0, 360, 360/15):

            #obtain original dimensions
            original_rect = saw.renderer.sprite.get_rect()
            rotated_surface = pygame.transform.rotate(saw.renderer.sprite, degree)

            #adjust new surface's center with the original's
            rotate_rect = original_rect.copy()
            rotate_rect.center = rotated_surface.get_rect().center

            rotated_surface = rotated_surface.subsurface(rotate_rect).copy()

            anim.add_frame(rotated_surface)

        animator.set_animation(anim)

        img = pygame.image.load("assets/images/tiles/56x100_switchOFF.png").convert()

        # add the switch to deactivate lever
        switch = self.create_game_object(img)
        switch.collider.is_trigger = True
        switch.transform.position = Vector2(2050, -120)
        switch.renderer.depth = 70
        switch.tag = "saw switch"

    def load_book_shelves(self):

        img = pygame.image.load("assets/images/environment/bookcase.png").convert()
        w = img.get_width()
        h = img.get_height()
        pivot = Vector2(w/2, h/2)

        book = self.create_box_collider_object(w, h)
        book.add_component(Renderer(img, pivot))
        book.transform.position = Vector2(400, 500)
        book.tag = "book shelf"

        book.collider.is_trigger = True
        book.renderer.depth = 70

        book = self.create_box_collider_object(w, h)
        book.add_component(Renderer(img, pivot))
        book.transform.position = Vector2(4350, 0)

        book.collider.is_trigger = True
        book.renderer.depth = 70
        book.tag = "book shelf"

    def load_backgrounds(self):

        # load a plain black background that goes with the camera
        w = self.engine.display.get_width()
        h = self.engine.display.get_height()
        background_image = pygame.Surface((w, h))
        background_image.convert()
        background_image.fill((0, 0, 0))

        background = self.create_entity()
        background.add_component(Transform(Vector2(0, 0)))
        background.add_component(Renderer(background_image))
        background.renderer.depth = 110
        background.renderer.is_static = True

        path = "assets/images/backgrounds/"
        img = pygame.image.load(path + "eye_duck.png").convert()
        background = self.create_renderable_object(img)
        background.renderer.pivot = Vector2(0, 0)
        #background = self.create_box_collider_object()
        background.renderer.depth = 100
        background.transform.position = Vector2(200, -300)

        img = pygame.image.load(path + "horse.png").convert()
        background = self.create_renderable_object(img)
        background.renderer.pivot = Vector2(0, 0)
        background.renderer.depth = 100

        x = 2600
        background.transform.position = Vector2(x, -300)

        w = img.get_width()
        img = pygame.image.load(path + "all_toys.png").convert()
        background = self.create_renderable_object(img)
        background.renderer.pivot = Vector2(0, 0)
        background.renderer.depth = 100
        background.transform.position = Vector2(x + w, -300)

        img = pygame.image.load(path + "no_toys.png").convert()
        background = self.create_renderable_object(img)
        background.renderer.pivot = Vector2(0, 0)
        background.renderer.depth = 100
        background.transform.position = Vector2(1300, -300)

    def load_ladders(self):
        path = "assets/images/ladders/"
        load = pygame.image.load
        ladder_body = load(path + "ladder_body.png").convert_alpha()
        ladder_top = load(path + "ladder_top.png").convert_alpha()

        shift = 200

        self.create_ladder(ladder_body, ladder_top, 470, 950-shift, 385)
        self.create_ladder(ladder_body, ladder_top, 300, 1650-shift, 10)
        self.create_ladder(ladder_body, ladder_top, 460, 1920-shift-50, 375)

    def load_platforms(self):

        path = "assets/images/platforms/"

        load = pygame.image.load

        img_200x30 = load(path + "30x200.png").convert_alpha()
        img_400x30 = load(path + "30x400.png").convert_alpha()
        img_250x50 = load(path + "50x250.png").convert_alpha()
        img_400x120 = load(path + "120x400.png").convert_alpha()
        # img_300x30 = load(path + "30x300.png").convert_alpha()
        img_800x150 = load(path + "150x800.png").convert_alpha()
        img_300x50 = load(path + "50x300.png").convert_alpha()

        plat_a = self.create_game_object(img_200x30)
        plat_a.transform.position = Vector2(300, 250+50+50)
        set_platform_attributes(plat_a)

        plat_b = self.create_game_object(img_400x30)
        plat_b.transform.position = Vector2(450, 400+50)
        set_platform_attributes(plat_b)

        shift = 200
        plat_c = self.create_game_object(img_250x50)
        plat_c.transform.position = Vector2(1100-shift, 200)
        set_platform_attributes(plat_c)

        plat_d = self.create_game_object(img_250x50)
        plat_d.transform.position = Vector2(1700-shift, 200)
        set_platform_attributes(plat_d)

        plat_e = self.create_game_object(img_250x50)
        plat_e.transform.position = Vector2(1500-shift, -100)
        set_platform_attributes(plat_e)

        plat_f = self.create_game_object(img_400x120)
        plat_f.transform.position = Vector2(2150-shift-50, 250)
        set_platform_attributes(plat_f)

        plat_g = self.create_game_object(img_200x30)
        plat_g.transform.position = Vector2(2250-shift-50, -50)
        set_platform_attributes(plat_g)

        # plat_h = self.create_game_object(img_300x30)
        # plat_h.transform.position = Vector2(1800, -150)
        # set_platform_attributes(plat_h)
        # plat_h.add_script(PlatformMovement("plat move"))

        plat_i = self.create_game_object(img_800x150)
        plat_i.transform.position = Vector2(3150, 300)
        set_platform_attributes(plat_i)

        plat_j = self.create_game_object(img_300x50)
        plat_j.transform.position = Vector2(4000, 120)
        set_platform_attributes(plat_j)

    def load_walls(self):

        path = "assets/images/walls/"

        load = pygame.image.load

        img_200x500 = load(path + "200x500.png").convert_alpha()
        img_200x350 = load(path + "200x350.png").convert_alpha()
        img_200x200 = load(path + "200x200.png").convert_alpha()
        img_600x170 = load(path + "170x600.png").convert_alpha()

        wall_a = self.create_game_object(img_200x500)
        wall_a.transform.position = Vector2(100, 350+75)
        set_wall_attributes(wall_a)

        wall_b = self.create_game_object(img_200x350)
        wall_b.transform.position = Vector2(80, -200)
        set_wall_attributes(wall_b)

        wall_c = self.create_game_object(img_200x200)
        wall_c.transform.position = Vector2(4600, 500)
        set_wall_attributes(wall_c)

        wall_d = self.create_game_object(img_600x170)
        wall_d.transform.position = Vector2(4400, 180)
        set_wall_attributes(wall_d)

        wall_e = self.create_game_object(img_200x200)
        wall_e.transform.position = Vector2(4600, 0)
        set_wall_attributes(wall_e)

    def load_floors(self):

        w = self.engine.display.get_width()
        h = self.engine.display.get_height()

        floor_tile = pygame.image.load("assets/images/floors/floor_tile.png").convert_alpha()

        img = create_img_from_tile(floor_tile, w*2, 200)
        floor_a = self.create_game_object(img)
        floor_a.transform.position = Vector2(w, h)
        set_floor_attributes(floor_a)

        img = create_img_from_tile(floor_tile, 1000, 200)
        x = w*2 + 100 + 500 + 50
        floor_b = self.create_game_object(img)
        floor_b.transform.position = Vector2(x, h)
        set_floor_attributes(floor_b)

        x += 250 + 800 + 150
        floor_c = self.create_game_object(img)
        floor_c.transform.position = Vector2(x, h)
        set_floor_attributes(floor_c)

    def load_ceilings(self):

        w = self.engine.display.get_width()
        floor_tile = pygame.image.load("assets/images/floors/floor_tile.png").convert_alpha()

        img = create_img_from_tile(floor_tile, w*2, 200)
        img = pygame.transform.flip(img, False, True)
        ceil_a = self.create_game_object(img)
        ceil_a.transform.position = Vector2(w, -470)
        set_ceiling_attributes(ceil_a)

        img = create_img_from_tile(floor_tile, 1200, 400)
        img = pygame.transform.flip(img, False, True)
        ceil_b = self.create_game_object(img)
        ceil_b.transform.position = Vector2(w*2 + 500, -350)
        set_ceiling_attributes(ceil_b)

        img = create_img_from_tile(floor_tile, 1150, 200)
        img = pygame.transform.flip(img, False, True)
        ceil_c = self.create_game_object(img)
        ceil_c.transform.position = Vector2(w*2 + 1650, -470)
        set_ceiling_attributes(ceil_c)

    def load_player(self):

        # load animation frames
        self.load_anims()

        starting_image = self.player_anim_handler.current_state.animation.frames[0]

        self.player = self.create_game_object(starting_image)
        self.player.add_component(RigidBody())
        self.player.transform.position = Vector2(230, 580)
        #self.player.transform.position = Vector2(4330, 580)
        #self.player.transform.position = Vector2(100, 80)
        self.player.transform.scale = Vector2(1, 1)
        self.player.renderer.depth = -10
        self.player.rigid_body.gravity_scale = 1.0
        self.player.collider.restitution = 0
        self.player.name = "player"

        self.player.collider.set_box(40, 70)
        self.player.collider.set_offset(-12, 10)

        self.player.add_script(PlayerClimbing("player climb"))
        self.player.add_script(PlayerPlatformMovement("player plat move"))
        self.player.add_script(DeactivateSaw())
        self.player.add_script(HandleLightLife())
        self.player.add_script(GoToOtherLevel())
        self.player.add_script(BookShelfInteraction())

        # add animator to player from the animation state machine
        self.player.add_component(self.player_anim_handler.animator)

    def load_elevators(self):

        path = "assets/images/platforms/"

        img_140x50 = pygame.image.load(path + "50x140.png").convert_alpha()
        img_180x50 = pygame.image.load(path + "50x180.png").convert_alpha()

        # create elevator platforms
        for i in range(0, 4):
            platform = self.create_game_object(img_140x50)
            x = 2475
            spawn_point = Vector2(x, 700)

            platform.transform.position = Vector2(x, 700 - i*200)
            set_platform_attributes(platform)
            platform.add_script(ElevatorPlatMovement(spawn_point, "elev move"))
            platform.collider.treat_as_dynamic = True

        for i in range(0, 5):
            platform = self.create_game_object(img_180x50)
            x = 3650
            spawn_point = Vector2(x, 700)

            platform.transform.position = Vector2(x, 700 - i*200)
            set_platform_attributes(platform)
            platform.add_script(ElevatorPlatMovement(spawn_point, "elev move"))
            platform.collider.treat_as_dynamic = True

        # objects to detect crates
        teleport_a = self.create_box_collider_object(200, 200)
        teleport_a.collider.is_trigger = True
        teleport_a.transform.position = Vector2(2475, -280)
        teleport_a.name = "teleport a"

        teleport_b = self.create_box_collider_object(200, 200)
        teleport_b.collider.is_trigger = True
        teleport_b.transform.position = Vector2(3650, -490)
        teleport_b.name = "teleport b"

        # the elevator shaft
        path = "assets/images/environment/elevator/"

        elev_shaft_img = pygame.image.load(path + "elevator_shaft.png").convert_alpha()
        elevator_shaft = self.create_renderable_object(elev_shaft_img)

        y = elev_shaft_img.get_width()-150

        elevator_shaft.add_component(Transform(Vector2(1100, y)))
        elevator_shaft.renderer.depth = 50

        elevator_cabin_img = pygame.image.load(path + "extended_elevator.png").convert()
        elevator_cabin = self.create_renderable_object(elevator_cabin_img)
        elevator_cabin.add_component(Transform(Vector2(1100, y + elevator_cabin_img.get_height() + 100)))
        elevator_cabin.renderer.depth = 40

        # place dormant collider on the bottom of the elevator shaft
        elevator_cabin.add_component(BoxCollider(140, 20))
        elevator_cabin.collider.is_trigger = True
        elevator_cabin.collider.surface_friction = 0.75
        elevator_cabin.tag = "cabin"

        elevator_cabin.add_script(MoveCabin())

    def load_boxes(self):
        box_img = pygame.image.load("assets/images/crates/red_green.png").convert_alpha()
        box = self.create_game_object(box_img)
        box.transform.position = Vector2(900, 560)
        set_box_attributes(box)
        box.add_script(TeleportCrate())
        self.crates.append(box)

        box_img = pygame.image.load("assets/images/crates/gold_blue.png").convert_alpha()
        box = self.create_game_object(box_img)
        box.transform.position = Vector2(540, 400)
        set_box_attributes(box)
        box.add_script(TeleportCrate())
        self.crates.append(box)

        box_img = pygame.image.load("assets/images/crates/blue_green.png").convert_alpha()
        box = self.create_game_object(box_img)
        box.transform.position = Vector2(1300, -320)
        set_box_attributes(box)
        box.add_script(TeleportCrate())
        self.crates.append(box)

        box_img = pygame.image.load("assets/images/crates/blue_red.png").convert_alpha()
        box = self.create_game_object(box_img)
        box.transform.position = Vector2(2475, 300)
        set_box_attributes(box)
        box.add_script(TeleportCrate())
        self.crates.append(box)

    def load_anims(self):

        self.player_anim_handler = AnimationStateMachine(Animator())

        path_to_anims = "assets/animations/"

        # setup the idle animation
        anim = load_anim_from_directory(path_to_anims + "Idle/")

        # set time between frames in seconds
        anim.frame_latency = 0.18

        # create a state and load it to the state machine
        state = AnimationStateMachine.AnimationState("idle", anim)
        self.player_anim_handler.add_state(state)

        # setup the walk animation
        anim = load_anim_from_directory(path_to_anims + "Walking/")
        anim.frame_latency = 0.083
        state = AnimationStateMachine.AnimationState("walking", anim)
        self.player_anim_handler.add_state(state)

        # add jump animation
        anim = load_anim_from_directory(path_to_anims + "Jumping/")
        anim.frame_latency = 0.12
        anim.cycle = False
        state = AnimationStateMachine.AnimationState("jumping", anim)
        self.player_anim_handler.add_state(state)

        # climb animation
        anim = load_anim_from_directory(path_to_anims + "Climbing/")
        anim.frame_latency = 0.12
        state = AnimationStateMachine.AnimationState("climbing", anim)
        self.player_anim_handler.add_state(state)

        self.player_anim_handler.set_current_state("idle")

        # transitions for the state machine
        walk_transition = StateMachine.Transition()
        idle_transition = StateMachine.Transition()
        jump_transition = StateMachine.Transition()
        climb_transition = StateMachine.Transition()

        # add conditions to the transitions
        # test to see if the player is moving on the x-axis
        min_speed_to_walk = 60

        walk_transition.add_condition(lambda: abs(self.player.rigid_body.velocity.x) > min_speed_to_walk)
        walk_transition.add_condition(lambda: self.player.get_script("player plat move").moving)
        walk_transition.add_condition(lambda: self.player.get_script("player plat move").grounded)
        walk_transition.add_condition(lambda: not self.player.get_script("player climb").climbing)

        idle_transition.add_condition(lambda: abs(self.player.rigid_body.velocity.x) < min_speed_to_walk or not self.player.get_script("player plat move").moving)
        idle_transition.add_condition(lambda: self.player.get_script("player plat move").grounded)
        idle_transition.add_condition(lambda: not self.player.get_script("player climb").climbing)

        jump_transition.add_condition(lambda: not self.player.get_script("player plat move").grounded)
        jump_transition.add_condition(lambda: not self.player.get_script("player climb").climbing)

        climb_transition.add_condition(lambda: self.player.get_script("player climb").colliding_with_ladder())
        climb_transition.add_condition(lambda: self.player.get_script("player climb").climbing)

        # set up transitions between states
        self.player_anim_handler.add_bi_transition("idle", "walking", walk_transition, idle_transition)

        self.player_anim_handler.add_bi_transition("idle", "jumping", jump_transition, idle_transition)
        self.player_anim_handler.add_bi_transition("walking", "jumping", jump_transition, walk_transition)

        self.player_anim_handler.add_bi_transition("idle", "climbing", climb_transition, idle_transition)
        self.player_anim_handler.add_bi_transition("walking", "climbing", climb_transition, walk_transition)
        self.player_anim_handler.add_bi_transition("jumping", "climbing", climb_transition, jump_transition)

    def create_ladder(self, ladder_body, ladder_top, height, x, y):
        img = create_img_from_tile(ladder_body, ladder_body.get_width(), height)

        # pad image on top of the ladder
        img = conjoin_surfaces_vertically(ladder_top, img)

        ladder = self.create_game_object(img)
        ladder.collider.is_trigger = True
        ladder.transform.position = Vector2(x, y)

        new_w = ladder.collider.box.w - 90
        new_h = ladder.collider.box.h - 50
        ladder.collider.set_box(new_w, new_h)

        ladder.renderer.depth = 3

        ladder.tag = "ladder"

        self.ladders.append(ladder)

    def initialize_monster(self):

        render_sys = self.get_system(RenderSystem.tag)

        img = pygame.image.load("assets/images/environment/hazards/monster.png").convert_alpha()
        w = img.get_width()
        h = img.get_height()
        pivot = Vector2(w/2, h/2)

        self.monster.add_component(Renderer(img, pivot))
        self.monster.add_component(BoxCollider(50, 50))
        self.monster.add_script(MonsterMovement())
        self.monster.collider.is_trigger = True
        self.monster.renderer.depth = -10

        # insert to scene
        render_sys.dynamic_insertion_to_scene(self.monster)
        render_sys.light_sources.append(self.monster_light)

    # Make the monster invisible and unable to interact with
    def disable_monster(self):
        # remove from the render system
        render_sys = self.get_system(RenderSystem.tag)
        render_sys.remove_from_scene(self.monster)
        render_sys.remove_from_scene(self.monster_light)
        render_sys.light_sources.remove(self.monster_light)

        self.monster.remove_component(Renderer.tag)
        self.monster.remove_component(BoxCollider.tag)
        self.monster.remove_script("monster movement")

    # test to see if the entity is considered as ground in the world
    def is_ground(self, entity):
        e = entity
        return e.tag == "floor" or e.tag == "box" or e.tag == "cabin" or e.tag == "wall" or e.tag == "platform"

# p = PlatformWorld()
# engine.set_world(p)
# engine.worlds.append(p)
# engine.run()
#
