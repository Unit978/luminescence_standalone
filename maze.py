
from world import *
from engine import *
from components import BehaviorScript
from scripts import CameraFollow

Engine(10, 10)

scale_x = 56  # original 56
scale_y = 100  # original 100
tile = pygame.image.load("assets/images/tiles/56x100 tile.png").convert()
off_switch_state_on = pygame.image.load("assets/images/tiles/56x100_switchOFF.png").convert()
off_switch_state_off = pygame.image.load("assets/images/tiles/56x100_switchNORM.png").convert()
on_switch = pygame.image.load("assets/images/tiles/56x100_switchON.png").convert()

player_image_north = pygame.image.load("assets/images/character/character_north.png").convert_alpha()
player_image_south = pygame.image.load("assets/images/character/character_south.png").convert_alpha()
player_image_east = pygame.image.load("assets/images/character/character_east.png").convert_alpha()
player_image_west = pygame.image.load("assets/images/character/character_west.png").convert_alpha()


player_image_northeast = pygame.image.load("assets/images/character/character_northeast.png").convert_alpha()
player_image_northwest = pygame.image.load("assets/images/character/character_northwest.png").convert_alpha()
player_image_southeast = pygame.image.load("assets/images/character/character_southeast.png").convert_alpha()
player_image_southwest = pygame.image.load("assets/images/character/character_southwest.png").convert_alpha()


lamp_light_img = pygame.image.load("assets/images/lights/lamp_light_1200x700.png").convert_alpha()

bump_sound = mixer.Sound("assets/sound/bump.WAV")
block_removed = mixer.Sound("assets/sound/dooropen.WAV")
blocked_wall = mixer.Sound("assets/sound/effect_ice1.WAV")
puzzle_finished_sfx = mixer.Sound("assets/sound/piano_low_key.wav")
bump_sound.set_volume(0.2)
block_removed.set_volume(0.3)
blocked_wall.set_volume(0.3)


def create_blocked_wall(c1, c2):
        lever = pygame.Surface(((c2[0]-c1[0])*scale_x, (c2[1]-c1[1])*scale_y)).convert()
        lever.fill((0, 255, 0))
        return lever


def create_lever(c1, c2):
        lever = pygame.Surface(((c2[0]-c1[0])*scale_x, (c2[1]-c1[1])*scale_y)).convert()
        lever.fill((255, 0, 0))
        return lever


def create_wall(c1, c2):
        wall = pygame.Surface(((c2[0]-c1[0])*scale_x, (c2[1]-c1[1])*scale_y)).convert()
        wall.fill((255, 255, 255))
        return wall


def find_coordinate(c):
        coordinate = c[0]*scale_x, c[1]*scale_y
        return coordinate


class LightFollow(WorldScript):

    def __init__(self):
        super(LightFollow, self).__init__("light follow")

    def update(self):

        # make the lamp mask follow the player
        self.world.lamp_mask.transform.position = self.world.player.transform.position


class PlayerMovement(BehaviorScript):

    def __init__(self, script_name):
        super(PlayerMovement, self).__init__(script_name)
        self.speed = 300.0

    def update(self):
        keys = pygame.key.get_pressed()

        velocity = self.entity.rigid_body.velocity

        if keys[pygame.K_a]:
            velocity.x = -self.speed
            self.entity.renderer.sprite = player_image_west
        elif keys[pygame.K_d]:
            velocity.x = self.speed
            self.entity.renderer.sprite = player_image_east
        else:
            velocity.x = 0

        if keys[pygame.K_w]:
            velocity.y = -self.speed
            self.entity.renderer.sprite = player_image_north
        elif keys[pygame.K_s]:
            velocity.y = self.speed
            self.entity.renderer.sprite = player_image_south
        else:
            velocity.y = 0

        if keys[pygame.K_w] and keys[pygame.K_d]:
            self.entity.renderer.sprite = player_image_northeast

        elif keys[pygame.K_w] and keys[pygame.K_a]:
            self.entity.renderer.sprite = player_image_northwest

        elif keys[pygame.K_s] and keys[pygame.K_d]:
            self.entity.renderer.sprite = player_image_southeast

        elif keys[pygame.K_s] and keys[pygame.K_a]:
            self.entity.renderer.sprite = player_image_southwest

    def collision_event(self, other_collider):

        # if the player hit the exit trigger to get out of the maze
        if other_collider.entity == self.entity.world.exit_object_trigger:

            # player falls into the maze if he comes back
            self.entity.transform.position.zero()

            # take the player back to the main room
            self.entity.world.engine.game.go_to_main()

            self.entity.world.puzzle = True


class Maze(World):
    def __init__(self):
        super(Maze, self).__init__()

        self.player = None

        # border walls
        self.topWall = None
        self.leftWall = None
        self.rightWall = None
        self.bottomWall = None

        # location
        self.coordinate = None

        # levers
        self.lever1 = None
        self.lever2 = None
        self.lever3 = None
        self.lever4 = None
        self.lever5 = None
        self.lever6 = None
        self.lever7 = None

        # blocked path walls for each lever
        self.blocked1 = None
        self.blocked2 = None
        self.blocked3 = None
        self.blocked4 = None
        self.blocked5 = None
        self.blocked6 = None
        self.blocked7 = None

        # static maze walls
        self.new_wall = None
        # lamp for sprite
        self.lamp_mask = None

        # if puzzle is complete
        self.puzzle = False

        # this object signals that the player completed the puzzle and can exit the maze
        self.exit_object_trigger = None

    def resume(self):
        mixer.music.load("assets/music/VoiceInMyHead.ogg")
        mixer.music.play(-1)
        mixer.music.set_volume(0.3)

    def construct_blocked_walls(self):
        l1c = (12, 4)
        self.blocked1 = self.create_game_object(tile)
        self.blocked1.tag = "blocked1"
        _l1c = find_coordinate(l1c)
        self.blocked1.transform.position = Vector2(_l1c[0], _l1c[1])

        l2c = (24, 6)
        self.blocked2 = self.create_game_object(tile)
        self.blocked2.tag = "blocked2"
        _l2c = find_coordinate(l2c)
        self.blocked2.transform.position = Vector2(_l2c[0], _l2c[1])

        l3c = (15, 11)
        self.blocked3 = self.create_game_object(tile)
        self.blocked3.tag = "blocked3"
        _l3c = find_coordinate(l3c)
        self.blocked3.transform.position = Vector2(_l3c[0], _l3c[1])

        l4c = (4, 12)
        self.blocked4 = self.create_game_object(tile)
        self.blocked4.tag = "blocked4"
        _l4c = find_coordinate(l4c)
        self.blocked4.transform.position = Vector2(_l4c[0], _l4c[1])

        l5c = (1, 7)
        self.blocked5 = self.create_game_object(tile)
        self.blocked5.tag = "blocked5"
        _l5c = find_coordinate(l5c)
        self.blocked5.transform.position = Vector2(_l5c[0], _l5c[1])

        l6c = (23, 7)
        self.blocked6 = self.create_game_object(tile)
        self.blocked6.tag = "blocked6"
        _l6c = find_coordinate(l6c)
        self.blocked6.transform.position = Vector2(_l6c[0], _l6c[1])

        l7c = (-1, 15)
        self.blocked7 = self.create_game_object(tile)
        self.blocked7.tag = "blocked7"
        _l7c = find_coordinate(l7c)
        self.blocked7.transform.position = Vector2(_l7c[0], _l7c[1])

    def construct_off_levers(self):

        animation = Animator.Animation()
        animation.add_frame(off_switch_state_off)
        animation.add_frame(off_switch_state_on)
        animation.frame_latency = 0.75

        animator_1 = Animator()
        animator_1.current_animation = animation

        animator_2 = Animator()
        animator_2.current_animation = animation

        animator_3 = Animator()
        animator_3.current_animation = animation

        animator_4 = Animator()
        animator_4.current_animation = animation

        animator_5 = Animator()
        animator_5.current_animation = animation

        animator_6 = Animator()
        animator_6.current_animation = animation

        animation_1 = Animator.Animation()
        animation_1.add_frame(off_switch_state_off)
        animation_1.add_frame(off_switch_state_on)
        animation_1.frame_latency = 0.5

        animator_7 = Animator()
        animator_7.current_animation = animation_1

        l1c = (14, 7)
        # self.lever1 = self.create_game_object(create_lever(l1c, (l1c[0]+1, l1c[1]+1)))
        self.lever1 = self.create_game_object(off_switch_state_off)
        self.lever1.add_component(animator_1)
        self.lever1.tag = "lever1_off"
        _l1c = find_coordinate(l1c)
        self.lever1.transform.position = Vector2(_l1c[0], _l1c[1])

        l2c = (23, 0)
        # self.lever2 = self.create_game_object(create_lever(l2c, (l2c[0]+1, l2c[1]+1)))
        self.lever2 = self.create_game_object(off_switch_state_off)
        self.lever2.add_component(animator_2)
        self.lever2.tag = "lever2_off"
        _l2c = find_coordinate(l2c)
        self.lever2.transform.position = Vector2(_l2c[0], _l2c[1])

        l3c = (16, 13)
        # self.lever3 = self.create_game_object(create_lever(l3c, (l3c[0]+1, l3c[1]+1)))
        self.lever3 = self.create_game_object(off_switch_state_off)
        self.lever3.add_component(animator_3)
        self.lever3.tag = "lever3_off"
        _l3c = find_coordinate(l3c)
        self.lever3.transform.position = Vector2(_l3c[0], _l3c[1])

        l4c = (5, 13)
        # self.lever4 = self.create_game_object(create_lever(l4c, (l4c[0]+1, l4c[1]+1)))
        self.lever4 = self.create_game_object(off_switch_state_off)
        self.lever4.add_component(animator_4)
        self.lever4.tag = "lever4_off"
        _l4c = find_coordinate(l4c)
        self.lever4.transform.position = Vector2(_l4c[0], _l4c[1])

        l5c = (0, 6)
        # self.lever5 = self.create_game_object(create_lever(l5c, (l5c[0]+1, l5c[1]+1)))
        self.lever5 = self.create_game_object(off_switch_state_off)
        self.lever5.add_component(animator_5)
        self.lever5.tag = "lever5_off"
        _l5c = find_coordinate(l5c)
        self.lever5.transform.position = Vector2(_l5c[0], _l5c[1])

        l6c = (35, 10)
        self.lever6 = self.create_game_object(off_switch_state_off)
        self.lever6.add_component(animator_6)
        self.lever6.tag = "lever6_off"
        _l6c = find_coordinate(l6c)
        self.lever6.transform.position = Vector2(_l6c[0], _l6c[1])

        # coordinates.append((6, 8))
        l7c = (6, 8)
        self.lever7 = self.create_game_object(off_switch_state_off)
        self.lever7.add_component(animator_7)
        self.lever7.tag = "lever7_off"
        _l7c = find_coordinate(l7c)
        self.lever7.transform.position = Vector2(_l7c[0], _l7c[1])

    def construct_on_lever(self):
        l1c = (14, 7)
        self.lever1 = self.create_game_object(on_switch)
        self.lever1.tag = "lever1_on"
        _l1c = find_coordinate(l1c)
        self.lever1.transform.position = Vector2(_l1c[0], _l1c[1])

        l2c = (23, 0)
        self.lever2 = self.create_game_object(on_switch)
        self.lever2.tag = "lever2_on"
        _l2c = find_coordinate(l2c)
        self.lever2.transform.position = Vector2(_l2c[0], _l2c[1])

        l3c = (16, 13)
        self.lever3 = self.create_game_object(on_switch)
        self.lever3.tag = "lever3_on"
        _l3c = find_coordinate(l3c)
        self.lever3.transform.position = Vector2(_l3c[0], _l3c[1])

        l4c = (5, 13)
        self.lever4 = self.create_game_object(on_switch)
        self.lever4.tag = "lever4_on"
        _l4c = find_coordinate(l4c)
        self.lever4.transform.position = Vector2(_l4c[0], _l4c[1])

        l5c = (0, 6)
        self.lever5 = self.create_game_object(on_switch)
        self.lever5.tag = "lever5_on"
        _l5c = find_coordinate(l5c)
        self.lever5.transform.position = Vector2(_l5c[0], _l5c[1])

        l6c = (35, 10)
        self.lever6 = self.create_game_object(on_switch)
        self.lever6.tag = "lever6_on"
        _l6c = find_coordinate(l6c)
        self.lever6.transform.position = Vector2(_l6c[0], _l6c[1])

        l7c = (6, 8)
        self.lever7 = self.create_game_object(off_switch_state_off)
        self.lever7.tag = "lever7_on"
        _l7c = find_coordinate(l7c)
        self.lever7.transform.position = Vector2(_l7c[0], _l7c[1])

    def construct_wall(self, c):
        # self.new_wall = self.create_game_object(create_wall(c, (c[0]+1, c[1]+1)))
        self.new_wall = self.create_game_object(tile)
        self.new_wall.tag = "wall"
        c1 = find_coordinate(c)
        self.new_wall.transform.position = Vector2(c1[0], c1[1])

    def end_path(self):

        # play the sound the effect to let the player know that the puzzle was completed
        puzzle_finished_sfx.play()

        self.destroy_entity(self.blocked7)
        vertical_beam = pygame.image.load("assets/images/tiles/vertical_beam.png").convert_alpha()

        new_wall = self.create_renderable_object(vertical_beam)
        c = find_coordinate((-1, 8))
        new_wall.transform.position = Vector2(c[0], c[1])

        new_wall = self.create_renderable_object(vertical_beam)
        c = find_coordinate((-1, 9))
        new_wall.transform.position = Vector2(c[0], c[1])

        new_wall = self.create_renderable_object(vertical_beam)
        c = find_coordinate((-1, 10))
        new_wall.transform.position = Vector2(c[0], c[1])

        new_wall = self.create_renderable_object(vertical_beam)
        c = find_coordinate((-1, 11))
        new_wall.transform.position = Vector2(c[0], c[1])

        new_wall = self.create_renderable_object(vertical_beam)
        c = find_coordinate((-1, 12))
        new_wall.transform.position = Vector2(c[0], c[1])

        new_wall = self.create_renderable_object(vertical_beam)
        c = find_coordinate((-1, 13))
        new_wall.transform.position = Vector2(c[0], c[1])

        new_wall = self.create_renderable_object(vertical_beam)
        c = find_coordinate((-1, 14))
        new_wall.transform.position = Vector2(c[0], c[1])

        new_wall = self.create_renderable_object(vertical_beam)
        c = find_coordinate((8, 8))
        new_wall.transform.position = Vector2(c[0], c[1])

        new_wall = self.create_renderable_object(vertical_beam)
        c = find_coordinate((8, 9))
        new_wall.transform.position = Vector2(c[0], c[1])

        new_wall = self.create_renderable_object(vertical_beam)
        c = find_coordinate((3, 9))
        new_wall.transform.position = Vector2(c[0], c[1])

        new_wall = self.create_renderable_object(vertical_beam)
        c = find_coordinate((3, 8))
        new_wall.transform.position = Vector2(c[0], c[1])

        horizontal_beam = pygame.image.load("assets/images/tiles/horizontal_beam.png").convert_alpha()

        new_wall = self.create_renderable_object(horizontal_beam)
        c = find_coordinate((0, 7))
        new_wall.transform.position = Vector2(c[0], c[1])

        new_wall = self.create_renderable_object(horizontal_beam)
        c = find_coordinate((1, 7))
        new_wall.transform.position = Vector2(c[0], c[1])

        new_wall = self.create_renderable_object(horizontal_beam)
        c = find_coordinate((2, 7))
        new_wall.transform.position = Vector2(c[0], c[1])

        new_wall = self.create_renderable_object(horizontal_beam)
        c = find_coordinate((4, 10))
        new_wall.transform.position = Vector2(c[0], c[1])

        new_wall = self.create_renderable_object(horizontal_beam)
        c = find_coordinate((5, 10))
        new_wall.transform.position = Vector2(c[0], c[1])

        new_wall = self.create_renderable_object(horizontal_beam)
        c = find_coordinate((6, 10))
        new_wall.transform.position = Vector2(c[0], c[1])

        new_wall = self.create_renderable_object(horizontal_beam)
        c = find_coordinate((7, 10))
        new_wall.transform.position = Vector2(c[0], c[1])

    def load_scene(self):

        img_width = lamp_light_img.get_width()
        img_height = lamp_light_img.get_height()

        pivot = Vector2(img_width/2, img_height/2)

        # add necessary components to be able to position and render the background
        self.lamp_mask = self.create_entity()
        self.lamp_mask.add_component(Transform(Vector2(0, 0)))
        self.lamp_mask.add_component(Renderer(lamp_light_img, pivot))
        self.lamp_mask.renderer.depth = -100

        self.get_system(PhysicsSystem.tag).gravity.zero()
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

        # add necessary components to be able to position and render the background
        floor_image = pygame.image.load("assets/images/floors/WoodenFloor.png").convert()

        # add necessary components to be able to position and render
        # the background

        # ========================================Floor====================================================

        floor_1 = self.create_entity()
        floor_coordinate = find_coordinate((-2, -2))
        floor_1.add_component(Transform(Vector2(floor_coordinate[0], floor_coordinate[1])))
        floor_1.add_component(Renderer(floor_image))
        floor_1.renderer.depth = 100

        floor_2 = self.create_entity()
        floor_coordinate = find_coordinate((21, -2))
        floor_2.add_component(Transform(Vector2(floor_coordinate[0], floor_coordinate[1])))
        floor_2.add_component(Renderer(floor_image))
        floor_2.renderer.depth = 100

        floor_3 = self.create_entity()
        floor_coordinate = find_coordinate((-2, 5))
        floor_3.add_component(Transform(Vector2(floor_coordinate[0], floor_coordinate[1])))
        floor_3.add_component(Renderer(floor_image))
        floor_3.renderer.depth = 100

        floor_4 = self.create_entity()
        floor_coordinate = find_coordinate((21, 5))
        floor_4.add_component(Transform(Vector2(floor_coordinate[0], floor_coordinate[1])))
        floor_4.add_component(Renderer(floor_image))
        floor_4.renderer.depth = 100

        floor_5 = self.create_entity()
        floor_coordinate = find_coordinate((-2, 8))
        floor_5.add_component(Transform(Vector2(floor_coordinate[0], floor_coordinate[1])))
        floor_5.add_component(Renderer(floor_image))
        floor_5.renderer.depth = 100

        floor_6 = self.create_entity()
        floor_coordinate = find_coordinate((21, 8))
        floor_6.add_component(Transform(Vector2(floor_coordinate[0], floor_coordinate[1])))
        floor_6.add_component(Renderer(floor_image))
        floor_6.renderer.depth = 100

        floor_7 = self.create_entity()
        floor_coordinate = find_coordinate((17, -2))
        floor_7.add_component(Transform(Vector2(floor_coordinate[0], floor_coordinate[1])))
        floor_7.add_component(Renderer(floor_image))
        floor_7.renderer.depth = 100

        floor_8 = self.create_entity()
        floor_coordinate = find_coordinate((17, 5))
        floor_8.add_component(Transform(Vector2(floor_coordinate[0], floor_coordinate[1])))
        floor_8.add_component(Renderer(floor_image))
        floor_8.renderer.depth = 100

        floor_9 = self.create_entity()
        floor_coordinate = find_coordinate((17, 8))
        floor_9.add_component(Transform(Vector2(floor_coordinate[0], floor_coordinate[1])))
        floor_9.add_component(Renderer(floor_image))
        floor_9.renderer.depth = 100

        # =========================================Create Player====================================
        self.player = self.create_game_object(player_image_north)
        self.player.add_component(RigidBody())
        self.player.transform.position = Vector2(0, 0)
        self.player.renderer.depth = -10
        self.player.rigid_body.gravity_scale = 1
        self.player.add_script(PlayerMovement("player_move"))
        self.player.collider.restitution = 1
        self.player.collider.set_box(30, 30)

        # ==================================== Trigger Exit Object ===============================
        self.exit_object_trigger = self.create_box_collider_object(500, 200)
        self.exit_object_trigger.transform.position = Vector2(0, 1650)
        self.exit_object_trigger.collider.is_trigger = True

        # =============================================Static Maze Tiles==========================
        coordinates = list()
        coordinates.append((0, 1))  # 1
        coordinates.append((0, 2))  # 2
        coordinates.append((0, 3))  # 3
        coordinates.append((0, 4))  # 4
        coordinates.append((1, 4))  # 5
        coordinates.append((2, 4))  # 6
        coordinates.append((2, 5))  # 7
        coordinates.append((3, 6))  # 8
        coordinates.append((4, 6))  # 9
        coordinates.append((5, 6))  # 10
        coordinates.append((6, 6))  # 11
        coordinates.append((2, 1))  # 12
        coordinates.append((2, 2))  # 13
        coordinates.append((3, 2))  # 14
        coordinates.append((4, 2))  # 15
        coordinates.append((4, 3))  # 16
        coordinates.append((4, 4))  # 17
        coordinates.append((5, 4))  # 18
        coordinates.append((7, 6))  # 19
        coordinates.append((7, 5))  # 20
        coordinates.append((7, 4))  # 21
        coordinates.append((7, 3))  # 22
        coordinates.append((6, 2))  # 23
        coordinates.append((7, 2))  # 24
        coordinates.append((8, 2))  # 25
        coordinates.append((4, 1))  # 26
        coordinates.append((4, 0))  # 27
        coordinates.append((5, 0))  # 28
        coordinates.append((6, 0))  # 29
        coordinates.append((7, 0))  # 30
        coordinates.append((8, 0))  # 31
        coordinates.append((9, 0))  # 32
        coordinates.append((10, 0))  # 33
        coordinates.append((10, 1))  # 34
        coordinates.append((10, 2))  # 35
        coordinates.append((10, 3))  # 36
        coordinates.append((10, 4))  # 37
        coordinates.append((9, 4))   # 38
        coordinates.append((8, 6))   # 39
        coordinates.append((9, 6))   # 40
        coordinates.append((10, 6))  # 41
        coordinates.append((10, 7))  # 42
        coordinates.append((10, 8))  # 43
        coordinates.append((10, 9))  # 44
        coordinates.append((10, 10))  # 45
        coordinates.append((11, 10))  # 46
        coordinates.append((12, 10))  # 47
        coordinates.append((13, 10))  # 48
        coordinates.append((14, 10))  # 49
        coordinates.append((15, 10))  # 50
        coordinates.append((16, 10))  # 51
        coordinates.append((17, 10))  # 52
        coordinates.append((17, 9))  # 53
        coordinates.append((17, 8))  # 54
        coordinates.append((17, 7))  # 55
        coordinates.append((17, 6))  # 56
        coordinates.append((17, 5))  # 57
        coordinates.append((17, 4))  # 58
        coordinates.append((11, 4))  # 59
        coordinates.append((12, 5))  # 60
        coordinates.append((12, 6))  # 61
        coordinates.append((12, 7))  # 62
        coordinates.append((13, 7))  # 63
        coordinates.append((12, 8))  # 64
        coordinates.append((13, 8))  # 65
        coordinates.append((14, 8))  # 66
        coordinates.append((15, 8))  # 67
        coordinates.append((15, 7))  # 68
        coordinates.append((15, 6))  # 69
        coordinates.append((15, 5))  # 70
        coordinates.append((14, 5))  # 71
        coordinates.append((13, 3))  # 72
        coordinates.append((14, 3))  # 73
        coordinates.append((15, 3))  # 74
        coordinates.append((16, 3))  # 75
        coordinates.append((13, 2))  # 76
        coordinates.append((14, 2))  # 77
        coordinates.append((15, 2))  # 78
        coordinates.append((16, 2))  # 79
        coordinates.append((11, 3))  # 80
        coordinates.append((11, 2))  # 81
        coordinates.append((11, 1))  # 82
        coordinates.append((11, 0))  # 83
        coordinates.append((12, 0))  # 84
        coordinates.append((13, 0))  # 85
        coordinates.append((14, 0))  # 86
        coordinates.append((15, 0))  # 87
        coordinates.append((16, 0))  # 88
        coordinates.append((17, 0))  # 89
        coordinates.append((18, 0))  # 90
        coordinates.append((18, 1))  # 91
        coordinates.append((18, 2))  # 92
        coordinates.append((19, 2))  # 93
        coordinates.append((19, 3))  # 94
        coordinates.append((19, 4))  # 95
        coordinates.append((19, 5))  # 96
        coordinates.append((20, 5))  # 97
        coordinates.append((21, 5))  # 98
        coordinates.append((22, 5))  # 99
        coordinates.append((18, 7))  # 100
        coordinates.append((19, 7))  # 101
        coordinates.append((20, 7))  # 102
        coordinates.append((21, 7))  # 103
        coordinates.append((22, 7))  # 104
        coordinates.append((24, 7))  # 105
        coordinates.append((24, 5))  # 107
        coordinates.append((24, 4))  # 108
        coordinates.append((24, 3))  # 109
        coordinates.append((23, 3))  # 110
        coordinates.append((22, 3))  # 111
        coordinates.append((21, 3))  # 112
        coordinates.append((19, 1))  # 113
        coordinates.append((20, 1))  # 114
        coordinates.append((21, 1))  # 115
        coordinates.append((22, 1))  # 116
        coordinates.append((24, 2))  # 117
        coordinates.append((24, 1))  # 118
        coordinates.append((22, 0))  # 119
        coordinates.append((24, 0))  # 120
        coordinates.append((22, 8))  # 121
        coordinates.append((22, 9))  # 122
        coordinates.append((22, 10))  # 123
        coordinates.append((21, 10))  # 124
        coordinates.append((20, 10))  # 125
        coordinates.append((19, 10))  # 126
        coordinates.append((18, 10))  # 127
        coordinates.append((24, 8))   # 128
        coordinates.append((24, 9))   # 129
        coordinates.append((24, 10))  # 130
        coordinates.append((24, 11))  # 131
        coordinates.append((24, 12))  # 132
        coordinates.append((23, 12))  # 133
        coordinates.append((22, 12))  # 134
        coordinates.append((21, 12))  # 135
        coordinates.append((20, 12))  # 136
        coordinates.append((19, 12))   # 137
        coordinates.append((18, 12))   # 138
        coordinates.append((17, 12))   # 139
        coordinates.append((15, 12))   # 141
        coordinates.append((14, 12))   # 142
        coordinates.append((13, 12))   # 143
        coordinates.append((12, 12))   # 144
        coordinates.append((11, 12))   # 145
        coordinates.append((10, 12))   # 146
        coordinates.append((9, 13))   # 147
        coordinates.append((9, 10))   # 148
        coordinates.append((8, 11))   # 149
        coordinates.append((7, 11))   # 150
        coordinates.append((8, 13))   # 151
        coordinates.append((7, 13))   # 152
        coordinates.append((6, 13))   # 153
        coordinates.append((6, 11))   # 154
        coordinates.append((5, 11))   # 155
        coordinates.append((4, 13))   # 157
        coordinates.append((3, 13))   # 158
        coordinates.append((2, 13))   # 159
        coordinates.append((1, 13))   # 160
        coordinates.append((0, 13))   # 161
        coordinates.append((3, 11))   # 162
        coordinates.append((2, 11))   # 163
        coordinates.append((1, 11))   # 164
        coordinates.append((1, 10))   # 165
        coordinates.append((1, 9))   # 166
        coordinates.append((1, 8))   # 167
        coordinates.append((1, 6))   # 169
        coordinates.append((1, 5))   # 170
        coordinates.append((0, 5))   # 171
        coordinates.append((2, 6))   # 172
        coordinates.append((2, 8))   # 173
        coordinates.append((2, 9))   # 174
        coordinates.append((2, 10))   # 175
        coordinates.append((4, 7))   # 176
        coordinates.append((4, 8))   # 177
        coordinates.append((4, 9))   # 178
        coordinates.append((5, 9))   # 179
        coordinates.append((6, 9))   # 180
        coordinates.append((7, 9))   # 181
        coordinates.append((7, 8))   # 182
        coordinates.append((4, 11))
        coordinates.append((23, -1))
        coordinates.append((25, 5))
        coordinates.append((26, 5))
        coordinates.append((28, 5))
        coordinates.append((28, 4))
        coordinates.append((28, 3))
        coordinates.append((28, 2))
        coordinates.append((28, 1))
        coordinates.append((28, 0))
        coordinates.append((28, -1))
        coordinates.append((26, 4))
        coordinates.append((26, 2))
        coordinates.append((26, 1))
        coordinates.append((25, -1))
        coordinates.append((27, 0))
        coordinates.append((26, 6))
        coordinates.append((26, 7))
        coordinates.append((26, 8))
        coordinates.append((27, 8))
        coordinates.append((28, 8))
        coordinates.append((28, 9))
        coordinates.append((28, 10))
        coordinates.append((26, 10))
        coordinates.append((26, 11))
        coordinates.append((26, 12))
        coordinates.append((27, 12))
        coordinates.append((26, 13))
        coordinates.append((24, 14))
        coordinates.append((22, 13))
        coordinates.append((20, 14))
        coordinates.append((18, 13))
        coordinates.append((17, 13))
        coordinates.append((17, 14))
        coordinates.append((27, 13))
        coordinates.append((28, 13))
        coordinates.append((29, 13))
        coordinates.append((29, 12))
        coordinates.append((30, 12))
        coordinates.append((30, 11))
        coordinates.append((30, 10))
        coordinates.append((30, 9))
        coordinates.append((31, 9))
        coordinates.append((30, 7))
        coordinates.append((31, 7))
        coordinates.append((32, 7))
        coordinates.append((32, 6))
        coordinates.append((32, 5))
        coordinates.append((32, 4))
        coordinates.append((32, 3))
        coordinates.append((32, 2))
        coordinates.append((31, 1))
        coordinates.append((31, 0))
        coordinates.append((32, 0))
        coordinates.append((33, 0))
        coordinates.append((34, 0))
        coordinates.append((35, 1))
        coordinates.append((36, 1))
        coordinates.append((36, 2))
        coordinates.append((36, 3))
        coordinates.append((35, 3))
        coordinates.append((34, 3))
        coordinates.append((34, 4))
        coordinates.append((34, 5))
        coordinates.append((34, 6))
        coordinates.append((34, 7))
        coordinates.append((35, 7))
        coordinates.append((36, 7))
        coordinates.append((39, 0))
        coordinates.append((40, 0))
        coordinates.append((38, 1))
        coordinates.append((38, 2))
        coordinates.append((38, 3))
        coordinates.append((38, 4))
        coordinates.append((39, 5))
        coordinates.append((38, 5))
        coordinates.append((37, 5))
        coordinates.append((36, 5))
        coordinates.append((39, 6))
        coordinates.append((39, 7))
        coordinates.append((39, 8))
        coordinates.append((39, 9))
        coordinates.append((39, 10))
        coordinates.append((39, 11))
        coordinates.append((39, 12))
        coordinates.append((39, 13))
        coordinates.append((38, 13))
        coordinates.append((37, 13))
        coordinates.append((36, 13))
        coordinates.append((35, 13))
        coordinates.append((34, 13))
        coordinates.append((33, 13))
        coordinates.append((32, 13))
        coordinates.append((31, 13))
        coordinates.append((30, 13))
        coordinates.append((39, 8))
        coordinates.append((36, 8))
        coordinates.append((36, 9))
        coordinates.append((36, 10))
        coordinates.append((36, 11))
        coordinates.append((35, 11))
        coordinates.append((34, 11))
        coordinates.append((33, 11))
        coordinates.append((32, 11))
        coordinates.append((32, 9))
        coordinates.append((33, 9))
        coordinates.append((34, 9))
        coordinates.append((35, 9))
        coordinates.append((-1, 1))

        # =========================================Perimeter=======================================

        # top perimeter
        for i in range(-1, 42):
            coordinates.append((i, -2))
        # bottom perimeter
        for i in range(0, 42):
            coordinates.append((i, 15))
        # left perimeter wall
        for i in range(-2, 16):
            coordinates.append((-2, i))

        # right perimeter wall
        for i in range(-2, 16):
            coordinates.append((42, i))

        self.player.add_script(PlayerBehavior("player behavior"))

        # add camera
        render = self.get_system(RenderSystem.tag)
        render.camera = self.create_entity()
        render.camera.add_component(Transform(Vector2(0, 0)))
        render.camera.add_script(CameraFollow("camera follow", self.player.transform, w, h))
        # ====================================Construct Maze======================================
        # create levers to be triggered by player

        # static walls for maze
        for i in coordinates:
            self.construct_wall(i)
        # construct on levers first after off lever is destroyed
        self.construct_on_lever()

        self.construct_off_levers()

        # create blocked path walls
        self.construct_blocked_walls()
        self.add_script(LightFollow())

        # start the background music and set it to loop forever
        mixer.music.play(-1)
        mixer.music.set_volume(0.3)


class PlayerBehavior (BehaviorScript):
    def __init__(self, script_name):
        super(PlayerBehavior, self).__init__(script_name)
        self.touched_lever1 = False
        self.touched_lever2 = False
        self.touched_lever3 = False
        self.touched_lever4 = False
        self.touched_lever5 = False
        self.touched_lever6 = False
        self.touched_lever7 = False

    def collision_event(self, other_collider):
        other_entity = other_collider.entity
        # hits lever 1-7
        if other_entity.tag == "lever1_off":
            # print "You hit lever 1!"
            self.touched_lever1 = True
            self.entity.world.destroy_entity(other_entity)
        elif other_entity.tag == "lever2_off":
            # print "You hit lever 2!"
            self.touched_lever2 = True
            self.entity.world.destroy_entity(other_entity)
        elif other_entity.tag == "lever3_off":
            # print "You hit lever 3!"
            self.touched_lever3 = True
            self.entity.world.destroy_entity(other_entity)
        elif other_entity.tag == "lever4_off":
            # print "You hit lever 4!"
            self.touched_lever4 = True
            self.entity.world.destroy_entity(other_entity)
        elif other_entity.tag == "lever5_off":
            # print "You hit lever 5!"
            self.touched_lever5 = True
            self.entity.world.destroy_entity(other_entity)
        elif other_entity.tag == "lever6_off":
            # print "You hit lever 6!"
            self.touched_lever6 = True
            self.entity.world.destroy_entity(other_entity)
        elif other_entity.tag == "lever7_off":
            # print "You hit lever 7!"
            self.entity.world.end_path()
            self.touched_lever7 = True

        if other_entity.tag == "blocked1" and self.touched_lever1 is True:
            self.entity.world.destroy_entity(other_entity)
            block_removed.play()
        elif other_entity.tag == "blocked2" and self.touched_lever2 is True:
            self.entity.world.destroy_entity(other_entity)
            block_removed.play()
        elif other_entity.tag == "blocked3" and self.touched_lever3 is True:
            self.entity.world.destroy_entity(other_entity)
            block_removed.play()
        elif other_entity.tag == "blocked4" and self.touched_lever4 is True:
            self.entity.world.destroy_entity(other_entity)
            block_removed.play()
        elif other_entity.tag == "blocked5" and self.touched_lever5 is True:
            self.entity.world.destroy_entity(other_entity)
            block_removed.play()
        elif other_entity.tag == "blocked6" and self.touched_lever6 is True:
            self.entity.world.destroy_entity(other_entity)
            block_removed.play()
        elif other_entity.tag == "blocked7" and self.touched_lever7 is True:
            self.entity.world.destroy_entity(other_entity)
            block_removed.play()
            self.entity.world.puzzle = True
            if self.entity.world.puzzle is True:
                print "puzzle is complete"
            else:
                print "puzzle is not complete"

        if other_entity.tag == "blocked1" and self.touched_lever1 is False:
            blocked_wall.play()
        elif other_entity.tag == "blocked2" and self.touched_lever2 is False:
            blocked_wall.play()
        elif other_entity.tag == "blocked3" and self.touched_lever3 is False:
            blocked_wall.play()
        elif other_entity.tag == "blocked4" and self.touched_lever4 is False:
            blocked_wall.play()
        elif other_entity.tag == "blocked5" and self.touched_lever5 is False:
            blocked_wall.play()
        elif other_entity.tag == "blocked6" and self.touched_lever6 is False:
            blocked_wall.play()
        elif other_entity.tag == "blocked7" and self.touched_lever6 is False:
            blocked_wall.play()
#
# engine.set_world(Maze())
# engine.run()