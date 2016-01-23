
from world import *
from engine import *
from components import BehaviorScript
from components import WorldScript

engine = Engine(1200, 700)

# sound effect to play once the puzzle is completed
puzzle_finished_sfx = mixer.Sound("assets/sound/piano_low_key.wav")


class CheckBoxes(WorldScript):

        def __init__(self):
            super(CheckBoxes, self).__init__("check boxes")
                
            # list of coordinates of where boxes are supposed to be
            self.boxCoordinates = dict()

            # 450, 275 500, 275 475, 200 350, 225
            # the correct position of boxes with a boolean condition
            # to check if a box has been snapped into place
            self.boxCoordinates["pbox1"] = [Vector2(526, 294), False]
            self.boxCoordinates["pbox2"] = [Vector2(489, 294), False]
            self.boxCoordinates["pbox3"] = [Vector2(508, 239), False]
            self.boxCoordinates["pbox4"] = [Vector2(415, 257), False]
            self.boxCoordinates["pbox5"] = [Vector2(452, 405), False]
            self.boxCoordinates["pbox6"] = [Vector2(692, 350), False]

        def update(self):
            # if the boxes are not in their final form
            if not self.all_in_place():

                # loop to iterate through all the boxes as well as the list of boxes
                for box in self.world.boxes:
                    if not self.boxCoordinates[box.tag][1]:
                        tgt_position = self.boxCoordinates[box.tag][0]

                        tgt_distance = box.transform.position - tgt_position

                        #  if the player lets go close to the target position
                        if tgt_distance.sq_magnitude() < 400 and (self.world.player.get_script("player move").selected_crate is not box):

                            # snap the box
                            box.transform.position.x = tgt_position.x
                            box.transform.position.y = tgt_position.y
                            self.boxCoordinates[box.tag][1] = True

                        # position of where boxes are supposed to go
                        # used for testing purposes
                        # pygame.draw.circle(self.world.engine.display, (255, 255, 255), (int(tgt_position.x), int(tgt_position.y)), 2)

        def all_in_place(self):

            # already completed no need to iterate through the boxes to check
            if self.world.puzzle_finished:
                return True

            for key in self.boxCoordinates:

                # if one is not in place then the test fails
                if not self.boxCoordinates[key][1]:
                    return False

            # puzzle has been finished - play a sound to notify the player
            self.world.puzzle_finished = True
            puzzle_finished_sfx.play()

            return True


class PlayerFibMovement(BehaviorScript):

    def __init__(self):
        super(PlayerFibMovement, self).__init__("player move")
        self.h_speed = 200
        self.v_speed = 300
        self.right = pygame.image.load("assets/images/character/character_east.png").convert_alpha()
        self.left = pygame.image.load("assets/images/character/character_west.png").convert_alpha()
        self.up = pygame.image.load("assets/images/character/character_north.png").convert_alpha()
        self.down = pygame.image.load("assets/images/character/character_south.png").convert_alpha()

        self.up_right = pygame.image.load("assets/images/character/character_northeast.png").convert_alpha()
        self.up_left = pygame.image.load("assets/images/character/character_northwest.png").convert_alpha()
        self.down_right = pygame.image.load("assets/images/character/character_southeast.png").convert_alpha()
        self.down_left = pygame.image.load("assets/images/character/character_southwest.png").convert_alpha()

        self.selected_crate = None

    def update(self):

        keys = pygame.key.get_pressed()

        velocity = self.entity.rigid_body.velocity

        if keys[pygame.K_a]:
            velocity.x = -self.h_speed
            self.entity.renderer.sprite = self.left
        elif keys[pygame.K_d]:
            velocity.x = self.h_speed
            self.entity.renderer.sprite = self.right

        else:
            velocity.x = 0

        if keys[pygame.K_w]:
            velocity.y = -self.h_speed
            self.entity.renderer.sprite = self.up

        elif keys[pygame.K_s]:
            velocity.y = self.h_speed
            self.entity.renderer.sprite = self.down

        else:
            velocity.y = 0

        if keys[pygame.K_w] and keys[pygame.K_d]:
            self.entity.renderer.sprite = self.up_right
        elif keys[pygame.K_w] and keys[pygame.K_a]:
            self.entity.renderer.sprite = self.up_left
        elif keys[pygame.K_s] and keys[pygame.K_d]:
            self.entity.renderer.sprite = self.down_right
        elif keys[pygame.K_s] and keys[pygame.K_a]:
            self.entity.renderer.sprite = self.down_left

        # move the crate with the player
        if self.selected_crate is not None:
            self.move_crate()

        # player left crate bounds
        if not self.check_if_near_crate()[0] or not self.colliding_with_selected_crate():
            self.selected_crate = None

    def take_input(self, event):

        # if the boxes are not in the final formation then be able to move them around
        if not self.entity.world.puzzle_finished:

            # player decides to move crate. He must not be holding anything
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LCTRL and self.selected_crate is None:

                    result = self.check_if_near_crate()
                    # if the player is near a crate
                    if result[0]:
                        crate = result[1]
                        self.selected_crate = crate

                        # the crate becomes un-snapped
                        self.entity.world.get_script("check boxes").boxCoordinates[crate.tag][1] = False

                # player decides to stop moving crate
                elif event.key == pygame.K_LCTRL and self.selected_crate is not None:
                    self.selected_crate = None

    def move_crate(self):
        v = self.entity.rigid_body.velocity
        dt = self.entity.world.engine.delta_time

        crate = self.selected_crate

        # stop movement
        # colliders of the world
        entity_colliders = self.entity.world.boxes + self.entity.world.walls

        move_x = 1
        move_y = 1

        for other in entity_colliders:

            if crate is not other:

                # if the selected crate collided with anything else
                if PhysicsSystem.box2box_collision(crate.collider, other.collider):

                    # figure out from what direction it hit the other object
                    side = PhysicsSystem.calc_box_hit_orientation(crate.collider, other.collider)

                    # get crate outside the collider
                    PhysicsSystem._resolve_box2box_with_collider(side, crate.transform, crate.collider, other.collider)

                    # stop the crate in place
                    # hit from top or bottom
                    if side == PhysicsSystem.bottom or side == PhysicsSystem.top:
                        move_y = 0

                    # hit from left or right
                    elif side == PhysicsSystem.right or side == PhysicsSystem.left:
                        move_x = 0

        # move crate if possible
        self.selected_crate.transform.position.x += v.x * dt * move_x
        self.selected_crate.transform.position.y += v.y * dt * move_y

    def colliding_with_selected_crate(self):

        if self.selected_crate is None:
            return False

        result = False

        player = self.entity
        crate = self.selected_crate

        temp_player_box = player.collider.box
        temp_crate_box = crate.collider.box

        # use the tolerance hit boxes to detect collision
        player.collider.box = player.collider.tolerance_hitbox
        crate.collider.box = crate.collider.tolerance_hitbox

        # use the tolerance hit boxes to detect collision
        if PhysicsSystem.box2box_collision(player.collider, crate.collider):
            result = True

        # # reset the collider boxes to the original ones
        player.collider.box = temp_player_box
        crate.collider.box = temp_crate_box

        return result

    def check_if_near_crate(self):
        result = (False, None)

        # check if the player is near a box
        for crate in self.entity.world.boxes:
            if PhysicsSystem.tolerance_collision(self.entity.collider, crate.collider):
                result = (True, crate)
        return result

    def collision_event(self, other_collider):
        if other_collider.entity.name == "trigger object exit":

            # save the original spawn point to this world
            self.entity.transform.position = Vector2(100, 100)
            self.entity.rigid_body.velocity.zero()

            self.entity.world.engine.game.go_to_main()


class FibWorld(World):

    def __init__(self):
        super(FibWorld, self).__init__()

        self.player = None
        self.box = None
        self.boxes = list()

        # walls
        self.topWall = None
        self.leftWall = None
        self.rightWall = None
        self.bottomWall = None

        self.walls = None

        # used to signal that the puzzle has been already done
        self.puzzle_finished = False

        self.trigger_object_exit = None

    def resume(self):
        mixer.music.load("assets/music/game_select_bak.ogg")
        mixer.music.play(-1)
        mixer.music.set_volume(0.3)

    def load_scene(self):

        self.trigger_object_exit = self.create_box_collider_object(200, 60)
        self.trigger_object_exit.collider.is_trigger = True
        self.trigger_object_exit.transform.position = Vector2(-90, 100)
        self.trigger_object_exit.name = "trigger object exit"

        background_image = pygame.image.load("assets/images/floors/Floor.png").convert()
        lamps_image  = pygame.image.load("assets/images/floors/Lamps.png").convert_alpha()

        # add necessary components to be able to position and render 
        # the background
        background = self.create_entity()
        background.add_component(Transform(Vector2(0, 0)))
        background.add_component(Renderer(background_image))
        background.renderer.depth = 100

        background_lamps = self.create_entity()
        background_lamps.add_component(Transform(Vector2(0, 0)))
        background_lamps.add_component(Renderer(lamps_image))
        background_lamps.renderer.depth = -100

        # frames to demonstrate player animation
        frame1 = pygame.image.load("assets/images/character/character_west.png").convert_alpha()

        # setupt the player
        self.player = self.create_game_object(frame1)
        self.player.add_component(RigidBody(Vector2(0, 0)))
        self.player.transform.position = Vector2(100, 100)
        self.player.renderer.depth = -10
        self.player.rigid_body.gravity_scale = 1
        self.player.add_script(PlayerFibMovement())
        self.player.collider.restitution = 1
        self.player.collider.box.w -= 60
        self.player.collider.box.h -= 60
        self.player.collider.tolerance_hitbox.w -= 60
        self.player.collider.tolerance_hitbox.h -= 60

        # boxes to be moved around
        # 450, 275 # 500, 275 # 475, 200 # 350, 225 # 400, 425 # 725, 350
        box_image = pygame.image.load("assets/images/crates/FibonacciBox_37a.png").convert_alpha()
        pbox = self.create_game_object(box_image)
        pbox.transform.position = Vector2(526+600, 294-50)
        pbox.tag = "pbox1"
        self.boxes.append(pbox)

        box_image = pygame.image.load("assets/images/crates/FibonacciBox_37b.png").convert_alpha()
        pbox = self.create_game_object(box_image)
        pbox.transform.position = Vector2(489-400, 294+50)
        pbox.tag = "pbox2"
        self.boxes.append(pbox)

        box_image = pygame.image.load("assets/images/crates/FibonacciBox_74.png").convert_alpha()
        pbox = self.create_game_object(box_image)
        pbox.transform.position = Vector2(508+450, 239-150)
        pbox.tag = "pbox3"
        self.boxes.append(pbox)

        box_image = pygame.image.load("assets/images/crates/FibonacciBox_111.png").convert_alpha()
        pbox = self.create_game_object(box_image)
        pbox.transform.position = Vector2(415+200, 257+200)
        pbox.tag = "pbox4"
        self.boxes.append(pbox)

        box_image = pygame.image.load("assets/images/crates/FibonacciBox_185.png").convert_alpha()
        pbox = self.create_game_object(box_image)
        pbox.transform.position = Vector2(452-100, 405+150)
        pbox.tag = "pbox5"
        self.boxes.append(pbox)

        box_image = pygame.image.load("assets/images/crates/FibonacciBox_296.png").convert_alpha()
        pbox = self.create_game_object(box_image)
        pbox.transform.position = Vector2(692+230, 350+150)
        pbox.tag = "pbox6"
        self.boxes.append(pbox)

        #screen dimensions halved 
        half_w = self.engine.display.get_width()/2
        half_h = self.engine.display.get_height()/2

        # Create the wall entities and set their collision boxes.
        # Make these wall thick, so the ball doesn't escape from the level.
        self.topWall = self.create_box_collider_object(half_w*4, 50)
        self.bottomWall = self.create_box_collider_object(half_w*4, 50)
        self.leftWall = self.create_box_collider_object(200, half_h*4)
        self.rightWall = self.create_box_collider_object(200, half_h*4)

        # set up wall positions
        self.topWall.transform.position = Vector2(half_w, 0-25)
        self.bottomWall.transform.position = Vector2(half_w, half_h*2+25)
        self.leftWall.transform.position = Vector2(0-50-50, half_h)
        self.rightWall.transform.position = Vector2(half_w*2+50+50, half_h)

        self.get_system(PhysicsSystem.tag).gravity.zero()
        self.add_script(CheckBoxes())

        self.walls = [self.topWall, self.leftWall, self.rightWall, self.bottomWall]

        # start the background music and set it to loop forever
        mixer.music.play(-1)
        mixer.music.set_volume(0.3)

# f = FibWorld()
# engine.set_world(f)
# engine.worlds.append(f)
# engine.run()
