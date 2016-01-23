from pygame import Rect
from abc import ABCMeta

from util_math import Vector2

from pygame import transform


# Base class for all components
class Component (object):

    __metaclass__ = ABCMeta

    def __init__(self):

        # The associated entity
        self.entity = None


# Basic transform that handles position , rotation and scale
class Transform (Component):

    tag = "transform"

    def __init__(self, position=Vector2(0.0, 0.0), degrees=0, x_scale=1, y_scale=1):
        super(Transform, self).__init__()
        self.position = position
        self.degrees = degrees
        self.scale = Vector2(x_scale, y_scale)

    # Even though the render system should handle this logic, it would do it at very
    # rendering update but these scaling operations on surface are expensive. To fix
    # this, the logic has been done here for efficiency purposes.
    def scale_by(self, x_scale, y_scale):

        # update the scale vector
        self.scale = Vector2(x_scale, y_scale)

        # transform the renderer's attributes
        renderer = self.entity.renderer

        # scale the image surface
        renderer.sprite = Renderer.scale_image(renderer.original_image, x_scale, y_scale)

        renderer.pivot.x *= abs(x_scale)
        renderer.pivot.y *= abs(y_scale)

        # scale every frame in the animator's current animation relative to the original frames
        if self.entity.animator is not None:
            anim = self.entity.animator.current_animation
            for i in range(0, len(anim.frames)):
                anim.frames[i] = Renderer.scale_image(anim.original_frames[i], x_scale, y_scale)

        collider = self.entity.collider

        #  scale the collider as well
        if collider is not None:

            # for box colliders
            if collider.tag == BoxCollider.tag:
                collider.scale_box_by(x_scale, y_scale)

            # circle colliders - use the x scale to scale the radius
            elif collider.tag == CircleCollider.tag:
                collider.radius *= abs(x_scale)

            # scale the offsets of the colliders
            collider.offset.x = x_scale * collider.original_offset.x
            collider.offset.y = y_scale * collider.original_offset.y


# Contains image to render
# pivot is of type Vector2 - it is the position relative to the image
# which specifies where to start drawing the image from.
# Rendering a image centered on a point would require that
# pivot = Vector2(image.width/2, image.height/2)
class Renderer (Component):
    tag = "render"

    def __init__(self, image, pivot=Vector2(0, 0)):
        super(Renderer, self).__init__()
        self.original_image = image
        self.sprite = image
        self.pivot = pivot

        # the rendering order. 0 is default
        # lower values render last (use for foreground)
        # larger values render first (use for background)
        # USE THE RENDER SYSTEM TO MODIFY THIS VALUE
        self.depth = 0

        # if the renderer is affected by the camera
        self.is_static = False

    # scale the destination image surface relative to the
    # source image.
    @staticmethod
    def scale_image(src_image, x_scale, y_scale):

        # flip the image if the scales are negative
        invert_x = x_scale < 0
        invert_y = y_scale < 0
        dst_image = transform.flip(src_image, invert_x, invert_y)

        # find the new dimensions of scaled image
        new_width = src_image.get_width() * abs(x_scale)
        new_height = src_image.get_height() * abs(y_scale)
        return transform.scale(dst_image, (int(new_width), int(new_height)))

    def set_image(self, image):
        self.original_image = image
        self.sprite = image

        xs = self.entity.transform.scale.x
        ys = self.entity.transform.scale.y

        # scale the the image by the transforms current scale
        self.entity.transform.scale_by(xs, ys)


# Only holds velocity vector and mass scalar, may be expanded in future development
# for a better physics simulations
class RigidBody (Component):
    tag = "rigid body"

    def __init__(self, velocity=Vector2(0.0, 0.0), m=1.0):
        super(RigidBody, self).__init__()
        self.velocity = velocity
        self.mass = m

        # gravity scale
        self.gravity_scale = 0

        self.gravity_enabled = False

        # self.fixed_angle = True
        # self.angular_velocity = Vector2(0, 0)
        # self.angular_drag = 0


class Collider(Component):
    tag = "collider"

    def __init__(self):
        super(Collider, self).__init__()

        # A value of 1 means no friction
        # A value of 0 means total friction, brings it to a complete halt.
        # Higher values add energy to the object it is colliding with
        self.surface_friction = 1

        # Bouncy-ness of a collider
        # A value of 0 means no bouncing effect
        # A value of 1 means completely elastic collision effect
        self.restitution = 0

        # It flags a static collider to be treated as one that can move but
        # does not have a rigid body. This is used to apply collision resolution
        # on the static collider.
        self.treat_as_dynamic = False

        # detects a collision against another collider only
        self.is_trigger = False

        # An offset relative to the transform position associated to the entity
        # that this collider belongs to
        self.offset = Vector2(0.0, 0.0)
        self.original_offset = Vector2(0, 0)

    def set_offset(self, x, y):
        self.original_offset = Vector2(x, y)
        self.offset = Vector2(x, y)


class BoxCollider (Collider):
    tag = "box collider"

    def __init__(self, width=0.0, height=0.0):
        super(BoxCollider, self).__init__()
        self.box = Rect(0, 0, width, height)

        # The tolerance hit box is used because when a collider is resolved, the original hit box , for a short time,
        # is not colliding with anything. The tolerance hit box takes care of that situation. (objects resting on floor).

        ################################################################################
        # The physics system does not use this for its calculations yet. When it is later implemented, the tolerance
        # hit box should only register that it is overlapping.
        ###############################################################################

        self.tolerance = 10
        self.tolerance_hitbox = Rect(0, 0, width+self.tolerance, height+self.tolerance)

        self.tolerance_hitbox.center = (self.offset.to_tuple())

    def set_box(self, width, height):
        self.box = Rect(0, 0, width, height)
        self.tolerance_hitbox = Rect(0, 0, width+self.tolerance, height+self.tolerance)
        self.tolerance_hitbox.center = (self.offset.to_tuple())

    def scale_box_by(self, x_scale, y_scale):

        self.box.w *= abs(x_scale)
        self.box.h *= abs(y_scale)

        self.tolerance_hitbox.w *= abs(x_scale)
        self.tolerance_hitbox.h *= abs(y_scale)


class CircleCollider(Collider):
    tag = "circle collider"

    def __init__(self, radius=1.0):
        super(CircleCollider, self).__init__()
        self.radius = radius


class Animator(Component):

    tag = "animator"

    class Animation:

        def __init__(self):

            # name to identity the animation
            self.name = "base animation"

            # list of the original frames
            self.original_frames = list()

            # a list of images that will be displayed
            self.frames = list()

            # time between frames in seconds
            self.frame_latency = 1.0

            # Cycle == True means to restart back to the first frame
            # and cycle == False means to stop at the last frame.
            self.cycle = True

        def add_frame(self, frame):
            self.frames.append(frame)
            self.original_frames.append(frame)

    def __init__(self):
        super(Animator, self).__init__()
        self.current_animation = None

        # track how much time has passed
        self.latency_accumulator = 0.0

        # the current frame from the animation
        self.current_frame_index = 0

        # pause an animation at the current frame
        self.pause = False

    def set_animation(self, new_animation):

        self.current_animation = new_animation
        self.current_frame_index = 0

        # apply transform scaling
        x_scale = self.entity.transform.scale.x
        y_scale = self.entity.transform.scale.y
        self.entity.transform.scale_by(x_scale, y_scale)

        # change the original image of the renderer to some frame of
        # the current animation, and also scale it to the current transform's scale
        self.entity.renderer.original_image = Renderer.scale_image(self.current_animation.frames[0], x_scale, y_scale)

    def _update_animation(self):

        anim = self.current_animation

        if anim is not None and not self.pause:

            num_of_frames = len(anim.frames)

            if num_of_frames > 0:

                # time to go to the next frame
                if self.latency_accumulator > anim.frame_latency:

                    self.current_frame_index += 1

                    # cycle through frames
                    if anim.cycle:
                        self.current_frame_index %= num_of_frames

                    # stop at the last frame
                    elif self.current_frame_index >= len(anim.frames):
                        return

                    # Update the renderer's image to display
                    index = self.current_frame_index
                    self.entity.renderer.sprite = anim.frames[index]

                    # reset accumulator
                    self.latency_accumulator = 0.0

                # increment accumulator
                dt = self.entity.world.engine.delta_time
                self.latency_accumulator += dt


# This component simply flags which entity can receive input.
class InputComponent (Component):
    tag = "input"

    def __init__(self):
        super(InputComponent, self).__init__()


class Script (object):
    tag = "script"

    def __init__(self, script_name):
        self.script_name = script_name

    def take_input(self, event):
        pass

    # Called at every game iteration. Used for logic.
    def update(self):
        pass

    # compare equality by script name
    def __eq__(self, other):
        return self.script_name == other.script_name


class WorldScript(Script):

    tag = "world script"

    def __init__(self, script_name):
        super(WorldScript, self).__init__(script_name)
        self.script_name = script_name
        self.world = None


class BehaviorScript(Script):

    tag = "behavior script"

    def __init__(self, script_name):
        super(Script, self).__init__()
        self.script_name = script_name
        self.entity = None

    # The physics system calls this function when the belonging
    # entity of this script collides with another entity's collider
    def collision_event(self, other_collider):
        pass

    # This is called by the Physics system when the entity stops colliding with the other
    # collider
    def collision_exit_event(self, other_collider):
        pass