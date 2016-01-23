import components
from util_math import Vector2
from components import Transform
from components import RigidBody
from components import Collider
from components import Renderer
from components import Animator


class Entity (object):

    # Unique id should be modified by the entity manager
    def __init__(self, uuid=0):
        self.uuid = uuid
        self.components = list()
        self.tag = ""
        self.name = ""
        self.scripts = list()
        self.world = None

        # quick access to important components
        self.transform = None
        self.rigid_body = None
        self.renderer = None
        self.collider = None
        self.animator = None

        self.disabled = False

    def add_component(self, component):

        # link the component
        component.entity = self

        # special components. Variable for quick access
        if isinstance(component, Transform):
            self.transform = component

        elif isinstance(component, RigidBody):
            self.rigid_body = component

        elif isinstance(component, Collider):
            self.collider = component

        elif isinstance(component, Renderer):
            self.renderer = component

        elif isinstance(component, Animator):
            self.animator = component

        self.components.append(component)

    def remove_component(self, component_tag):
        i = 0
        for c in self.components:
            if c.tag == component_tag:
                self.components.pop(i)

                if component_tag == Transform.tag:
                    self.transform = None

                elif component_tag == RigidBody.tag:
                    self.rigid_body = None

                elif component_tag == Collider.tag:
                    self.collider = None

                elif component_tag == Renderer.tag:
                    self.renderer = None

                elif component_tag == Animator.tag:
                    self.animator = None

                return
            i += 1

    def add_script(self, script):

        # link the script
        script.entity = self
        self.scripts.append(script)

    def remove_script(self, script_name):
        i = 0
        for s in self.scripts:

            # script found
            if s.script_name == script_name:
                self.scripts.pop(i)
                return
            i += 1

    def get_script(self, script_name):
        i = 0
        for s in self.scripts:
            # script found
            if s.script_name == script_name:
                return s
            i += 1
        return None

    def get_component(self, component_tag):
        for c in self.components:
            if c.tag == component_tag:
                return c
        return None

    def __eq__(self, other):
        return self.uuid == other.uuid

    def __str__(self):
        return self.tag + ", " + self.name + ", " + str(self.uuid)

    def __repr__(self):
        return self.__str__()


# A basic game object with a transform, render, box collision, and rigid body components.
# The image is centered for the render component.
# A box collider is automatically bounded to the image dimensions.
# In order to create a game object, the initial sprite image must be specified.
class GameObject (Entity):
    def __init__(self, image_surface, uuid=0):
        super(GameObject, self).__init__(uuid)

        img_width = image_surface.get_width()
        img_height = image_surface.get_height()

        # Set up pivot for the image
        pivot = Vector2(img_width/2, img_height/2)

        self.transform = Transform(Vector2(0, 0))
        self.renderer = Renderer(image_surface, pivot)
        self.collider = components.BoxCollider(img_width, img_height)

        self.add_component(self.transform)
        self.add_component(self.renderer)
        self.add_component(self.collider)


# create a game object with only a transform and renderer
class RenderableObject (Entity):
    def __init__(self, image_surface, pivot=None, uuid=0):
        super(RenderableObject, self).__init__(uuid)

        img_width = image_surface.get_width()
        img_height = image_surface.get_height()

        # Set up pivot for the image
        pivot = Vector2(img_width/2, img_height/2) if pivot is None else Vector2(0, 0)

        self.transform = Transform(Vector2(0, 0))
        self.renderer = Renderer(image_surface, pivot)

        self.add_component(self.transform)
        self.add_component(self.renderer)


# A game object with only a transform and collision box components.
# Could be used to create invisible game barriers
class BoxColliderObject (Entity):
    def __init__(self, width, height, uuid=0):
        super(BoxColliderObject, self).__init__(uuid)

        self.transform = Transform(Vector2(0, 0))
        self.collider = components.BoxCollider(width, height)

        self.add_component(self.transform)
        self.add_component(self.collider)


class CircleColliderObject(Entity):

    def __init__(self, radius, uuid=0):
        super(CircleColliderObject, self).__init__(uuid)

        self.transform = Transform(Vector2(0, 0))
        self.collider = components.CircleCollider(radius)

        self.add_component(self.transform)
        self.add_component(self.collider)