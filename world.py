
from managers import EntityManager
from entity import *
from systems import *


# A world is like a game level. It holds the necessary game objects
# and assets to be used for a level.
# Inherit from this class and implement the abstract methods in order
# to create a custom world.

class World (object):
    __metaclass__ = ABCMeta

    def __init__(self):
        # Reference to the engine that it exists in
        self.engine = None
        self.systems = list()
        self.entity_manager = EntityManager()

        # for world behavior
        self.scripts = list()

        # add the fundamental systems
        self.add_system(PhysicsSystem())
        self.add_system(RenderSystem())

        # bounds - negative values means no bounds
        self.width = -1
        self.height = -1
        self.origin = Vector2(0, 0)

        self.loading_scene = False

    # this function is a wrapper that is used to detect if we are loading the scene of the world
    def start_scene_loading(self):
        self.loading_scene = True

        # call the overrided scene loading function
        self.load_scene()

        self.loading_scene = False

    @abstractmethod
    def load_scene(self):
        """
        Setup the world scene by specifying how/where to create
        game objects.
        """

    # initiate things when the game goes back to this level
    def resume(self):
        pass

    # Do not override
    def _take_input(self, event):

        for s in self.scripts:
            s.take_input(event)

        # run script input for entities
        for e in self.entity_manager.entities:
            for s in e.scripts:
                s.take_input(event)

    def get_entity_by_tag(self, tag):
        for e in self.entity_manager.entities:
            if e.tag == tag:
                return e
        return None

    # create an empty entity (no components)
    def create_entity(self):
        e = self.entity_manager.create_entity()
        e.world = self
        return e

    def create_game_object(self, image_surface):
        entity = GameObject(image_surface)
        entity.world = self
        self.entity_manager.add(entity)

        # if the entity was created outside the load_scene then do
        # a dynamic insertion to the RenderSystem's scene
        if not self.loading_scene:
            self.get_system(RenderSystem.tag).dynamic_insertion_to_scene(entity)

        return entity

    def create_renderable_object(self, image_surface, pivot=None):
        entity = RenderableObject(image_surface, pivot)
        entity.world = self
        self.entity_manager.add(entity)

        if not self.loading_scene:
            self.get_system(RenderSystem.tag).dynamic_insertion_to_scene(entity)

        return entity

    def create_box_collider_object(self, width, height):
        entity = BoxColliderObject(width, height)
        entity.world = self
        self.entity_manager.add(entity)
        return entity

    def create_circle_collider_object(self, radius):
        entity = CircleColliderObject(radius)
        entity.world = self
        self.entity_manager.add(entity)
        return entity

    def destroy_entity(self, entity):

        # remove the entity from the scene
        render_system = self.get_system(RenderSystem.tag)
        render_system.remove_from_scene(entity)

        self.entity_manager.remove_entity(entity)

    def add_system(self, system):
        system.world = self

        # add to the front - so the physics and render systems are
        # the last systems to do their logic.
        self.systems.insert(0, system)

    def remove_system(self, tag):
        for system in self.systems:
            if system.tag == tag:
                self.systems.remove(system)
                return

    def get_system(self, tag):
        for system in self.systems:
            if system.tag == tag:
                return system
        return None

    def add_script(self, script):
        script.world = self
        self.scripts.append(script)

    def remove_script(self, script):
        self.scripts.remove(script)

    def get_script(self, script_name):
        i = 0
        for s in self.scripts:
            # script found
            if s.script_name == script_name:
                return s
            i += 1
        return None

    # Have each system process the entities
    def run(self):
        for s in self.systems:
            s.process(self.entity_manager.entities)

        # Run script updates - Reverse iteration to handle removals of entities.
        for i in xrange(len(self.entity_manager.entities) - 1, -1, -1):
            e = self.entity_manager.entities[i]
            for s in e.scripts:
                s.update()

        # World scripts - Reverse iteration to handle removals of entities.
        for i in xrange(len(self.scripts)-1, -1, -1):
            s = self.scripts[i]
            s.update()

    # determine if the world has bounds
    def is_bounded(self):
        return self.width > 0 and self.height > 0