
from entity import *


class EntityManager (object):

    def __init__(self):
        self.entities = list()
        self.id_manager = IdManager()

    # Adds entity to the entity manager and returns it so it can be modified
    def create_entity(self):
        id_value = self.id_manager.get_id()
        e = Entity(id_value)
        self.entities.append(e)
        return e

    def add(self, entity):
        id_value = self.id_manager.get_id()
        entity.uuid = id_value
        self.entities.append(entity)

    # Remove entity and recycle id
    def remove_entity(self, entity):
        self.entities.remove(entity)
        self.id_manager.recycle_id(entity.uuid)


class IdManager:

    def __init__(self):
        self.id_counter = 0
        self.ids = list()

    def get_id(self):
        # If there are no ids left to recycle then create a new one
        if len(self.ids) == 0:
            self.id_counter += 1
            return self.id_counter

        # reuse id
        return self.ids.pop()

    # put it back in the ids container
    def recycle_id(self, id_val):
        self.ids.append(id_val)