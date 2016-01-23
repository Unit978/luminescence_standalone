__author__ = 'luisl_000'

# This class contains helper functions to create objects or animations

from components import Animator
from util_math import Vector2
from os import listdir
from re import split
from pygame import Surface
from pygame import image
from components import RigidBody
from components import BoxCollider


def set_lamp_light_attributes(lamp_light, rs):
    lamp_light.renderer.depth = 10000
    lamp_light.add_component(BoxCollider(50, 50))
    lamp_light.collider.is_trigger = True
    lamp_light.tag = "lamp light"
    rs.light_sources.append(lamp_light)


def set_floor_attributes(floor):
    floor.collider.restitution = 0
    floor.collider.surface_friction = 0.75
    floor.renderer.depth = -10
    floor.tag = "floor"


def set_wall_attributes(wall):
    wall.collider.restitution = 0
    wall.collider.surface_friction = 0.8
    wall.tag = "wall"


def set_ceiling_attributes(ceiling):
    ceiling.collider.restitution = 0
    ceiling.collider.surface_friction = 0.8
    ceiling.tag = "ceiling"


def set_platform_attributes(platform):
    platform.renderer.depth = 1
    platform.collider.restitution = 0
    platform.collider.surface_friction = 0.75
    platform.tag = "platform"


def set_box_attributes(box):
    box.renderer.depth = 2
    box.collider.restitution = 0
    box.collider.surface_friction = 0.8
    box.collider.box.w -= 10
    box.collider.box.h -= 10

    box.add_component(RigidBody())
    box.rigid_body.velocity = Vector2(0.0, 0.0)
    box.rigid_body.gravity_scale = 2.0
    box.tag = "box"


def get_files_in_dir(dir_path):

    directory = listdir(dir_path)

    # a list to store the paths to the individual files
    file_paths = list()
    for file_ in directory:

        # add the whole relative path
        file_paths.append(dir_path + file_)

    # do a natural sort on the file names, meaning that string numbers are sorted
    # by numeric values.
    file_paths = natural_sort(file_paths)
    return file_paths


# This goes to a directory where each file represents an individual
# animation frame. This returns an animation object with the loaded frames.
def load_anim_from_directory(dir_path):

    file_list = get_files_in_dir(dir_path)

    # set up animation
    animation = Animator.Animation()
    for file_ in file_list:
        frame = image.load(file_).convert_alpha()
        animation.add_frame(frame)

    return animation


# Excerpt from stack overflow, Mark Byers
def natural_sort(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in split('([0-9]+)', key)]
    return sorted(l, key=alphanum_key)


# given a tile image, this function will create a quilt of tiles onto a another surface
# given the width and height.
def create_img_from_tile(tile_surface, width, height):

    tile_w = tile_surface.get_width()
    tile_h = tile_surface.get_height()

    size = (width, height)
    dst_surface = Surface(size).convert()

    # make the dst_surface transparent
    color_mask = (7, 13, 17)
    dst_surface.fill(color_mask)
    dst_surface.set_colorkey(color_mask)

    # fill the dst_surface horizontally first with the tile, once we get to the edge
    # go down a column and repeat.
    for y in range(0, height, tile_h):
        for x in range(0, width, tile_w):
            dst_surface.blit(tile_surface, (x, y))

    return dst_surface


# conjoins surface b on the bottom of surface a
def conjoin_surfaces_vertically(surface_a, surface_b):
    width = max(surface_a.get_width(), surface_b.get_width())
    height = surface_a.get_height() + surface_b.get_height()

    size = (width, height)
    dst_surface = Surface(size).convert()

    # make the dst_surface transparent
    color_mask = (123, 54, 33)
    dst_surface.fill(color_mask)
    dst_surface.set_colorkey(color_mask)

    # center each surface relative to the destination surface
    center_a = width/2 - surface_a.get_width()/2
    center_b = width/2 - surface_b.get_width()/2

    dst_surface.blit(surface_a, (center_a, 0))
    dst_surface.blit(surface_b, (center_b, surface_a.get_height()))

    return dst_surface
