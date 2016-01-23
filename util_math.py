from math import sqrt
from math import acos
from math import sin
from math import cos

# Two dimensional vector that supports the basic operations
# such addition of vectors, scalar multiplication, dot product,
# and normalization


class Vector2:

    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vector2(self.x + other.x, self.y + other.y)

    def __iadd__(self, other):
        self.x += other.x
        self.y += other.y
        return self

    def __sub__(self, other):
        return Vector2(self.x - other.x, self.y - other.y)

    def __isub__(self, other):
        self.x -= other.x
        self.y -= other.y
        return self

    def __mul__(self, scale):
        return Vector2(self.x * scale, self.y * scale)

    def __rmul__(self, scale):
        return self.__mul__(scale)

    def __div__(self, scale):
        return self.__mul__(1/scale)

    def scale_by(self, s):
        self.x *= s
        self.y *= s

    # Return copy of a scaled version of the vector
    @staticmethod
    def get_scaled_by(vector2, s):
        return Vector2(vector2.x*s, vector2.y*s)

    def normalize(self):
        m = self.magnitude()
        self.x /= m
        self.y /= m

    # Return a copy of the normalized vector
    @staticmethod
    def get_normal(vector2):
        m = vector2.magnitude()
        return Vector2(vector2.x/m, vector2.y/m)

    def magnitude(self):
        return sqrt(self.x*self.x + self.y*self.y)

    # Squared magnitude
    def sq_magnitude(self):
        return self.x*self.x + self.y*self.y

    def set_magnitude(self, mag):
        # Use properties of similar triangles
        m = self.magnitude()
        if m != 0:
            ratio = mag / m
            self.x *= ratio
            self.y *= ratio
        else:
            self.x = 0.0
            self.y = 0.0

    # Uses the angle between i-unit vector and x-y vector component
    def direction(self):
        r = acos(self.x / self.magnitude())
        if self.y < 0:
            r *= -1
        return r

    # direction must be in radians
    def set_direction(self, direction):
        m = self.magnitude()
        self.x = m * cos(direction)
        self.y = m * sin(direction)

        # y_temp = m * sin(direction)
        # self.y = -y_temp if self.y > 0 else y_temp

    # Dot product of two vectors
    def dot(self, other):
        return self.x*other.x + self.y*other.y

    # Return angle between vectors a and b
    @staticmethod
    def angle(vector_a, vector_b):
        n = vector_a.dot(vector_b)
        d = vector_a.magnitude() * vector_b.magnitude()
        return acos(n / d)

    def zero(self):
        self.x = 0
        self.y = 0

    def is_zero(self):
        return self.x == 0 and self.y == 0

    def __str__(self):
        return "<" + str(self.x) + ", " + str(self.y) + ">"

    def to_tuple(self):
        return self.x, self.y


# centers the rect around position coordinate
def get_relative_rect_pos(position, collider):
    collider.box.x = position.x - collider.box.width/2 + collider.offset.x
    collider.box.y = position.y - collider.box.height/2 + collider.offset.y