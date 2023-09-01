from typing import List

# Place Position
PLACE = [1.50, -1.76, 1.90, -1.59, -1.56, 0.0]

class Object:

    def __init__(self, name: str, pick_position: List[float], place_position: List[float] = PLACE):

        # Object Name, Pick Position, Place Position
        self.name: str = name
        self.pick_position: List[float] = pick_position
        self.place_position: List[float] = place_position

    def getName(self):
        return self.name

    def getPickPosition(self):
        return self.pick_position

    def getPlacePosition(self):
        return self.place_position

class Area:

    def __init__(self, name: str, position: List[float]):

        # Area Name, Position
        self.name: str = name
        self.position: List[float] = position

    def getName(self):
        return self.name

    def getPosition(self):
        return self.position

object_list = [
    Object('block1', [1.00, -1.76, 1.90, -1.59, -1.56, 0.0]),
    Object('block2', [1.00, -1.76, 1.90, -1.59, -1.56, 0.0]),
    Object('block3', [1.00, -1.76, 1.90, -1.59, -1.56, 0.0]),
    Object('block4', [1.00, -1.76, 1.90, -1.59, -1.56, 0.0]),
    Object('special', [1.00, -1.76, 1.90, -1.59, -1.56, 0.0], [0.50, -1.76, 1.90, -1.59, -1.56, 0.0]),
]

area_list = [
    Area('box',       [0.00, -1.76, 1.90, -1.59, -1.56, 0.0]),
    Area('place',     [0.00, -1.76, 1.90, -1.59, -1.56, 0.0]),
    Area('left_area', [0.00, -1.76, 1.90, -1.59, -1.56, 0.0]),
]
