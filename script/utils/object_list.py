from typing import List

# Place Position
PLACE = [-0.01490956941713506, -2.0275737247862757, 2.4103458563434046, -1.953010698358053, -1.5686352888690394, -0.05916053453554326]

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
    Object('block1', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    Object('block2', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    Object('block3', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    Object('block4', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    Object('special', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
]

area_list = [
    Area('box',       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    Area('place',     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    Area('left_area', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
]
