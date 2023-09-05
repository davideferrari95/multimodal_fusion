from typing import List

# Robot Positions
HOME  = [3.863, -1.765, 1.983, -1.760, -1.572, 0.720]
PLACE = [1.91663, -1.52108, 2.44315, -2.51999, -1.55590, 3.48031]

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
    Object('green_block',   [3.864, -1.401, 2.343, -2.483, -1.571, 0.720]),
    Object('red_block',     [4.184, -1.505, 2.471, -2.508, -1.561, 1.039]),
    Object('yellow_block',  [4.002, -1.251, 2.151, -2.472, -1.539, 2.396]),
    Object('special_block', [4.301, -1.418, 2.197, -2.341, -1.597, -0.443], [1.4134, -1.3824, 1.9419, -2.1471, -1.5453, -3.2848]),
]

area_list = [
    # Area('box',   [2.00, -1.76, 1.90, -1.59, -1.56, 0.0]),
    Area('place', PLACE),
    Area('left',  [0.018, -1.993, 2.789, -2.352, -1.546, -4.725]),
    Area('front', [1.761, -1.117, 1.911, -2.389, -1.552, -2.957]),
]
