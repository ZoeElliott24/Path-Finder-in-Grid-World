from grid import Point
class Problem():
    x = int
    y = int
    value = Point(x,y)
    def __init__(self,value):
        self.initial = value
    def is_Goal(self,destination_value):
        if self.x == destination_value.x and self.y == destination_value:
            return True