from grid import Point
class Node:
    set_node_x = int
    set_node_y = int
    def __init__(self,set_node_x,set_node_y):
        self.child = []
        self.parent = None
        self.x = set_node_x
        self.y = set_node_y
        self.set_node = Point(self.x,self.y)
        self.path_cost = 0
    def __str__(self):
        return str(self.set_node)

    def add_child(self,child_node):
        self.child.append(child_node)
        return child_node
    def get_child(self):
        return self.child
    def get_parent(self):
        return self.parent
    def left(self):
       left_node = self.set_node.x - 1
       return Point(left_node,self.set_node.y)

    def right(self):
        right_node = self.set_node.x + 1
        return Point(right_node,self.set_node.y)

    def up(self):
        #print(self.x)
        up_node = self.set_node.y+1
        return Point(self.set_node.x,up_node)

    def down(self):
        down_node = self.set_node.y-1
        return Point(self.set_node.x,down_node)