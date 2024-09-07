import matplotlib.pyplot as plt
import numpy as np
import time
import matplotlib.animation as animation
import math
from utils import *
from grid import *
import Problem
from Problem import Problem
from Node import Node
options = []
polygon_list = []
res_path = []
Shape_list = []
turfs = []
VALUE =False
EXPANDED_NODES = 0
PATH_NODES = 0
f_value = 0

def gen_polygons(worldfilepath):
    temp = 0
    polygons = []
    with open(worldfilepath, "r") as f:
        lines = f.readlines()
        lines = [line[:-1] for line in lines]
        for line in lines:
            polygon = []
            pts = line.split(';')
            for pt in pts:
                xy = pt.split(',')
                polygon.append(Point(int(xy[0]), int(xy[1])))
                polygon_list.append((Point(int(xy[0]), int(xy[1]))))
            temp = temp + 1
            polygons.append(polygon)
    return polygons

#Breadth-First Search algorith
def BFS(problem):
    global EXPANDED_NODES
    global PATH_NODES
    find_shapes()
    node = Node(problem.initial.x,problem.initial.y)
    if problem.is_Goal(dest):
        return res_path
    reached = set()
    frontier = Queue()
    frontier.push(node)
    frontier_counter = set()
    frontier_counter.add((node.x, node.y))
    while not frontier.isEmpty():
        options.clear() #will clear the options
        node = frontier.pop()
        if (node.x, node.y) not in reached: #this adds the state to explored
            EXPANDED_NODES = EXPANDED_NODES + 1
            reached.add((node.x, node.y)) #adds the new node to explored
        options_append(Point(node.x, node.y))
        for child in options: #options is EXPAND
            s = child
            check_s = checker(s)  #checks for collision
            if check_s == False:
                if(0<=s.x<50) and (0<=s.y<50):
                    if (s.x, s.y) not in reached and (s.x, s.y) not in frontier_counter:
                        s.parent = node
                        frontier.push(s)
                        frontier_counter.add((s.x, s.y))
                        if s.x == dest.x and s.y == dest.y:
                            res_path.append(Point(s.x, s.y))
                            s = s.parent
                            while s:
                                PATH_NODES = PATH_NODES + 1
                                res_path.append(Point(s.x, s.y))
                                s = s.parent
                            return res_path

#Depth-First Search algorithm
def DFS(problem):
   global EXPANDED_NODES
   global PATH_NODES
   find_shapes()
   node = Node(problem.initial.x,problem.initial.y)
   explored = set()
   frontier = Stack()
   frontier.push(node)
   frontier_counter = set()
   frontier_counter.add((node.x, node.y))
   while not frontier.isEmpty():
       options.clear()
       node = frontier.pop()
       if node.x == dest.x and node.y == dest.y: #goal test
           res_path.append(Point(node.x, node.y))
           node = node.parent
           while node:
               PATH_NODES = PATH_NODES + 1
               res_path.append(Point(node.x, node.y))
               node = node.parent
           return res_path
       if (node.x, node.y) not in explored: #this adds the state to explored
           EXPANDED_NODES = EXPANDED_NODES + 1
           explored.add((node.x, node.y))
       options_append(Point(node.x, node.y))
       for action in options: #options is EXPAND()
           child = action
           check_child = checker(child)
           if check_child == False:
               if(0<=child.x<50) and (0<=child.y<50):
                   if (child.x, child.y) not in explored and (child.x, child.y) not in frontier_counter:
                       child.parent = node
                       frontier.push(child)
                       frontier_counter.add((child.x, child.y))

#A* algorithm
def Astar(problem):
    node = Node(problem.initial.x, problem.initial.y)
    BestFS(problem, f1)
#f1(node): calculates the f(n) value for A*
def f1(node):
    h= hn_function(node)
    path = node.path_cost
    f_value=path +h
    return f_value

#Greedy Best First Search Algorithm
def Greedy(problem):
    node = Node(problem.initial.x, problem.initial.y)
    BestFS(problem, f2)

#f2(node): calculates the f(n) value for Greedy Best First Search
def f2(node):
    return hn_function(node)

#Best-First Search algorithm
def BestFS(problem,fn):
    global EXPANDED_NODES
    global PATH_NODES
    find_shapes()
    find_turfs()
    node = Node(problem.initial.x, problem.initial.y)
    frontier = PriorityQueue()
    f = fn(node)
    frontier.push(node, f)
    frontier_tracker = set()
    frontier_tracker.add((node.x, node.y))
    reached = {}
    reached[node] = node.path_cost
    while not frontier.isEmpty():
        options.clear() #will clear the options so it will start over a new set and not repeat through the same
        node = frontier.pop()
        EXPANDED_NODES += 1
        options_append(Point(node.x, node.y))
        if node.x == dest.x and node.y == dest.y:
            res_path.append(Point(node.x, node.y))
            node = node.parent
            while node:
                res_path.append(Point(node.x, node.y))
                node = node.parent
            return res_path
        for child in options:   #options is EXPAND
            check_child = checker(child)
            if check_child == False:
                if(0<=child.x<50) and (0<=child.y<50):
                    s = Node(child.x,child.y)
                    if(s.x, s.y) not in reached or s.path_cost<reached[node]:
                        if(s.x, s.y) not in frontier_tracker:
                            child.parent = node
                            turf_value = turf_checker(s)
                            child.path_cost = node.path_cost + turf_value #this updates the path_cost
                            f = fn(child) #calculates the f value
                            reached[s] = child
                            frontier_tracker.add((s.x, s.y))
                            frontier.update(child,f) #add child to frontier
                            PATH_NODES = child.path_cost

#options_append(node): converts the options to the node point
def options_append(node):
    node = Node(node.x,node.y)
    options.append(Node(node.x,node.y+1)) #up
    options.append(Node(node.x+1,node.y))  #right
    options.append(Node(node.x, node.y -1))    #down
    options.append(Node(node.x-1,node.y))     #left

#Checker(node): checks to see if the point is inside or outside of the shape.
#False means that the option will not go into the shape.
# True means it is/ will go inside of the shape
def checker(node):
    node = Node(node.x, node.y)
    number = collision(node,Shape_list)
    if number == None:
        VALUE = False
    else:
        if number %2 !=0:
            VALUE = True  #means collision
        else:
            VALUE = False #means no collision
    return VALUE

#hn_function(node): calculates the distance from the current node to the goal
def hn_function(node):
    first = dest.x - node.x
    second = dest.y - node.y
    Final = math.sqrt((first)**2+(second)**2)
    return Final

#turf_checker(node): checks to see if point is inside of turf. If it is, then it will be set to 1.5
def turf_checker(node):
    node = Node(node.x, node.y)
    number = collision(node,turfs) #testing with turf values
    if number == None or number ==2 or number ==0:
        VALUE = 1
    if number ==1:
        VALUE = 1.5 #inside of turf
    return VALUE

#collision(node_value,List = []): checks to see how many times a point collides into a shape
def collision(node_value,List = []):
    temp = 0
    Number_of_collisions = 0
    for i in range(len(List)):  #This for loop will loop through each shape
        max_x = 0   #set at 0 so any number over 0 will be picked
        min_x = 55 #over the max so the lowest node value will be picked
        max_y = 0
        min_y = 55
        for i in range(len(List[temp]) - 1): #This for loop loops through the shapes points to create the verticies.
            node1 = List[temp][i]
            node2 = List[temp][i+1]
            if((node1.x==node_value.x and node1.y == node_value.y) or (node2.x ==node_value.x and node2.y == node_value.y)): #means that the point is on an edge
                Number_of_collisions = 1
                return Number_of_collisions
            if(node1.x>max_x):
                max_x = node1.x
            if(node2.x>max_x):
                max_x = node1.x
            if(node1.x<min_x):
                min_x = node1.x
            if(node2.x<min_x):
                min_x = node2.x
            if(node1.y>max_y):
                max_y = node1.y
            if(node2.y>max_y):
                max_y = node1.y
            if(node1.y<min_y):
                min_y = node1.y
            if(node2.y<min_y):
                min_y = node2.y
        if((node_value.x<=max_x and node_value.x>=min_x) and (node_value.y<=max_y and node_value.y>=min_y)): #checking to see that the x value is inside of the shapes min and max
            for i in range(len(List[temp])):     #Loops through all of the points in the shape
                if(len(List[temp])-1==i): #checks that the last node connects to the first node
                    node_one = List[temp][0]
                    node_two= List[temp][i]
                else:
                    node_one = List[temp][i]
                    node_two = List[temp][i+1]
                side =vertex(node_value,node_one,node_two)  #will go into vertex to calculate if it is inside the line being presented
                if side==1:
                    Number_of_collisions = Number_of_collisions+side
                if side!=None:
                    if side>=20:
                        Number_of_collisions = 1
                        return Number_of_collisions
                    if side>=13:
                        up_node = Node(node_value.x,node_value.y+1)
                        Test = vertex(up_node, node_one,node_two)
                        if(Test == 3) and up_node.x<node_one.x and up_node.x<node_two.x:
                            return 2
                        else:
                            Number_of_collisions = 1
                            return Number_of_collisions
                    if side>=6:
                        Number_of_collisions = 2
                        return Number_of_collisions
            return Number_of_collisions
        temp = temp+1

#vertex(): calculates the difference. if the difference is greater than the original point. The returned value will be
#checked in the collisions function.
def vertex(og_point,pointa,pointb):
    if pointa.y == pointb.y and og_point.y == pointa.y and pointb.y:
        check_og_point = Node(og_point.x,og_point)
        while check_og_point.x!=50: #checking to see if the point is on the right side of the shape
            check_og_point.x = check_og_point.x+1
            if(check_og_point.x==pointa.x or check_og_point.x ==pointb.x):
                return 13
        return 6
    if pointa.y>pointb.y:
        max = pointa.y
        min = pointb.y
    else:
        max = pointb.y
        min = pointa.y
    if og_point.y<max and og_point.y>=min:
        diff = ((og_point.y-pointa.y)*(pointb.x-pointa.x))/(pointb.y-pointa.y) + pointa.x
        if og_point.x<diff:
            return 1
        if og_point.x==diff:
            return 20
        else:
            return 0
    return 3
#find_shapes(): find the enclosed shapes points
def find_shapes():
    for polygon in epolygons:
        list = []
        for i in range(0, len(polygon)):
            list.append(polygon[i])
        Shape_list.append(list)
#find_turfs(): finds the turfs points
def find_turfs():
    for polygon in tpolygons:
        list = []
        for i in range(0, len(polygon)):
            list.append(polygon[i])
        turfs.append(list)

if __name__ == "__main__":
    epolygons = gen_polygons('TestingGrid/world1_enclosures.txt')
    tpolygons = gen_polygons('TestingGrid/world1_turfs.txt')

    #Test Values
    #epolygons = gen_polygons('TestingGrid/Testing_case_enclosures.txt')
    #tpolygons = gen_polygons('TestingGrid/Testing_case_turfs.txt')

    source = Point(8, 10)
    dest = Point(43, 45)
    problem = Problem(source)

    fig, ax = draw_board()
    draw_grids(ax)
    draw_source(ax, source.x, source.y)  # source point
    draw_dest(ax, dest.x, dest.y)  # destination point

    # Draw enclosure polygons
    for polygon in epolygons:
        for p in polygon:
            draw_point(ax, p.x, p.y)
    for polygon in epolygons:
        for i in range(0, len(polygon)):
            draw_line(ax, [polygon[i].x, polygon[(i+1)%len(polygon)].x], [polygon[i].y, polygon[(i+1)%len(polygon)].y])

    # Draw turf polygons
    for polygon in tpolygons:
        for p in polygon:
            draw_green_point(ax, p.x, p.y)
    for polygon in tpolygons:
        for i in range(0, len(polygon)):
            draw_green_line(ax, [polygon[i].x, polygon[(i+1)%len(polygon)].x], [polygon[i].y, polygon[(i+1)%len(polygon)].y])

    user = input("Enter 1:BFS, 2:DFS, 3:GBFS, 4:A* ")
    #### Here call your search to compute and collect res_path
    if user =="1":
        BFS(problem)
    if user =="2":
        DFS(problem)
    if user =="3":
        Greedy(problem)
    if user =="4":
        Astar(problem)

    for i in range(len(res_path)-1):
        draw_result_line(ax, [res_path[i].x, res_path[i+1].x], [res_path[i].y, res_path[i+1].y])
        #plt.pause(0.1)

    plt.show()
    print("Value of explored nodes:", EXPANDED_NODES)
    print("Value of path",PATH_NODES)

