__author__ = 'diablo'
import math
from astar import *


class Triangle(object):
    def __init__(self, node1, node2, node3):
        self.d = 5
        self.node1 = node1
        self.node2 = node2
        self.node3 = node3
        self.v21 = np.array([(node2.x_n - node1.x_n), (node2.y_n - node1.y_n)])
        self.v21_incline = math.atan2(self.v21[1], self.v21[0])
        self.v21_incline2 = (self.node2.y_n - self.node1.y_n) / (self.node2.x_n - self.node1.x_n)
        self.v12 = np.array([(node1.x_n - node2.x_n), (node1.y_n - node2.y_n)])
        self.v12_incline = math.atan2(self.v12[1], self.v12[0])
        self.v12_incline2 = (self.node1.y_n - self.node2.y_n) / (self.node1.x_n - self.node2.x_n)
        self.v12_incline3 = (self.node1.y_n - self.node2.y_n) / (self.node1.x_n - self.node2.x_n) - 0.5
        self.v31 = np.array([(node3.x_n - node1.x_n), (node3.y_n - node1.y_n)])
        self.v31_incline = math.atan2(self.v31[1], self.v31[0])
        self.v13 = np.array([(node1.x_n - node3.x_n), (node1.y_n - node3.y_n)])
        self.v13_incline = math.atan2(self.v13[1], self.v13[0])
        self.v32 = np.array([(node3.x_n - node2.x_n), (node3.y_n - node2.y_n)])
        self.v32_incline = math.atan2(self.v32[1], self.v32[0])
        self.v23 = np.array([(node2.x_n - node3.x_n), (node2.y_n - node3.y_n)])
        self.v23_incline = math.atan2(self.v23[1], self.v23[0])
        self.v23_incline2 = (self.node2.y_n - self.node3.y_n) / (self.node2.x_n - self.node3.x_n)
        self.v32_v21_angle = np.abs(np.abs((self.v32_incline - self.v21_incline) * 180 / np.pi) - 180)
        self.v31_v23_angle = np.abs(np.abs((self.v31_incline - self.v23_incline) * 180 / np.pi) - 180)
        self.v13_v21_angle = np.abs(np.abs((self.v13_incline - self.v21_incline) * 180 / np.pi) - 180)
        self.desc_angle_u = self.v32_v21_angle - self.v32_v21_angle % self.d
        self.desc_angle_l = self.v32_v21_angle - self.v32_v21_angle % self.d + self.d
        self.angle_change = self.v32_v21_angle % self.d
        # self.incline_change = self.v32_v21_angle%self.d*np.pi/180
        self.incline_change = self.d * np.pi / 180
        self.desc_incline_u = self.v21_incline + self.incline_change
        self.desc_incline_l = self.v21_incline - self.incline_change
        self.desc_incline_u2 = self.v21_incline2 + self.incline_change
        self.desc_incline_l2 = self.v21_incline2 - self.incline_change
        self.m1 = (self.node2.y_n - self.node1.y_n) / (self.node2.x_n - self.node1.x_n)
        self.m2 = (self.node3.y_n - self.node2.y_n) / (self.node3.x_n - self.node2.x_n)

        self.this_node = None


def line_intersect(node1, node2, incline1, incline2):
    A = (-incline2 * node2.x_n + incline1 * node1.x_n - node1.y_n + node2.y_n) / (incline1 - incline2)
    x = A
    y = incline1 * (A - node1.x_n) + node1.y_n
    return (x, y)


class Desc_path(object):
    def __init__(self, initial_answer_path, obstacles):
        self.initial_answer_path = initial_answer_path
        self.obstacles = obstacles
        self.open_set = []
        self.final_edges = []

    def decs_edges(self):
        init = self.initial_answer_path[0]
        final = node_copy(self.initial_answer_path[-1])
        self.this_node = init
        self.initial_answer_path.append(final)
        for i in self.initial_answer_path:  # seach through all points of trivial path
            triangle_temp = None
            j = i.parent_node
            if j is not None:  # introducing
                k = j.parent_node
                node_temp = None
                if k is not None:
                    for m in ('u', 'l'):
                        if self.tri(i, m):
                            break
        return self.open_set

    def tri(self, node3, u_or_l):
        triangle_temp = Triangle(self.this_node, node3.parent_node, node3)  # introducing nodes 1 to 3 as triangle
        if u_or_l == 'l':
            line_temp = line_intersect(node3.parent_node.parent_node, node3, triangle_temp.desc_incline_l2,
                                       triangle_temp.v23_incline2)  # getting coordination of the node on the v23
        if u_or_l == 'u':
            line_temp = line_intersect(node3.parent_node.parent_node, node3, triangle_temp.desc_incline_u2,
                                       triangle_temp.v23_incline2)  # getting coordination of the node on the v23
            node_temp = Node(line_temp[0], line_temp[1])
            counter = 0  # checking to see the new edge doesn't intesect with the objects
        for l in range(len(self.obstacles.obstacles)):
            if not free_edge(node3.parent_node, node_temp, self.obstacles.obstacles[l]):
                return False
        counter += 1
        corner = None
        if counter == len(self.obstacles.obstacles):  # check ends here
            self.open_set.append(node_temp)
            node_temp.parent_node = self.this_node
            this_edge = Edge(self.this_node, node_temp)
            node_temp.parent_edge = this_edge
            self.this_node = node_temp
            return True


class Corner(object):
    def __init__(self, triangle, obstacles):
        self.triangle = triangle
        self.cnode = triangle.node2
        self.node3 = triangle.node3
        self.node1 = triangle.node1
        self.angle = triangle.desc_incline_l2  # there is a need to decide between u2 or l2
        self.obstacles = obstacles
        self.distance = None
        self.node21 = None
        self.node23 = None
        self.node21_c = None
        self.node23_c = None
        self.edge12 = np.sqrt((self.node1.x_n - self.cnode.x_n) ** 2 + (self.node1.y_n - self.cnode.y_n) ** 2)
        self.edge23 = np.sqrt((self.node3.x_n - self.cnode.x_n) ** 2 + (self.node3.y_n - self.cnode.y_n) ** 2)

    def node_by_distance(self):
        dd = 0.5
        d = 0.005
        free_edge_ = True
        while free_edge_:
            self.node21 = node_angle_b(self.cnode, self.node1, self.triangle.v12_incline2, d)
            self.node23 = node_angle_b(self.cnode, self.node3, self.triangle.v23_incline2, d)
            self.node21_c = np.sqrt((self.node21.x_n - self.cnode.x_n) ** 2 + (self.node21.y_n - self.cnode.y_n) ** 2)
            self.node23_c = np.sqrt((self.node23.x_n - self.cnode.x_n) ** 2 + (self.node23.y_n - self.cnode.y_n) ** 2)
            counter = 0
            for j in self.obstacles.obstacles:
                free_edge_ = free_edge(self.node21, self.node23, j)
            if not free_edge_:
                continue
            counter += 1
            if counter == len(self.obstacles.obstacles) and d < self.edge12 and d < self.edge23:
                d += dd
            else:
                break
            self.distance = d


def node_angle_b(cnode, anode, incline, distance):
    a1 = (distance ** 2 / (incline ** 2 + 1.0)) ** (0.5)
    a2 = incline * (distance ** 2 / (incline ** 2 + 1.0)) ** (0.5)
    dx = cnode.x_n - anode.x_n
    dy = cnode.y_n - anode.y_n
    if 0 < dx and dy > 0:
        x1, y1 = -a1 + cnode.x_n, -a2 + cnode.y_n
    if dx < 0 < dy:
        x1, y1 = a1 + cnode.x_n, a2 + cnode.y_n
    if dx < 0 and dy < 0:
        x1, y1 = a1 + cnode.x_n, a2 + cnode.y_n
    if dx > 0 and dy < 0:
        x1, y1 = -a1 + cnode.x_n, -a2 + cnode.y_n
    return Node(x1, y1)


def node_angle_n(cnode, anode, incline, distance):
    a1 = (distance ** 2 / (incline ** 2 + 1.0)) ** (0.5)
    a2 = incline * (distance ** 2 / (incline ** 2 + 1.0)) ** (0.5)
    dx = cnode.x_n - anode.x_n
    dy = cnode.y_n - anode.y_n
    if 0 < dx and dy > 0:
        x1, y1 = -a1 + cnode.x_n, -a2 + cnode.y_n
    if dx < 0 < dy:
        x1, y1 = a1 + cnode.x_n, a2 + cnode.y_n
    if dx < 0 and dy < 0:
        x1, y1 = a1 + cnode.x_n, a2 + cnode.y_n
    if dx > 0 > dy:
        x1, y1 = a1 + cnode.x_n, a2 + cnode.y_n
    return Node(x1, y1)


class Corner_triangle(object):
    def __init__(self, disc_path, obstacles):
        self.disc_path = disc_path
        self.obstacles = obstacles

    def triangle(self):
        for i in self.disc_path:  # seach through all points of trivial path
            j = i.parent_node
            if j is not None:  # introducing
                k = j.parent_node
                if k is not None:
                    i.parent_node.corner = Corner(Triangle(i.parent_node.parent_node, i.parent_node, i), self.obstacles)
                    i.parent_node.corner.node_by_distance()
