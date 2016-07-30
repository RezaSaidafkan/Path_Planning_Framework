from prm import free_edge, Node

__author__ = 'diablo'
import numpy as np


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
            # self.node21 = node_angle(self.cnode, self.triangle.v12_incline2, d)
            # self.node23 = node_angle(self.cnode, self.triangle.v23_incline2, d)
            self.node21 = node_angle_b(self.cnode, self.node1, self.triangle.v12_incline2, d)
            self.node23 = node_angle_b(self.cnode, self.node3, self.triangle.v23_incline2, d)
            self.node21_c = np.sqrt((self.node21.x_n - self.cnode.x_n) ** 2 + (self.node21.y_n - self.cnode.y_n) ** 2)
            self.node23_c = np.sqrt((self.node23.x_n - self.cnode.x_n) ** 2 + (self.node23.y_n - self.cnode.y_n) ** 2)
            # self.node21 = node_angle_n(self.cnode, self.triangle.v12_incline2, d)
            # self.node23 = node_angle_n(self.cnode, self.triangle.v23_incline2, d)
            counter = 0
            for j in self.obstacles.obstacles:
                free_edge_ = free_edge(self.node21, self.node23, j)
                if not free_edge_:
                    continue
                counter += 1
                # if counter == len(self.obstacles.obstacles):
            # if d < self.edge23 and d < self.edge12:
            # d += dd
            # else:
            # break
            if counter == len(self.obstacles.obstacles):
                if d < min(self.node21_c, self.edge12) and d < min(self.node23_c, self.edge23):
                    d += dd
                else:
                    break
            else:
                break
                # if counter == len(self.obstacles.obstacles):
            # d += dd
            # else:
            # break
        self.distance = d

    # def node_angle_n(node, incline, distance):
    # x1 = (distance**2/(incline**2 + 1.0))**(0.5) + node.x_n
    # y1 = incline*(distance**2/(incline**2 + 1.0))**(0.5) + node.y_n
    # return Node(x1, y1)


def node_angle_b(cnode, anode, incline, distance):
    a1 = (distance ** 2 / (incline ** 2 + 1.0)) ** (0.5)
    a2 = incline * (distance ** 2 / (incline ** 2 + 1.0)) ** 0.5
    dx = cnode.x_n - anode.x_n
    dy = cnode.y_n - anode.y_n
    if dx > 0 and dy > 0:
        x1, y1 = -a1 + cnode.x_n, -a2 + cnode.y_n
    if dx < 0 < dy:
        x1, y1 = a1 + cnode.x_n, a2 + cnode.y_n
    if 0 > dx and dy < 0:
        x1, y1 = a1 + cnode.x_n, a2 + cnode.y_n
    if dx > 0 > dy:
        x1, y1 = -a1 + cnode.x_n, -a2 + cnode.y_n
    return Node(x1, y1)


def node_angle_n(cnode, anode, incline, distance):
    # a1 = np.sqrt((distance**2/(incline**2 + 1.0))**(0.5))
    # a2 = np.sqrt(incline*(distance**2/(incline**2 + 1.0))**(0.5))
    a1 = (distance ** 2 / (incline ** 2 + 1.0)) ** (0.5)
    a2 = incline * (distance ** 2 / (incline ** 2 + 1.0)) ** (0.5)
    dx = cnode.x_n - anode.x_n
    dy = cnode.y_n - anode.y_n
    # if dx > 0 and dy > 0 :
    # x1, y1 = -a1 + cnode.x_n, a2 + cnode.y_n
    # if dx < 0 and dy > 0 :
    # x1, y1 = a1 + cnode.x_n, -a2 + cnode.y_n
    # if dx < 0 and dy < 0 :
    # x1, y1 = a1 + cnode.x_n, a2 + cnode.y_n
    # if dx > 0 and dy < 0 :
    # x1, y1 = -a1 + cnode.x_n, +a2 + cnode.y_n
    if 0 < dx and dy > 0:
        x1, y1 = -a1 + cnode.x_n, -a2 + cnode.y_n
    if dx < 0 < dy:
        x1, y1 = a1 + cnode.x_n, a2 + cnode.y_n
    if dx < 0 and dy < 0:
        x1, y1 = a1 + cnode.x_n, a2 + cnode.y_n
    if dx > 0 > dy:
        x1, y1 = a1 + cnode.x_n, a2 + cnode.y_n
    return Node(x1, y1)
