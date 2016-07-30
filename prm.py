__author__ = 'diablo'

import numpy as np
from matplotlib.transforms import Bbox
from matplotlib.path import Path


class World(object):
    def __init__(self, x_w, y_w):
        self.x_w = x_w
        self.y_w = y_w
        self.world = np.zeros((self.x_w, self.y_w))
        self.x_s = None
        self.x_f = None
        self.y_s = None
        self.y_f = None

    def StartFinish(self, x_s, y_s, x_f, y_f):
        self.x_s = x_s
        self.x_f = x_f
        self.y_s = y_s
        self.y_f = y_f


class Node(object):
    def __init__(self, x_n, y_n):
        self.x_n = x_n
        self.y_n = y_n
        self.g = None
        self.f = None
        self.distance = None
        self.parent_node = None
        self.parent_edge = None
        self._next_nodes = []
        self._next_edges = []
        self.corner = None
        self.hey = None

    def __eq__(self, other):
        self.x == other.x
        self.y == other.y


class Obstacle(object):
    def __init__(self, array, world):
        self.array = array
        self.x_0 = self.array[0]
        self.y_0 = self.array[1]
        self.x_f = self.array[2]
        self.y_f = self.array[3]
        self.world = world
        self.dist = 5
        # self.Bbox = Bbox.from_bounds(self.x_0, self.y_0 , \
        # (self.x_f - self.x_0), (self.y_f - self.y_0))
        # for i in range(self.x_0, self.x_f):
        # for j in range(self.y_0, self.y_f):
        # self.world.world[i][j] = 1
        self.Bbox = Bbox.from_bounds(self.x_0 - self.dist, self.y_0 - self.dist, \
                                     (self.x_f - self.x_0 + self.dist), (self.y_f - self.y_0 + self.dist))
        for i in range(self.x_0, self.x_f):
            for j in range(self.y_0, self.y_f):
                self.world.world[i][j] = 1


class Obstacle_generator(object):
    def __init__(self, world, array):
        self.world = world
        self.array = array
        self.M = len(self.array)
        self.obstacles = []
        for i in self.array:
            self.obstacles.append(Obstacle(i, self.world))


class Edge(object):
    def __init__(self, node1, node2):
        self.node1 = node1
        self.node2 = node2
        self.length = np.sqrt((node1.x_n - node2.x_n) ** 2 + (node1.y_n - node2.y_n) ** 2)


def node_generator(N, world):
    ''' generating nodes '''
    nodes = np.empty(N + 2, dtype=np.object)
    nodes[0] = Node(world.x_s, world.y_s)
    nodes[0].distance = np.sqrt((world.x_s - world.x_f) ** 2 + (world.y_s - world.y_f) ** 2)
    nodes[-1] = Node(world.x_f, world.y_f)
    nodes[-1].distance = 0
    for i in range(1, N + 1):
        random = 600 * np.random.random(2)
        nodes[i] = Node(random[0], random[1])
        nodes[i].distance = np.sqrt((nodes[i].x_n - world.x_f) ** 2 + (nodes[i].y_n - world.y_f) ** 2)
    return nodes


def edge_of_node(node, edge):
    node._next_edges.append(edge)


def next_nodes(node1, node2):
    if not node2 in node1._next_nodes:
        node1._next_nodes.append(node2)


def length(node1, node2):
    return np.sqrt((node1.x_n - node2.x_n) ** 2 + (node1.y_n - node2.y_n) ** 2)


def node_copy(node1):
    return Node(node1.x_n, node1.y_n)


def test_free_node(node, obstacle):
    ''' tests whether a given node intersects the given obstacle '''
    return not (obstacle.x_0 <= node.x_n <= obstacle.x_f \
                and obstacle.y_0 <= node.y_n <= obstacle.y_f)


def test_free_edge(node1, node2, obstacle):
    """
    :param node1:
    :param node2:
    :param obstacle:
    :return: whether a line from node1 to node2 intersects obstacle or not. returns True if they do not intersect
    """
    return not Path(np.array([(node1.x_n, node1.y_n), \
                              (node2.x_n, node2.y_n)])).intersects_bbox(obstacle.Bbox)


def detect_free_nodes(_N, _nodes, _obstacles):
    """ :returns a list of free nodes intersecting any obstacles
     _obstacle is an:
     _nodes is an:
     _N is number of primary random nodes
     """
    _free_nodes = []
    for i in range(_N + 2):
        counter = 0
        for j in range(len(_obstacles.obstacles)):
            if not test_free_node(_nodes[i], _obstacles.obstacles[j]):
                break
            counter += 1
        if counter == len(_obstacles.obstacles):
            _free_nodes.append(_nodes[i])
    return _free_nodes


def detect_free_edges(_obstacles, _free_nodes):
    """

    :type _obstacles: object
    """

    _free_edges = []
    for i in range(len(_free_nodes)):
        for j in range(len(_free_nodes)):
            counter = 0
            for k in range(len(_obstacles.obstacles)):
                if not test_free_edge(_free_nodes[i], _free_nodes[j], _obstacles.obstacles[k]):
                    break
                counter += 1
            if counter == len(_obstacles.obstacles) and i != j:
#                next_nodes(_free_nodes[j], _free_nodes[i])
                if not _free_nodes[j] in _free_nodes[i]._next_nodes:
                    _free_edges.append(Edge(_free_nodes[i], _free_nodes[j]))
                    edge_of_node(_free_nodes[i], Edge(_free_nodes[i], _free_nodes[j]))
    return _free_edges
