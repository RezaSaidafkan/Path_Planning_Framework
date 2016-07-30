__author__ = 'diablo'
from prm import *
import heapq


class AStar(object):
    def __init__(self, start_node, finish_node):
        self.start_node = start_node
        self.finish_node = finish_node
        self.closed_set = []
        self.open_set = [self.start_node]
        self.path = []
        self.start_node.g = 0
        self.start_node.f = self.start_node.g + start_node.distance

    def algorithm(self):
        while len(self.open_set) > 0:
            current_node = heapq.heappop(self.open_set)
            self.out = current_node
            self.closed_set.append(current_node)
            for i in current_node._next_nodes:
                if i in self.closed_set:
                    continue
                tentative_cost = current_node.g + length(current_node, i)
                if i not in self.open_set or tentative_cost < i.g:
                    i.parent_node = current_node
                    i.g = tentative_cost
                    i.f = i.g + i.distance
                    i.parent_edge = Edge(i, i.parent_node)
                    if i not in self.open_set:
                        heapq.heappush(self.open_set, i)

    def path_constructer(self):
        temp_node = self.finish_node
        while temp_node is not None:
            self.path.append(temp_node)
            temp_node = temp_node.parent_node
        return self.path

    def __cmp__(self, other):
        self.f = self.g + self.distance
        other.f = other.g + other.distance
        if self.f < other.f or (self.f == other.f and self.g > other.g):
            return -1
        elif self.f == other.f and self.g == other.g:
            return 0
        return 1
