__author__ = 'diablo'
import math

""" get every self.obstacle as an object
 calculate the 9 zones according to each object
 calculate distance of a node from the self.obstacle according to the zones
 for every x compare whether if y(x) is above or below the expected
 """


class Distance():
    def __init__(self, obstacle, node):
        self.obstacle = obstacle
        self.node = node

    def distance_cases(self):
        if self.node.x_n < self.obstacle.x_0 and self.node.y_n < self.obstacle.y_0:
            return self.circular_distance((self.obstacle.y_0, self.obstacle.x_0))

        if self.node.x_n < self.obstacle.x_0 and self.obstacle.y_0 < self.node.y_n < self.obstacle.y_f:
            return self.horizontal_distance(self.obstacle.x_0)

        if self.node.x_n < self.obstacle.x_0 and self.node.y_n > self.obstacle.y_f:
            return self.circular_distance((self.obstacle.y_f, self.obstacle.x_0))

        if self.obstacle.x_0 < self.node.x_n < self.obstacle.x_f and self.node.y_n < self.obstacle.y_0:
            return self.vertical_distance(self.obstacle.y_0)

        if self.obstacle.x_0 < self.node.x_n < self.obstacle.x_f and self.obstacle.y_0 < self.node.y_n < self.obstacle.y_f:
            return "error"

        if self.obstacle.x_0 < self.node.x_n < self.obstacle.x_f and self.node.y_n > self.obstacle.y_f:
            return self.vertical_distance(self.obstacle.y_f)

        if self.node.x_n > self.obstacle.x_f and self.node.y_n < self.obstacle.y_0:
            return self.circular_distance((self.obstacle.y_0, self.obstacle.x_f))

        if self.node.x_n > self.obstacle.x_f and self.obstacle.y_0 < self.node.y_n < self.obstacle.y_f:
            return self.horizontal_distance(self.obstacle.x_f)

        if self.node.x_n > self.obstacle.x_f and self.node.y_n > self.obstacle.y_f:
            return self.circular_distance((self.obstacle.y_f, self.obstacle.x_f))

    def circular_distance(self, point):
        dy = abs(self.node.y_n - point[0])
        dx = abs(self.node.x_n - point[1])
        return math.sqrt(dx ** 2 + dy ** 2)

    def horizontal_distance(self, pointed):
        return abs(self.node.x_n - pointed)

    def vertical_distance(self, pointed):
        return abs(self.node.y_n - pointed)

    def distance_method(self):
        return self.distance_cases()
