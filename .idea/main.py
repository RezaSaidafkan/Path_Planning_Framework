__author__ = 'diablo'
from prm import *
from astar import *
from obstacle_away import *
from triangle import *
import matplotlib.pyplot as plt
import time

x_w, y_w = 100, 100
#x_s, y_s, x_f, y_f = (15, 90, 90, 90)
x_s, y_s, x_f, y_f = (15, 10, 90, 90)
#x_s, y_s, x_f, y_f = (90, 10, 15, 10)

N = 10*8 # number of nodes
world = World(x_w, y_w)
start_finish = StartFinish(x_s, y_s, x_f, y_f)
#obstalces_array = np.array([(30, 50, 80, 95), (10, 34, 35, 45)])
#obstalces_array = np.array([(23, 5, 80, 30), (30, 50, 80, 95), (10, 34, 35, 45)])
obstalces_array = np.array([(23, 5, 40, 30), (13, 70, 33, 85), (50, 50, 70, 70), (90, 60, 100, 80)])
#obstalces_array = np.array([(23, 0, 30, 60), (23, 75, 30, 100), (50, 50, 70, 100), (85, 50, 100, 70), (55, 25, 65, 45)])
obstacles = Obstacle_generator(world, obstalces_array)
nodes = node_generator(N, start_finish)

def detect_nodes():
    global _free_nodes
    _free_nodes = []
    for i in range(N + 2):
        counter = 0
        for j in range(len(obstacles.obstacles)):
            if not free_node(nodes[i], obstacles.obstacles[j]):
                break
	    counter += 1
    if counter == len(obstacles.obstacles):
	    _free_nodes.append(nodes[i])
detect_nodes()
print "free nodes = " + str(len(_free_nodes))

def detect_edges(obstacle):
    global _free_edges
    _free_edges = []
    for i in range(len(_free_nodes)):
        for j in range(len(_free_nodes)):
            counter = 0
            for k in range(len(obstacles.obstacles)):
                if not free_edge(_free_nodes[i], _free_nodes[j], obstacles.obstacles[k]):
                    break
            counter += 1
            if counter == len(obstacles.obstacles) and i != j:
                next_nodes(_free_nodes[j], _free_nodes[i])
                if not _free_nodes[j] in _free_nodes[i]._next_nodes:
                    _free_edges.append(Edge(_free_nodes[i], _free_nodes[j]))
                    edge_of_node(_free_nodes[i], Edge(_free_nodes[i], _free_nodes[j]))
detect_edges(obstacles)
print 'free edges = ' + str(len(_free_edges))

a_star = AStar(_free_nodes[0], _free_nodes[-1])
algorithm = a_star.algorithm()
path = a_star.path_constructer()
path = path[::-1]

desc_path = Desc_path(path, obstacles)
lo = desc_path.decs_edges()

corner_tirangles = Corner_triangle(lo, obstacles)
co = corner_tirangles.triangle()

#distance1 = Distance(obstacle1, path[1])
#distance2 = Distance(obstacle2, path[1])
#print 'distance of node from obstalce1 = ' + str(distance1.distance_method())
#print 'distance of node from obstalce2 = ' + str(distance2.distance_method())

figure = plt.figure()
axis = figure.add_subplot(111)
#plt.xlim((0, x_w))
#plt.ylim((0, y_w))
plt.grid(False)
plt.show(block=False)
plt.pcolor(world.world.T, cmap=plt.cm.gray_r)
for i in range(len(_free_nodes)):
    'draw all collision free nodes'
    plt.plot(_free_nodes[i].x_n, _free_nodes[i].y_n, 'bo')
for i in range(len(_free_edges)):
    'draw all the primary collision free edges'
    plt.plot((_free_edges[i].node1.x_n, _free_edges[i].node2.x_n), (_free_edges[i].node1.y_n, _free_edges[i].node2.y_n), 'g-', lw=0.4, alpha=0.5)
for i in path:
    'draw solution nodes'
    plt.plot(i.x_n, i.y_n, marker='o', color='k')
for i in path:
    'draw solution path'
    if i.parent_edge != None:
	plt.plot((i.x_n, i.parent_edge.node2.x_n), (i.y_n, i.parent_edge.node2.y_n), 'k', lw=3, alpha=1)
for i in range(len(lo)):
    plt.plot(lo[i].x_n, lo[i].y_n ,'ro')
for i in lo:
    'draw solution descrete path'
    if i.parent_edge != None:
	plt.plot((i.x_n, i.parent_edge.node1.x_n), (i.y_n, i.parent_edge.node1.y_n), 'r', lw=1, alpha=1)
for i in lo:
    'draw corner triangles'
    if i.corner:
	if i.corner.node21 and i.corner.node23:
	    plt.plot((i.corner.node21.x_n, i.corner.node23.x_n), (i.corner.node21.y_n, i.corner.node23.y_n), color='c', marker='x', lw=2)

print 'path length = ' + str(len(path) - 1)
plt.plot(x_s, y_s, x_f, y_f, color='r', marker='s')
plt.show()
