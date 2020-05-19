from time import time
from matplotlib import pyplot as plt

import numpy as np
from sklearn.neighbors import KDTree
from colliders import Colliders

from planning_utils import create_grid, a_star_graph, heuristic, line_of_sigth_smoothing, cluster_grid_sampling
from planning_utils import random_grid_sampling, create_graph_from_nodes, subsample_grid

from plot_helpers import plot_polygons, plot_path, plot_points, plot_graph_edges


#start, goal = (316, 444), (800, 799)  # HOME Goal
#start, goal = (11, 362), (881, 907)   # Route A
#start, goal = (316, 444), (590, 135)   # Route B
#start, goal = (316, 444), (277, 711)   # Route C
start, goal = (557, 916), (177, 890)   # Route D
ALTITUDE, SAFETY_DISTANCE = 6, 5

np.random.seed(0)
obs = Colliders()

grid, nmin, emin = create_grid(obs.read_data(), ALTITUDE, SAFETY_DISTANCE)

graph_nodes = subsample_grid(grid, min_distance=10, seed=0)
print(len(graph_nodes))
tree_of_nodes = KDTree(graph_nodes)

graph_nodes.append(start)
graph_nodes.append(goal)
graph = create_graph_from_nodes(grid, nodes_list=graph_nodes, tree_of_nodes=tree_of_nodes, k=10)
print('searching on graphs')
path, cost = a_star_graph(graph, heuristic, start, goal)
print(len(path))

print('smoothing')
path = line_of_sigth_smoothing(grid, path, greedy=True)
print(len(path))


print('plotting')
polygons = obs.get_obstacles_as_polygons(ALTITUDE, SAFETY_DISTANCE)

ax = plot_polygons(polygons, show=False)
plot_points(graph_nodes, ax=ax, colour=(1, 0.82, 0.863), s=30, show=False)
plot_points([start, goal], ax=ax, colour='g', s=50, show=False)

plot_graph_edges(graph, ax=ax, alpha=0.1, show=False)
plot_path(path, ax=ax, show=True)
