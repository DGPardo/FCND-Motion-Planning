from enum import Enum
from queue import PriorityQueue
import numpy as np

from bresenham import bresenham
import networkx as nx

from sklearn.neighbors import KDTree
from sklearn.cluster import MiniBatchKMeans


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


def local_to_grid_coordinates(data, local_position):
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    north, east = local_position[0], local_position[1]
    grid_north = int(north - north_min)
    grid_east = int(east - east_min)
    return (grid_north, grid_east)


def grid_to_local_coordinates(data, grid_coordinates):
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    north, east = grid_coordinates[0], grid_coordinates[1]
    grid_north = int(north + north_min)
    grid_east = int(east + east_min)
    return (grid_north, grid_east)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)

    NORTH_EAST = (-1, +1, 1.41421356237)  #1.41421356237 = sqrt(2)
    NORTH_WEST = (-1, -1, 1.41421356237)

    SOUTH_EAST = (1, +1, 1.41421356237)
    SOUTH_WEST = (1, -1, 1.41421356237)

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return self.value[0], self.value[1]

    @staticmethod
    def add(tuple1, tuple2):
        return tuple1[0] + tuple2[0], tuple1[1] + tuple2[1]


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape

    # check if the node is off the grid or
    # it's an obstacle
    for action in Action:
        i, j = Action.add(action.delta, current_node)
        if not (0 <= i < n and 0 <= j < m and grid[i, j] == 0):
            valid_actions.remove(action)

    return valid_actions


def a_star(grid, h, start, goal):
    def is_valid_grid_point(point):
        n, m = grid.shape
        return 0 <= point[0] < n and 0 <= point[1] < m and grid[point[0], point[1]] == 0

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    if not is_valid_grid_point(start):
        raise Exception('start is either within an obstacle or outside the grid domain')
    if not is_valid_grid_point(goal):
        raise Exception('goal is either within an obstacle or outside the grid domain')

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost


def a_star_graph(graph, h, start, goal):
    def is_valid_graph_node(point):
        return point in graph.nodes

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    if not is_valid_graph_node(start):
        raise Exception('start is either within an obstacle or outside the grid domain')
    if not is_valid_graph_node(goal):
        raise Exception('goal is either within an obstacle or outside the grid domain')

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                edge_cost = graph.edges[current_node, next_node]['weight']
                branch_cost = current_cost + edge_cost
                queue_cost = branch_cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node)
                    queue.put((queue_cost, next_node))

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])

    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')

    return path[::-1], path_cost


def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))


def collinear(p1, p2, p3):
    if p1 != p2 != p3:
        double_triangle_area = p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1])
        return True if double_triangle_area == 0 else False
    else:
        raise Exception('The 3 input points must be different to each other')


def prune_collinear_points(path_waypoints):
    if len(path_waypoints) < 3:
        return path_waypoints

    remove_ids = [idx + 1 for idx in range(len(path_waypoints) - 2)
                  if collinear(path_waypoints[idx], path_waypoints[idx + 1], path_waypoints[idx + 2])]

    for entry in remove_ids[::-1]:
        path_waypoints.pop(entry)

    return path_waypoints


def line_of_sigth_smoothing(grid, path, greedy=False):
    def intersects(start, end):
        return any(grid[p[0], p[1]] == 1 for p in bresenham(start[0], start[1], end[0], end[1]))

    if len(path) < 3:
        return path

    idx = 0
    while idx < len(path) - 2:
        if not intersects(path[idx], path[idx + 2]):
            path.pop(idx + 1)
            idx = 0 if greedy else idx
        else:
            idx += 1
    return path


def random_grid_sampling(grid, size, seed=0):
    np.random.seed(seed)
    n, m = grid.shape
    ii = np.random.randint(0, n, size)  # random integer in [0, n)
    jj = np.random.randint(0, m, size)  # random integer in [0, m)

    final_points = []
    for i, j in zip(ii, jj):
        if grid[i, j] == 0:
            final_points.append((i, j))
    return final_points


def cluster_grid_sampling(grid, size, seed=0):
    coordinates = [(i, j) for i in range(grid.shape[0]) for j in range(grid.shape[1]) if grid[i, j] == 0]
    kmeans = MiniBatchKMeans(n_clusters=size, init_size=3*size, random_state=seed)
    kmeans.fit(coordinates)

    cluster_centres = set()
    for ii, jj in kmeans.cluster_centers_:
        cluster_centre = (int(round(ii)), int(round(jj)))
        if grid[cluster_centre[0], cluster_centre[1]] == 0:
            cluster_centres.add(cluster_centre)
    return list(cluster_centres)


def subsample_grid(grid, min_distance, max_iter=None, seed=0, max_points=1E4):
    """
    Samples the grid randomnly checking for proximity with previously found nodes
    This way it is possible to limit the nodes found by proximity in order to spread
    the nodes more uniformly on the grid
    """
    np.random.random(seed)
    found_points = set()
    iteration_counter = 0

    # estimated number of grid points that can be found
    target_nsamples = min(int(grid.shape[0] * grid.shape[1] / (np.pi * min_distance**2 / 2)), max_points)
    max_iter = 5 * target_nsamples if max_iter is None else max_iter

    while len(found_points) < target_nsamples and iteration_counter < max_iter:
        i = np.random.randint(0, grid.shape[0])
        j = np.random.randint(0, grid.shape[1])

        if grid[i, j] == 0:
            # It is not blocked
            is_far_enough = True
            for entry in found_points:
                i_dist = abs(i - entry[0])
                j_dist = abs(j - entry[1])
                if i_dist + j_dist < min_distance:  # upper bound of true distance
                    is_far_enough = False
                    break
            if is_far_enough:
                found_points.add((i, j))

        iteration_counter += 1
    return list(found_points)


def get_closest_node_to_point(point, nodes, tree_of_nodes):
    idx = tree_of_nodes.query(np.atleast_2d(point), k=1, return_distance=False)[0][0]
    return nodes[idx]


def create_graph_from_nodes(grid, nodes_list, tree_of_nodes=None, k=10, max_edges_per_node=3):
    graph = nx.Graph()

    if tree_of_nodes is None:
        tree_of_nodes = KDTree(nodes_list)

    if k > len(nodes_list):
        k = len(nodes_list)
        print(f'k is now {k}')

    for node in nodes_list:
        graph.add_node(node)

    edge_count = 0
    for node in nodes_list:
        connected_nodes = len(graph[node])
        if connected_nodes < max_edges_per_node:
            near_node_ids = tree_of_nodes.query(np.atleast_2d(node), k=k, return_distance=False)[0]
            for node_id in near_node_ids:
                next_node = nodes_list[node_id]

                can_connect = True
                for entry in bresenham(node[0], node[1], next_node[0], next_node[1]):
                    if grid[entry[0], entry[1]] == 1:
                        can_connect = False
                        break

                if can_connect:
                    weight = np.linalg.norm(np.array(node) - np.array(next_node))
                    graph.add_edge(node, next_node, weight=weight)
                    edge_count += 1
    return graph


def tuple_of_ints(tuple_input):
    return tuple([int(entry) for entry in tuple_input])
