import numpy as np
from matplotlib import pyplot as plt
from matplotlib.collections import LineCollection


def plot_points(point_list, ax=None, show=True, s=20, marker='o', colour='b'):
    if ax is None:
        f, ax = plt.subplots(1, 1)

    for point in point_list:
            ax.scatter(point[1], point[0], s=s, marker=marker, c=colour)
    if show:
        plt.show()

    return ax

def plot_graph_edges(graph, ax=None, alpha=0.1, colour='k', show=True):
    if ax is None:
        f, ax = plt.subplots(1, 1)

    segments = []
    for edge in graph.edges:
        segments.append((edge[0][::-1], edge[1][::-1]))

    ln_coll = LineCollection(segments, colors=[colour]*len(graph.edges), alpha=alpha)
    ax.add_collection(ln_coll)

    if show:
        plt.show()


def plot_path(path, ax=None, show=True):
    from matplotlib import pyplot as plt

    if ax is None:
        f, ax = plt.subplots(1, 1)

    np_path = np.array(path)
    ax.plot(np_path[:, 1], np_path[:, 0], '--')

    if show:
        plt.show()
        plt.axis('equal')

    return ax


def contour_polygons(grid, ax=None, show=True):
    if ax is None:
        f, ax = plt.subplots(1, 1)

    plt.contourf(grid, cmap='binary')

    plt.xlabel('East')
    plt.ylabel('North')

    if show:
        plt.show()
        plt.axis('equal')

    return ax


def plot_polygons(polygon_list, ax=None, show=True, colour='k'):
    if ax is None:
        f, ax = plt.subplots(1, 1)

    segments = []
    for p in polygon_list:
        north, east = p.exterior.xy
        segments.append(( (east[0], north[0]), (east[1], north[1]), (east[2], north[2]), (east[3], north[3]), (east[4], north[4])))

    ln_coll = LineCollection(segments, colors=[colour]*len(polygon_list))
    ax.add_collection(ln_coll)

    plt.xlabel('East')
    plt.ylabel('North')

    if show:
        plt.show()
        plt.axis('equal')

    return ax
