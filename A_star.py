import matplotlib.pyplot as plt
import numpy as np
from queue import PriorityQueue
from data import convert2networkx, plot_network, plot_3dpoly
import networkx as nx
import random
import itertools


def distance(point1, point2):
    x1, y1, z1 = point1
    x2, y2, z2 = point2
    return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)


def A_star(graph, start, end):
    visited = []

    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    f_score = {node: float('inf') for node in graph}
    f_score[start] = distance(start, end)

    open = PriorityQueue()
    open.put((f_score[start], distance(start, end), start))
    Path = {node: None for node in graph}

    while not open.empty():
        curr = open.get()[2]
        visited.append(curr)
        if curr == end:
            break
        for neighbor in graph.neighbors(curr):
            if neighbor in visited:
                continue
            temp_g_score = g_score[curr] + distance(curr, neighbor)
            temp_f_score = temp_g_score + distance(neighbor, end)

            if temp_f_score < f_score[neighbor]:
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_f_score
                open.put((temp_f_score, distance(neighbor, end), neighbor))
                Path[neighbor] = curr

    opt_path = []
    node = end
    while node != start:
        opt_path.append(node)
        node = Path[node]
    opt_path.append(start)
    # return the optimal path (reverse of collected points)
    return opt_path[::-1]


def tsp_perm_3d(start, end, points):
    """Solve the Traveling Salesman Problem using a permutation method"""
    # Generate all possible permutations of the points
    permutations = itertools.permutations(points)

    # Find the shortest path by calculating the distance of each permutation
    shortest_path = None
    shortest_distance = float('inf')
    for permutation in permutations:
        path_distance = 0
        path_distance += distance(permutation[0], start)
        for i in range(len(permutation) - 1):
            path_distance += distance(permutation[i], permutation[i + 1])
        path_distance += distance(permutation[-1], end)
        if path_distance < shortest_distance:
            shortest_path = permutation
            shortest_distance = path_distance

    return shortest_path  # return a list consist of points with the shortest path


def find_shortest_path(graph, start, end, waypoints, f):
    waypoints = tsp_perm_3d(start, end, waypoints)
    path = []
    for i in range(len(waypoints)):
        if i == 0:
            subpath = f(graph, start, waypoints[i])
        else:
            subpath = f(graph, waypoints[i - 1], waypoints[i])
        path = path + subpath[:-1]
    path = path + f(graph, waypoints[-1], end)
    return path


if __name__ == '__main__':
    data = np.genfromtxt('points.csv', delimiter=',')
    G = convert2networkx(data)
    # start = random.choice([node for node in G])
    # end = random.choice([node for node in G])
    start = (-128.24892, 0, 0)
    end = (123.23618, 0, -0.714)
    waypoints = random.choices([node for node in G], k=3)
    path = find_shortest_path(G, start, end, waypoints, A_star)
    # path2 = find_shortest_path(G, start, end, waypoints, nx.astar_path)
    # fig, ax = plot_network(G)
    fig, ax = plot_3dpoly(data)
    ax.plot([i[0] for i in path], [i[1] for i in path], [i[2] for i in path], c='r')
    # ax.plot([i[0] for i in path2], [i[1] for i in path2], [i[2] for i in path2], c='y')
    ax.scatter(start[0], start[1], start[2], c='g')
    ax.scatter(end[0], end[1], end[2], c='r')
    for point in waypoints:
        ax.scatter(point[0], point[1], point[2], c='b')
    plt.show()


