import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def convert2networkx(data):
    p = []
    for i in data:
        p.append([(i[0], i[1], i[2]), (i[3], i[4], i[5]),
                  (i[6], i[7], i[8]), (i[9], i[10], i[11])])
        p.append([(i[0], -i[1], i[2]), (i[3], -i[4], i[5]),
                  (i[6], -i[7], i[8]), (i[9], -i[10], i[11])])
    G = nx.Graph()
    for polygon in p:
        for vertices in polygon:
            G.add_node(vertices, pos=vertices)
        for i in range(len(polygon)):
            G.add_edge(polygon[i], polygon[(i + 1) % len(polygon)])
            G.add_edge(polygon[-1], polygon[0])
            G.add_edge(polygon[0], polygon[2])
            G.add_edge(polygon[1], polygon[3])
    return G


def plot_network(G):
    node_pos = {node: node for node in G.nodes()}

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Draw nodes as points
    for node, pos in node_pos.items():
        ax.scatter(*pos, c='k', s=1e-1)

    # Draw edges as lines
    for edge in G.edges():
        ax.plot(*zip(node_pos[edge[0]], node_pos[edge[1]]), c='k', linewidth=0.5)

    # Set plot limits and axis labels
    ax.set_xlim3d(-100, 100)
    ax.set_ylim3d(-50, 50)
    ax.set_zlim3d(-10, 20)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    return fig, ax


def plot_3dpoly(data):
    p = []
    for i in data:
        p.append([(i[0], i[1], i[2]), (i[3], i[4], i[5]),
                  (i[6], i[7], i[8]), (i[9], i[10], i[11])])
        p.append([(i[0], -i[1], i[2]), (i[3], -i[4], i[5]),
                  (i[6], -i[7], i[8]), (i[9], -i[10], i[11])])
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    poly = Poly3DCollection(p)
    poly.set_alpha(0.5)
    ax.add_collection3d(poly)
    ax.set_xlim3d(-100, 100)
    ax.set_ylim3d(-50, 50)
    ax.set_zlim3d(-10, 20)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    return fig, ax

# data = np.genfromtxt('points.csv', delimiter=',')
# G = convert2networkx(data)
# plot_network(G)
