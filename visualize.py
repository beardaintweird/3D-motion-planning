import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

def visualize_points(grid, data, path, graph, start):
    fig = plt.figure()

    plt.imshow(grid, cmap='Greys', origin='lower')

    nmin = np.min(data[:, 0])
    emin = np.min(data[:, 1])

    # draw nodes
    for n1 in graph.nodes:
        plt.scatter(n1[1]-emin, n1[0]-nmin, c='red')
    if start in graph.nodes:
        plt.scatter(start[1]-emin, start[0]-nmin, c='blue')
    # draw edges
    for (n1, n2) in graph.edges:
        plt.plot([n1[1]-emin , n2[1]-emin ], [n1[0]-nmin, n2[0]-nmin], 'black')

    # TODO: add code to visualize the path
    path_pairs = zip(path[:-1], path[1:])
    for (n1, n2) in path_pairs:
        plt.plot([n1[1]-emin , n2[1]-emin ], [n1[0]-nmin, n2[0]-nmin], 'yellow')

    plt.xlabel('NORTH')
    plt.ylabel('EAST')

    plt.show()
