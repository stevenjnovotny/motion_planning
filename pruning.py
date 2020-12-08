import numpy as np
from planning_utils import check_edge

def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=1e-6):   
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon


def prune_path(path, epsilon = 1.e-6):
    pruned_path = [p for p in path]

    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        
        # If the 3 points are in a line remove
        # the 2nd point.
        # The 3rd point now becomes and 2nd point
        # and the check is redone with a new third point
        # on the next iteration.
        if collinearity_check(p1, p2, p3, epsilon=epsilon):

            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path

def get_pruned_path(path, grid):
    # get maximum-pruned path
    eps = 10000
    hit = True
    pruned_path = []
    max_dist = 0.0
    while hit == True:
        pruned_path = prune_path(path, epsilon = eps)
        for i in range(len(pruned_path) - 1):
            p1 = pruned_path[i]
            p2 = pruned_path[i+1]
            hit = check_edge(p1, p2, grid)
            if hit:
                break

            edge_dist = np.linalg.norm(np.array(p1) - np.array(p2))
            #print(edge_dist)
            if hit == False and edge_dist > max_dist:
                max_dist = edge_dist
        eps *= 0.5

    return pruned_path
