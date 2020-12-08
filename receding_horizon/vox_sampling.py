import numpy as np


def vox_sample(n_samples = 10, voxmap = None):

    xmin = 0
    xmax = voxmap.shape[0]
    ymin = 0
    ymax = voxmap.shape[1]
    zmin = 0
    zmax = voxmap.shape[2]

    xvals = np.random.randint(xmin, xmax, n_samples, dtype = int)
    yvals = np.random.randint(ymin, ymax, n_samples, dtype = int)
    zvals = np.random.randint(zmin, zmax, n_samples, dtype = int)
    samples = list(zip(xvals, yvals, zvals))

    points = []
    for vox in samples:
        if voxmap[vox] == False:
            points.append(vox)
    return points


if __name__ == "__main__":
    
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    # prepare some coordinates

    x, y, z = np.indices((40, 30, 12))

    # draw cuboids in the top left and bottom right corners, and a link between them
    cube1 = (x < 10) & (y < 10) & (z < 3)
    cube2 = (x >=30) & (y >= 20) & (z <= 5)
    link = (x < 10) & (y >= 20) & (z <= 12)

    # combine the objects into a single boolean array
    voxels = cube1 | cube2 | link

    # set the colors of each object
    colors = np.empty(voxels.shape, dtype=object)
    colors[link] = 'blue'
    colors[cube1] = 'blue'
    colors[cube2] = 'blue'

    # and plot everything
    # fig = plt.figure()
    # ax = fig.gca(projection='3d')
    # ax.voxels(voxels, facecolors=colors, edgecolor='k')

    # plt.show()
    
    samples = vox_sample(n_samples=20, voxmap=voxels)
    for vox in samples:
        voxels[vox] = True

        colors[vox] = 'red'

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.voxels(voxels, facecolors=colors, edgecolor='k')

    plt.show()
