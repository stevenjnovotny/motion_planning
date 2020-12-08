import numpy as np

def create_voxmap(data, voxel_size=5):
    """
    Returns a grid representation of a 3D configuration space
    based on given obstacle data.
    
    The `voxel_size` argument sets the resolution of the voxel map. 
    """
    # minimum and maximum north coordinates
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

    # maximum altitude
    alt_max = np.ceil(np.amax(data[:, 2] + data[:, 5]))
    
    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min))) // voxel_size
    east_size = int(np.ceil((east_max - east_min))) // voxel_size
    alt_size = int(alt_max) // voxel_size

    # Create an empty grid
    voxmap = np.zeros((north_size, east_size, alt_size), dtype=np.bool)

    for i in range(data.shape[0]):
        # TODO: fill in the voxels that are part of an obstacle with `True`
        #
        # i.e. grid[0:5, 20:26, 2:7] = True
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        obstacle = [
            int(north - d_north - north_min) // voxel_size,
            int(north + d_north - north_min) // voxel_size,
            int(east - d_east - east_min) // voxel_size,
            int(east + d_east - east_min) // voxel_size,
            # np.ceil((north - d_north - north_min) / voxel_size),
            # np.ceil((north + d_north - north_min) / voxel_size),
            # np.ceil((east - d_east - east_min) / voxel_size),
            # np.ceil((east + d_east - east_min) / voxel_size),
        ]

        height = int(alt + d_alt) // voxel_size
        voxmap[obstacle[0]:obstacle[1], obstacle[2]:obstacle[3], 0:height] = True

    return voxmap

def create_voxmap_around_point(data, pt, region_size=40, voxel_size=5, alt_max = None):
    """
    Returns a grid representation of a 3D configuration space
    based on given obstacle data.
    
    The `voxel_size` argument sets the resolution of the voxel map. 
    """

    print('running voxmap around point')
    # minimum and maximum north coordinates
    data_x_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    data_x_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))

    x_min = pt[1] - region_size//2 + data_x_min
    x_max = pt[1] + region_size//2 + data_x_min

    # minimum and maximum east coordinates
    data_y_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    data_y_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

    y_min = pt[0] - region_size//2 + data_y_min
    y_max = pt[0] + region_size//2 + data_y_min

    #print(x_min, x_max, y_min, y_max)

    # maximum altitude
    if alt_max == None:
        alt_max = np.ceil(np.amax(data[:, 2] + data[:, 5]))
    
    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    x_size = int(np.ceil((x_max - x_min))) // voxel_size
    y_size = int(np.ceil((y_max - y_min))) // voxel_size
    alt_size = int(alt_max) // voxel_size

    # Create an empty grid
    voxmap = np.zeros((x_size, y_size, alt_size), dtype=np.bool)


    for i in range(data.shape[0]):
        # fill in the voxels that are part of an obstacle with `True`
        #
        # i.e. grid[0:5, 20:26, 2:7] = True
        x, y, alt, d_x, d_y, d_alt = data[i, :]
        
        # obstacle = [
        #     int(x - d_x - data_x_min) // voxel_size,
        #     int(x + d_x - data_x_min) // voxel_size,
        #     int(y - d_y - data_y_min) // voxel_size,
        #     int(y + d_y - data_y_min) // voxel_size,
        # ]

        obstacle = [
            x - d_x,
            x + d_x,
            y - d_y,
            y + d_y,
        ]
        #print(obstacle)
        #if (obstacle[1] >= x_min and x_max >= obstacle[0])  and  (obstacle[3] >= y_min and y_max >= obstacle[2]):
        if (obstacle[1] >= x_min and x_max >= obstacle[0])  and  (obstacle[3] >= y_min and y_max >= obstacle[2]):
            #print('hit')
            #print(obstacle)
            vox_stacle = [
                max(int(x - d_x - x_min) // voxel_size, 0),
                min(int(x + d_x - x_min) // voxel_size, region_size),
                max(int(y - d_y - y_min) // voxel_size, 0),
                min(int(y + d_y - y_min) // voxel_size, region_size)
            ]
            #print(vox_stacle)
            height = int(alt + d_alt) // voxel_size
            voxmap[vox_stacle[0]:vox_stacle[1], vox_stacle[2]:vox_stacle[3], 0:height] = True

    return voxmap