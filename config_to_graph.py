import robotic as ry
import numpy as np  
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from collections import defaultdict

def graph_from_config(config : ry.Config) -> dict:
    wall_names = []
    OBJ_NAME = "obj"
    for entity in config.getFrameNames():
        if "wall" in entity.lower():
            wall_names.append(entity)
        
    STEP_SIZE = 0.1
    obj_size = config.getFrame(OBJ_NAME).getSize()
    obj_width = obj_size[0]
    obj_height = obj_size[1]
    size_offset = np.array([obj_width, obj_height])
    grid_offset = np.array([2/STEP_SIZE, 2/STEP_SIZE]).astype(int)

    # grid_config = ry.Config()
    occupancy_grid = np.ones((int(4/STEP_SIZE), int(4/STEP_SIZE)))
    for wall in wall_names:
        print(f"Adding: {wall}")
        wall_frame = config.getFrame(wall)
        wall_size = wall_frame.getSize()[:2]
        wall_size += size_offset
        wall_position = wall_frame.getPosition()[:2]
        wall_anchor = wall_position - wall_size/2.0
        wall_end = wall_position + wall_size/2.0
        # get the indices over which the wall spans
        wall_indices = np.array([wall_anchor, wall_end])/STEP_SIZE + grid_offset
        wall_indices = wall_indices.astype(int)
        # cutoff indices to fit within the grid
        wall_indices = np.clip(wall_indices, 0, occupancy_grid.shape)
        print(wall_indices)
        occupancy_grid[wall_indices[0, 0]:wall_indices[1, 0], wall_indices[0, 1]:wall_indices[1, 1]] = 0

        # grid_config.addFrame(wall) \
        #             .setShape(ry.ST.box, size=(wall_size.tolist() + [0.2])) \
        #             .setPosition(wall_position.tolist() + [0.1]) \
        #             .setColor([1, 1, 0])

    # grid_config.addFrame(OBJ_NAME) \
    #             .setShape(ry.ST.box, size=(obj_size.tolist() + [0.2])) \
    #             .setPosition(config.getFrame(OBJ_NAME).getPosition()) \
    #             .setColor([0, 0, 1])


    # config.view_setCamera(config.getFrame("camera_top"))
    # config.view()
    # grid_config.view()
    # show occupancy grid as image
    # plt.imshow(occupancy_grid, cmap='binary_r')
    # plt.show()

    # grid_config.view_close
    # del grid_config

    obj_vertex = (config.getFrame(OBJ_NAME).getPosition()[:2]/STEP_SIZE).astype(int) + grid_offset

    return graph_from_grid(occupancy_grid), obj_vertex

def graph_from_grid(grid: np.ndarray) -> dict:
    """
    Converts a binary grid into a graph representation using an adjacency list.
    An edge exists between two orthogonal vertices if and only if both are 1.

    Args:
        grid (np.ndarray): A binary grid where each element is either 0 or 1.

    Returns:
        defaultdict: An adjacency list representation of the graph.
    """
    rows, cols = grid.shape
    graph = defaultdict(list)

    for i in range(rows):
        for j in range(cols):
            if grid[i][j] == 1:
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    ni, nj = i + dx, j + dy
                    if 0 <= ni < rows and 0 <= nj < cols and grid[ni][nj] == 1:
                        graph[(i, j)].append((ni, nj))

    return graph

if __name__ == "__main__":
    config = ry.Config()
    config.addFile("puzzles/p6-wall.g")
    graph, obj_vertex = graph_from_config(config)

    # Visualize the graph using matplotlib
    fig, ax = plt.subplots()
    for node, neighbors in graph.items():
        for neighbor in neighbors:
            ax.plot([node[1], neighbor[1]], [node[0], neighbor[0]], 'k-')
    ax.set_aspect('equal')
    ax.invert_yaxis()
    plt.show()
    
    del config
