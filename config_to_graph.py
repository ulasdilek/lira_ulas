import robotic as ry
import numpy as np  
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from collections import defaultdict
from math import sqrt

def get_obj_vertex(config : ry.Config, step_size : float) -> np.ndarray:
    OBJ_NAME = "obj"
    grid_offset = np.array([2/step_size, 2/step_size]).astype(int)
    obj_vertex = (config.getFrame(OBJ_NAME).getPosition()[:2]/step_size).astype(int) + grid_offset
    return obj_vertex

def graph_from_config(config : ry.Config, step_size : float) -> dict:
    wall_names = []
    OBJ_NAME = "obj"
    for entity in config.getFrameNames():
        if "wall" in entity.lower():
            wall_names.append(entity)
        
    obj_size = config.getFrame(OBJ_NAME).getSize()
    obj_width = obj_size[0]
    obj_height = obj_size[1]
    size_offset = np.array([obj_width, obj_height])
    grid_offset = np.array([2/step_size, 2/step_size]).astype(int)

    occupancy_grid = np.ones((int(4/step_size), int(4/step_size)))
    for wall in wall_names:
        # print(f"Adding: {wall}")
        wall_frame = config.getFrame(wall)
        wall_size = wall_frame.getSize()[:2]
        wall_size += size_offset
        wall_position = wall_frame.getPosition()[:2]
        wall_anchor = wall_position - wall_size/2.0
        wall_end = wall_position + wall_size/2.0
        wall_indices = np.array([wall_anchor, wall_end])/step_size + grid_offset
        wall_indices = wall_indices.astype(int)
        wall_indices = np.clip(wall_indices, 0, occupancy_grid.shape)
        # print(wall_indices)
        occupancy_grid[wall_indices[0, 0]:wall_indices[1, 0], wall_indices[0, 1]:wall_indices[1, 1]] = 0

    obj_vertex = get_obj_vertex(config, step_size)

    return graph_from_grid(occupancy_grid), obj_vertex

def graph_from_grid(grid: np.ndarray) -> dict:
    """
    Converts a binary grid into a weighted graph representation using an adjacency list.
    An edge exists between two vertices if and only if both are 1.

    Args:
        grid (np.ndarray): A binary grid where each element is either 0 or 1.

    Returns:
        defaultdict: An adjacency list representation of the weighted graph.
    """
    rows, cols = grid.shape
    graph = defaultdict(list)

    for i in range(rows):
        for j in range(cols):
            if grid[i][j] == 1:
                for dx, dy, weight in [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1), (-1, -1, sqrt(2)), (-1, 1, sqrt(2)), (1, -1, sqrt(2)), (1, 1, sqrt(2))]:
                    ni, nj = i + dx, j + dy
                    if 0 <= ni < rows and 0 <= nj < cols and grid[ni][nj] == 1:
                        graph[(i, j)].append(((ni, nj), weight))

    return graph

if __name__ == "__main__":
    config = ry.Config()
    config.addFile("puzzles/p7-o-room.g")
    graph, obj_vertex = graph_from_config(config, 0.1)

    # Visualize the graph using matplotlib
    fig, ax = plt.subplots()
    for node, neighbors in graph.items():
        for neighbor, weight in neighbors:
            ax.plot([node[1], neighbor[1]], [node[0], neighbor[0]], 'k-')
    ax.set_aspect('equal')
    ax.invert_yaxis()
    plt.show()
    
    del config
