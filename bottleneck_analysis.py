from config_to_graph import graph_from_config
import robotic as ry
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict, Counter
from queue import PriorityQueue

def get_target_vertices(graph, obj_vertex, radius, step_size):
    """
    Identify vertices within a given radius from the object vertex.
    The radius is measured in actual distance, not grid indices.
    """
    targets = []
    obj_position = np.array(obj_vertex) * step_size
    for vertex in graph.keys():
        vertex_position = np.array(vertex) * step_size
        distance = np.linalg.norm(vertex_position - obj_position)
        if distance <= radius:
            targets.append(vertex)
    return targets

def shortest_path(graph, start, end):
    """
    Find the shortest path between two vertices using Dijkstra's algorithm.
    Converts vertices to tuples for hashing.
    """
    start = tuple(start)
    end = tuple(end)
    pq = PriorityQueue()
    pq.put((0, start))
    distances = {start: 0}
    predecessors = {start: None}

    while not pq.empty():
        current_distance, current_vertex = pq.get()

        if current_vertex == end:
            # Reconstruct path
            path = []
            while current_vertex is not None:
                path.append(current_vertex)
                current_vertex = predecessors[current_vertex]
            return path[::-1]  # Reverse the path

        for neighbor, weight in graph[current_vertex]:
            neighbor = tuple(neighbor)
            new_distance = current_distance + weight
            if neighbor not in distances or new_distance < distances[neighbor]:
                distances[neighbor] = new_distance
                predecessors[neighbor] = current_vertex
                pq.put((new_distance, neighbor))
    return None  # No path found

def find_bottleneck_vertices(paths, bottleneck_threshold):
    """
    Identify vertices that are bottleneck vertices based on the given paths.
    """
    all_vertices = [vertex for path in paths for vertex in path]
    vertex_counts = Counter(all_vertices)
    max_usage = max(vertex_counts.values())
    
    bottleneck_vertices = [vertex for vertex, count in vertex_counts.items() if count > max_usage * bottleneck_threshold]
    return bottleneck_vertices

def bottleneck_vertices_from_config(config, radius, step_size, bottleneck_threshold=0.5):
    """
    Perform bottleneck analysis on a given configuration.
    """
    graph, obj_vertex = graph_from_config(config, step_size=step_size)
    target_vertices = get_target_vertices(graph, obj_vertex, radius, step_size)
    paths = []
    for target in target_vertices:
        path = shortest_path(graph, obj_vertex, target)
        if path:
            paths.append(path)
    return find_bottleneck_vertices(paths, bottleneck_threshold)

if __name__ == "__main__":
    # Define parameters
    radius = 3.0  # Radius in actual distance
    step_size = 0.05  # Same as the step_size used in the grid generation
    bottleneck_threshold = 0.25  # Bottleneck threshold as a fraction of the maximum usage
    vertex_count = 200  # Number of bottleneck vertices to identify

    # Load the configuration and convert it to a graph
    config = ry.Config()
    config.addFile("puzzles/p3-maze.g")
    # config.getFrame("obj").setPosition([0.2, 0.5, 0.2]) # manual override for p3-maze
    graph, obj_vertex = graph_from_config(config, step_size)

    # Perform bottleneck analysis
    bottleneck_vertices = bottleneck_vertices_from_config(config, radius, step_size, bottleneck_threshold=bottleneck_threshold)

    # Visualize the graph and bottleneck vertices
    fig, ax = plt.subplots()
    for node, neighbors in graph.items():
        for neighbor, weight in neighbors:
            ax.plot([node[1], neighbor[1]], [node[0], neighbor[0]], 'k-')

    # Highlight bottleneck vertices
    for bottleneck in bottleneck_vertices:
        ax.plot(bottleneck[1], bottleneck[0], 'ro')  # Red circles for bottlenecks

    # Highlight object vertex
    ax.plot(obj_vertex[1], obj_vertex[0], 'bo')  # Blue circle for the object vertex

    ax.set_aspect('equal')
    ax.invert_yaxis()
    plt.show()

    del config
