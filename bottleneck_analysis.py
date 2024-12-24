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

        for neighbor in graph[current_vertex]:
            neighbor = tuple(neighbor)
            new_distance = current_distance + 1  # Each edge has a weight of 1
            if neighbor not in distances or new_distance < distances[neighbor]:
                distances[neighbor] = new_distance
                predecessors[neighbor] = current_vertex
                pq.put((new_distance, neighbor))
    return None  # No path found

def find_bottleneck_vertices(paths):
    """
    Detect bottleneck vertices based on their frequency of occurrence in paths.
    """
    all_vertices = [vertex for path in paths for vertex in path]
    vertex_counts = Counter(all_vertices)
    max_usage = max(vertex_counts.values())
    
    # Define a bottleneck as vertices used more than 50% of the max usage
    bottleneck_vertices = [vertex for vertex, count in vertex_counts.items() if count > max_usage * 0.5]
    return bottleneck_vertices

if __name__ == "__main__":
    # Load the configuration and convert it to a graph
    config = ry.Config()
    config.addFile("puzzles/p6-wall.g")
    graph, obj_vertex = graph_from_config(config)

    # Define parameters
    radius = 2.0  # Radius in actual distance
    step_size = 0.1  # Same as the STEP_SIZE used in the grid generation

    # Step 1: Obtain target vertices
    target_vertices = get_target_vertices(graph, obj_vertex, radius, step_size)
    print(f"Target vertices within radius {radius}: {target_vertices}")

    # Step 2: Find shortest paths to each target vertex
    paths = []
    for target in target_vertices:
        path = shortest_path(graph, obj_vertex, target)
        if path:
            paths.append(path)

    # Step 3: Detect bottleneck vertices
    bottleneck_vertices = find_bottleneck_vertices(paths)
    print(f"Bottleneck vertices: {bottleneck_vertices}")

    # Visualize the graph and bottleneck vertices
    fig, ax = plt.subplots()
    for node, neighbors in graph.items():
        for neighbor in neighbors:
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
