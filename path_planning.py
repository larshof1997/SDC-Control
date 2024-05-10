import os
import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np


def build_graph_from_folder(folder_path):
    G = nx.MultiDiGraph()
    
    # Iterate through files in the folder
    for filename in os.listdir(folder_path):
        if filename.endswith(".osm"):  # Check if it's an OSM file
            file_path = os.path.join(folder_path, filename)
            try:
                # Load the OSM file and construct a graph
                graph = ox.graph_from_xml(file_path, simplify=True)
                # Add nodes and edges from the graph to the main graph
                G.add_nodes_from(graph.nodes(data=True))
                G.add_edges_from(graph.edges(keys=True, data=True))
            except Exception as e:
                print(f"Error processing file {file_path}: {e}")
    
    return G

folder_path = os.getcwd() + '/test_track_lelystad'
graph = build_graph_from_folder(folder_path)

if "crs" not in graph.graph:
    graph.graph["crs"] = "EPSG:4326"  # WGS84 coordinate system

# Manually compute the bounding box of the graph
node_positions = [(data['x'], data['y']) for node, data in graph.nodes(data=True)]
print(node_positions)
x_min, y_min = np.min(node_positions, axis=0)
x_max, y_max = np.max(node_positions, axis=0)




