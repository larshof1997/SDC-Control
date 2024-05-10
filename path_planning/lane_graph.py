import osmnx as ox
import matplotlib.pyplot as plt
import networkx as nx
import sklearn

# Path to the OSM file
osm_file_path = '/Users/lars/Documents/GitHub/SDC-Control/path_planning/map.osm'

# Read the OSM file and create a graph
G = ox.graph_from_xml(osm_file_path, simplify=False)

# Convert the graph to GeoDataFrames (optional)
nodes = ox.graph_to_gdfs(G, edges = False)
print(nodes)

# Visualize the graph (optional)
ox.plot_graph(G)

# # Specify start and end points (lat, lon coordinates)
# start_point = (5.51309, 52.45990)
# end_point = (5.51085, 52.45873)

# # Use A* algorithm to calculate the shortest path
# shortest_path = nx.astar_path(G, ox.distance.nearest_nodes(G, start_point[0], start_point[1]),
#                               ox.distance.nearest_nodes(G, end_point[0], end_point[1]),
#                               weight='length')

# # Visualize the graph and the shortest path
# fig, ax = ox.plot_graph_route(G, shortest_path, node_size=0, bgcolor='k', route_color='r')

# # Show the plot
# plt.show()