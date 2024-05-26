import osmnx as ox
import geopandas as gpd
import shapely.geometry as geometry
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon, MultiLineString, MultiPolygon, LineString
from shapely.strtree import STRtree
import os
import numpy as np
import itertools
import networkx as nx
import math
import pandas as pd
import csv
from pyproj import Proj, transform, Transformer
from scipy.optimize import minimize
import ast
from scipy.interpolate import Akima1DInterpolator

# Define the projection for latitude and longitude (WGS84)
wgs84 = Proj(init='epsg:4326')

# Define the local projection (UTM zone 31U)
local_projection = Proj(init='epsg:32631') # UTM zone 31U

class Map:
    def __init__(self, position, osm_file):
        # Position should be an (x, y) tuple
        # osm_file should be an OSM file path

        x, y = transform(wgs84, local_projection, position[0], position[1])
        self.position = Point((x,y))  # Initial position [x, y]

        # Read the OSM file and create a graph
        G = ox.graph_from_xml(osm_file)
        nodes, edges = ox.graph_to_gdfs(G, fill_edge_geometry=True)
        self.graph = ox.graph_from_gdfs(nodes, edges, graph_attrs=G.graph)

        # Extract the lines from the graph as a GeoDataFrame
        self.lines = edges

        # Preprocess the edges GeoDataFrame
        self.preprocess_edges()

    def preprocess_edges(self):
        # order linestrings suc

        line_strings = self.lines['geometry'].values
        self.line_strings = line_strings
        lines = {}
        # Concatenate all LineString geometries into a single LineString
        concatenated_line_string = LineString()
        for i, line_string in enumerate(line_strings):
            if i+1 in [6,8,11, 12, 14]:
                x, y = line_string.xy
                # plt.plot(x, y, label=f'Line {i+1}')
                lines[f"{i+1}"] = list(line_string.coords)
        concatenated_line_string = [
            list(reversed(list(lines["11"])))+
            list(reversed(list(lines["12"]))) +
            list((lines["14"])) +
            list(reversed(list(lines["6"]))) +
            list(lines["8"]) +
            list(lines["6"])
        ]

        polypoints = []
        for tuple in concatenated_line_string:
            for point in tuple:
                # Transform coordinates from WGS84 to local meters
                x, y = transform(wgs84, local_projection, point[0], point[1])
                polypoints.append((x, y))

        # Convert the concatenated LineString to a Polygon
        self.polygon = Polygon(polypoints)


        file_path = os.getcwd() + "/control/data/polygon.csv"

        # Write the coordinates to the CSV file
        with open(file_path, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["X-axis", "Y-axis"])  # Write header
            writer.writerows(polypoints)  # Write path coordinates

        print(f"Polygon saved to {file_path}")
        # plt.legend()
        # plt.legend()  # Add legend
        # plt.show()

        self.polygon_lines = polypoints
    

    def update_location(self, position):
        if self.check_point_in_polygon(position):
            self.position = Point(position)  # Update position
        else:
            raise ValueError("New location not in polygon")

    def check_point_in_polygon(self, position):
        # # Create a point from the given position
        point = Point(position)

        return self.polygon.contains(point)

    def plot_polygon(self):
        # Plot the polygon
        # Plot the polygon
        x, y = self.polygon.exterior.xy
        plt.plot(x, y)
        plt.show()

    def plot_location(self, point=None):
        # Plot the polygon and current location
        fig, ax = plt.subplots()
        x, y = self.polygon.exterior.xy
        plt.plot(x, y)
        if point:
            ax.scatter(point[0], point[1], color='red', label='Current Location')    
        else:
            ax.scatter(self.position.x, self.position.y, color='red', label='Current Location')
        plt.title('Map with Current Location')
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.legend()
        plt.show()

    def discretize_polygon(self, step_size=1):
        min_x, min_y, max_x, max_y = self.polygon.bounds
        x_coords = np.arange(min_x, max_x, step_size)
        y_coords = np.arange(min_y, max_y, step_size)
        points = [Point(x, y) for x, y in itertools.product(x_coords, y_coords)]
        self.grid = [point for point in points if self.polygon.contains(point)]

    def plot_discretized_points(self):
        fig, ax = plt.subplots()
        x, y = self.polygon.exterior.xy
        plt.plot(x, y, label='Polygon')
        
        points_coords = np.array([(point.x, point.y) for point in self.grid])
        plt.scatter(points_coords[:, 0], points_coords[:, 1], color='blue', s=1, label='Discretized Points')

        plt.title('Discretized Points within Polygon')
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.legend()
        plt.show()

    # def check_steering_angle(self, p1, p2, max_angle=35):
    #     dx = p2.x - p1.x
    #     dy = p2.y - p1.y
    #     angle = np.arctan2(dy, dx) * 180 / np.pi
    #     return abs(angle) <= max_angle

    def euclidean_distance(self, x1, y1, x2, y2):
        """
        Calculate the Euclidean distance between two points
        """
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def create_edges(self):
        self.edges = []
        for p1, p2 in itertools.combinations(self.grid, 2):
            edge = LineString([p1, p2])
            # intersects_list = [edge.intersects(LineString(line)) for line in self.polygon_lines]
            # print(intersects_list)  # Print the actual boolean values
            # intersects = any(intersects_list)

            if not edge.intersects(LineString(self.polygon_lines)):
                self.edges.append((p1, p2))

    def build_graph_from_points(self):
        self.G = nx.Graph()
        for point in self.grid:
            self.G.add_node((point.x, point.y))
        for edge in self.edges:
            p1, p2 = edge
            self.G.add_edge((p1.x, p1.y), (p2.x, p2.y), weight = self.euclidean_distance(p1.x, p1.y,p2.x, p2.y))

        # Define the file path
        file_path = os.getcwd() + "/control/data/weighted_graph.graphml"

        # Write the graph to an edge list file
        nx.write_graphml(self.G, file_path)


    def closest_node(self, x, y, nodes):
        """
        Find the closest node to a given latitude and longitude point
        from a list of nodes.
        """
        closest_node = None
        min_distance = float('inf')

        for node in nodes:
            distance = self.euclidean_distance(x, y, node[0], node[1])
            if distance < min_distance:
                min_distance = distance
                closest_node = node

        return closest_node

    # def optimal_path(self, current_location, goal_location):
        
    #     pass

    def reduce_graph(self):
        """
        Ensure that the path cannot be too close to the polygon boundary
        """
        wrong_nodes = []
        wrong_edges = []
        for node in self.G.nodes:
            point = Point((node[0],node[1]))
            distance = point.distance(LineString(self.polygon_lines))
            if distance < self.margin:
                wrong_nodes.append(node)

        for edge in self.G.edges:
            line = LineString([edge[0], edge[1]])
            distance = line.distance(LineString(self.polygon_lines))
            if distance < self.margin:
                wrong_edges.append(edge)

        for edge in wrong_edges:
            G.remove_edge(*edge)

        for node in wrong_nodes:
            G.remove_node(node)

    def find_shortest_path(self, start, end, G = None, margin= None):
        if not G:
            self.discretize_polygon()
            self.create_edges()
            self.build_graph_from_points()

        self.G = G

        if margin:
            self.margin = margin
            self.reduce_graph()


        x, y = transform(wgs84, local_projection, start[0], start[1])
        start = [x,y]
        x, y = transform(wgs84, local_projection, end[0], end[1])
        end = [x,y]

        start_node = self.closest_node(start[0], start[1], self.G.nodes)
        end_node = self.closest_node(end[0], end[1],self.G.nodes)

        shortest_path = nx.shortest_path(self.G, start_node, end_node,weight='weight')
        
        path_coords = [(x, y) for x, y in shortest_path]
        return path_coords

    def find_all_paths(self, start, end, G = None):
        if not G:
            self.discretize_polygon()
            self.create_edges()
            self.build_graph_from_points()

        self.G = G

        x, y = transform(wgs84, local_projection, start[0], start[1])
        start = [x,y]
        x, y = transform(wgs84, local_projection, end[0], end[1])
        end = [x,y]

        start_node = self.closest_node(start[0], start[1], self.G.nodes)
        end_node = self.closest_node(end[0], end[1],self.G.nodes)

        return list(nx.all_simple_paths(self.G, start_node, end_node))
    


    def plot_path(self, path_coords):
        fig, ax = plt.subplots()
        x, y = self.polygon.exterior.xy
        plt.plot(x, y, label='Polygon')
        
        path_coords = np.array(path_coords)
        plt.plot(path_coords[:, 0], path_coords[:, 1], color='blue', linewidth=2, label='Shortest Path')
        
        plt.scatter(path_coords[0, 0], path_coords[0, 1], color='green', label='Start Point')
        plt.scatter(path_coords[-1, 0], path_coords[-1, 1], color='red', label='End Point')

        plt.title('Shortest Path Inside Polygon')
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.legend()
        plt.show()

    def smooth_trajectory(self, path_coords, num_points=100):
        path_coords = np.array(path_coords)
        x = path_coords[:, 0]
        y = path_coords[:, 1]
        
        # Create a parameter for the points
        t = np.linspace(0, 1, len(x))
        
        # Interpolating with Akima1DInterpolator
        interp_x = Akima1DInterpolator(t, x)
        interp_y = Akima1DInterpolator(t, y)
        
        # Generate new points
        t_new = np.linspace(0, 1, num_points)
        x_new = interp_x(t_new)
        y_new = interp_y(t_new)
        
        smooth_path_coords = list(zip(x_new, y_new))

        # Calculate derivatives
        dx_dt = np.gradient(x_new, t_new)
        dy_dt = np.gradient(y_new, t_new)

        # Calculate yaw angles
        yaw_angles = np.arctan2(dy_dt, dx_dt)

        return smooth_path_coords, yaw_angles

# Path to the OSM file
osm_file_path = os.path.join(os.getcwd(), 'control', 'lanelet2_map_versimpeld.osm')

point = (5.513063, 52.459907)

map_instance = Map(point, osm_file_path)
# map_instance.plot_location()
# map_instance.discretize_polygon()
# map_instance.plot_discretized_points()


# print(map.check_point_in_polygon(point))
# point2 = (5.511368, 52.458993)
# print(map.check_point_in_polygon(point2))
# map.plot_location(point2)
# point3 = (5.511435, 52.459028)
# print(map.check_point_in_polygon(point3))
# map.plot_location(point3)

# Define start and end positions
start_position = (5.513063, 52.459907)
end_position = (5.511204, 52.458896)

new_graph = False
new_shortest_path = True

if new_graph:
    path_coords = map_instance.find_shortest_path(start_position, end_position)
else: 
    # Read the weighted graph from the file
    file_path = os.getcwd() + "/control/data/weighted_graph.graphml"
    # Read the graph from the GraphML file
    G_loaded = nx.read_graphml(file_path)

    # Convert node labels back to tuples
    G_converted = nx.Graph()
    for node in G_loaded.nodes:
        # Convert string representation of tuple back to tuple
        tuple_node = tuple(map(float, node.strip('()').split(',')))
        G_converted.add_node(tuple_node)
        # Copy over edges
        for neighbor in G_loaded.neighbors(node):
            tuple_neighbor = tuple(map(float, neighbor.strip('()').split(',')))
            weight = G_loaded[node][neighbor]['weight']
            G_converted.add_edge(tuple_node, tuple_neighbor, weight=weight)

    G = G_converted

    path_coords = map_instance.find_shortest_path(start_position, end_position, G, margin = 1)
    # else:
    #     # Define the file path
    #     file_path = os.getcwd() + "/control/data/waypoints.csv"
    #     with open(file_path, newline='') as f:
    #         rows = list(csv.reader(f, delimiter=','))

    #     # Assuming 'rows' is defined somewhere above this code
    #     path_coords = np.array(rows[1:])
    #     print(path_coords)
            
# Define the file path
file_path = os.getcwd() + "/control/data/waypoints.csv"

# Write the coordinates to the CSV file
with open(file_path, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["X-axis", "Y-axis"])  # Write header
    writer.writerows(path_coords)  # Write path coordinates

print(f"Path coordinates saved to {file_path}")


# Plot the shortest path
map_instance.plot_path(path_coords)

# # Smooth the trajectory
smooth_path_coords, yaw_angles = map_instance.smooth_trajectory(path_coords)
# Define the file path
file_path = os.getcwd() + "/control/data/yaw_angles.csv"

# Write the yaw angles to the CSV file
with open(file_path, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["Yaw_angle"])  # Write header
    writer.writerows([[angle] for angle in yaw_angles])  # Write yaw angles

print(f"Yaw angles saved to {file_path}")
# # Plot the smoothed trajectory
map_instance.plot_path(smooth_path_coords)



