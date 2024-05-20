import osmnx as ox
import geopandas as gpd
import shapely.geometry as geometry
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon, MultiLineString, MultiPolygon, LineString
import os
import numpy as np
import itertools
import networkx as nx
import math

class Map:
    def __init__(self, position, osm_file):
        # Position should be an (x, y) tuple
        # osm_file should be an OSM file path

        self.position = Point(position)  # Initial position [x, y]

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
                polypoints.append([point[0],point[1]])

        # Convert the concatenated LineString to a Polygon
        self.polygon = Polygon(polypoints)
        # plt.legend()
        # plt.legend()  # Add legend
        # plt.show()
    

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

    def discretize_polygon(self, step_size=0.00001):
        min_x, min_y, max_x, max_y = self.polygon.bounds
        x_coords = np.arange(min_x, max_x, step_size)
        y_coords = np.arange(min_y, max_y, step_size)
        self.grid = [Point(x, y) for x, y in itertools.product(x_coords, y_coords) if self.polygon.contains(Point(x, y))]

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

    def check_steering_angle(self, p1, p2, max_angle=35):
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        angle = np.arctan2(dy, dx) * 180 / np.pi
        return abs(angle) <= max_angle

    def create_edges(self, max_angle=45):
        self.edges = []
        for p1, p2 in itertools.combinations(self.grid, 2):
            if self.check_steering_angle(p1, p2, max_angle):
                self.edges.append((p1, p2))

    def build_graph_from_points(self):
        self.G = nx.Graph()
        for point in self.grid:
            self.G.add_node((point.x, point.y))
        for edge in self.edges:
            p1, p2 = edge
            self.G.add_edge((p1.x, p1.y), (p2.x, p2.y))

    def euclidean_distance(self, x1, y1, x2, y2):
        """
        Calculate the Euclidean distance between two points
        """
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def closest_node(self, lat, lon, nodes):
        """
        Find the closest node to a given latitude and longitude point
        from a list of nodes.
        """
        closest_node = None
        min_distance = float('inf')

        for node in nodes:
            distance = self.euclidean_distance(lon, lat, node[0], node[1])
            if distance < min_distance:
                min_distance = distance
                closest_node = node

        return closest_node

    def find_shortest_path(self, start, end):
        self.discretize_polygon()
        self.create_edges()
        self.build_graph_from_points()

        start_node = self.closest_node(start[0], start[1], self.G.nodes)
        end_node = self.closest_node(end[0], end[1],self.G.nodes)
        print(start_node,end_node)

        shortest_path = nx.shortest_path(self.G, start_node, end_node)

        path_coords = [(x, y) for x, y in shortest_path]
        return path_coords

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
        t = np.linspace(0, 1, len(x))
        
        # Interpolation
        x_interp = np.interp(np.linspace(0, 1, num_points), t, x)
        y_interp = np.interp(np.linspace(0, 1, num_points), t, y)
        
        smooth_path_coords = list(zip(x_interp, y_interp))
        return smooth_path_coords

# Path to the OSM file
osm_file_path = os.path.join(os.getcwd(), 'control', 'lanelet2_map_versimpeld.osm')

point = (5.513063, 52.459907)

map_instance = Map(point, osm_file_path)
# map_instance.plot_location()
map_instance.discretize_polygon()
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

# if map_instance.check_point_in_polygon(end_position):

# Find the shortest path
path_coords = map_instance.find_shortest_path(start_position, end_position)

# Plot the shortest path
map_instance.plot_path(path_coords)

# Smooth the trajectory
smooth_path_coords = map_instance.smooth_trajectory(path_coords)

# Plot the smoothed trajectory
map_instance.plot_path(smooth_path_coords)



