import osmnx as ox
import geopandas as gpd
import shapely.geometry as geometry
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon, MultiLineString, MultiPolygon
import os

from lanelet2_parser import Lanelet2Parser

# Step 1: Creating Lanelet2Parser instance
lanelet2_parser = Lanelet2Parser()

# Step 2: Parse OSM File
osm_map_file_path = "your_osm_file.osm"  # Replace with your OSM file path
osm_data = lanelet2_parser.parse(osm_map_file_path)

# Step 3: Convert Lanelet2 format from OSM
lanelet2_data = lanelet2_parser.convert_to_lanelet2(osm_data)

# Step 4: Create waypoints or trajectory (example)
# For demonstration, let's say we want to create a trajectory from point A to point B
# You need to define your start and end points in terms of coordinates (lat, lon)
start_point = (start_lat, start_lon)  # Replace with your start point coordinates
end_point = (end_lat, end_lon)  # Replace with your end point coordinates

# Now, you need to find the lanelets or line strings that cover these points
# You can iterate over lanelets or line strings and check if your points fall within them

# Example of finding lanelet or line string containing the start and end points
start_lanelet = None
end_lanelet = None

for lanelet in lanelet2_data.lanelets:
    if point_inside_lanelet(start_point, lanelet):
        start_lanelet = lanelet
    if point_inside_lanelet(end_point, lanelet):
        end_lanelet = lanelet

# Now that you have start and end lanelets, you can create a trajectory between them
# You can use various algorithms such as A* or Dijkstra's to find the shortest path

# Example of creating a trajectory
trajectory = find_shortest_path(start_lanelet, end_lanelet)

# You can then use this trajectory for navigation or any other purpose


# class Map:
#     def __init__(self, position, osm_file):
#         # Position should be an (x, y) tuple
#         # osm_file should be an OSM file path

#         self.position = Point(position)  # Initial position [x, y]

#         # Read the OSM file and create a graph
#         # self.graph = ox.graph_from_xml(osm_file)
#         self.features = ox.features_from_xml(osm_file)
#         print(self.features)
#         print(self.features['type'].values)
#         self.lines = self.features[self.features['type']=='line_thin']
        
#         # print(lines['geometry'])

#         # # Extract the edges (roads) from the graph as a GeoDataFrame
#         # self.edges_gdf = ox.graph_to_gdfs(self.graph, nodes=False, edges=True)

#         # Preprocess the edges GeoDataFrame
#         self.preprocess_edges()

#     def preprocess_edges(self):
#         # # Iterate through line strings and add all coordinates to the list
#         line_strings = self.lines['geometry'].values

#        # Concatenate all LineString geometries into a single LineString
#         concatenated_line_string = geometry.LineString()
#         for line_string in line_strings:
#             x,y = line_string.xy
#             plt.plot(x,y)
#             concatenated_line_string = geometry.LineString(
#                 list(concatenated_line_string.coords) + list(line_string.coords)
#             )
#         plt.show()
#         # # Create a polygon from all coordinates
#         self.polygon = Polygon(concatenated_line_string)
        


#     def update_location(self, position):
#         if self.check_point_in_polygon(position):
#             self.position = Point(position)  # Update position
#         else:
#             raise ValueError("New location not in polygon")

#     def check_point_in_polygon(self, position):
#         # # Create a point from the given position
#         point = Point(position)

#         return self.polygon.contains(point)

#     def plot_polygon(self):
#         # Plot the polygon
#         # Plot the polygon
#         x, y = self.polygon.exterior.xy
#         plt.plot(x, y)
#         plt.show()

#     def plot_location(self, point=None):
#         # Plot the polygon and current location
#         fig, ax = plt.subplots()
#         x, y = self.polygon.exterior.xy
#         plt.plot(x, y)
#         if point:
#             ax.scatter(point[0], point[1], color='red', label='Current Location')    
#         else:
#             ax.scatter(self.position.x, self.position.y, color='red', label='Current Location')
#         plt.title('Map with Current Location')
#         plt.xlabel('Longitude')
#         plt.ylabel('Latitude')
#         plt.legend()
#         plt.show()


# # Path to the OSM file
# osm_file_path = os.path.join(os.getcwd(), 'control', 'lanelet2_map.osm')

# point = (5.513063, 52.459907)

# map = Map(point, osm_file_path)
# map.plot_location()


# print(map.check_point_in_polygon(point))
# point2 = (5.511368, 52.458993)
# # print(map.check_point_in_polygon(point2))
# # map.plot_location(point2)
# # point3 = (5.511435, 52.459028)
# # print(map.check_point_in_polygon(point3))
# # map.plot_location(point3)

