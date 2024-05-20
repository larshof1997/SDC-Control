import osmnx as ox
import geopandas as gpd
import shapely.geometry as geometry
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon, MultiLineString, MultiPolygon, LineString
import os
import numpy as np

class Map:
    def __init__(self, position, osm_file):
        # Position should be an (x, y) tuple
        # osm_file should be an OSM file path

        self.position = Point(position)  # Initial position [x, y]

        # Read the OSM file and create a graph
        G = ox.graph_from_xml(osm_file)
        nodes, edges = ox.graph_to_gdfs(G, fill_edge_geometry=True)
        self.graph = ox.graph_from_gdfs(nodes, edges, graph_attrs=G.graph)

        # self.features = ox.features_from_xml(osm_file)

        # Extract the lines from the graph as a GeoDataFrame
        self.lines = edges

        # self.lines = self.features[self.features['type']=='line_thin']
        # self.graph = ox.graph_from_gdfs(self.features, nodes=True, edges=False)
        
        # print(lines['geometry'])

        # # Extract the edges (roads) from the graph as a GeoDataFrame
        # self.edges_gdf = ox.graph_to_gdfs(self.graph, nodes=False, edges=True)

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
                plt.plot(x, y, label=f'Line {i+1}')
                lines[f"{i+1}"] = list(line_string.coords)
                print(list(line_string.coords))
        concatenated_line_string = [
            list(reversed(list(lines["11"])))+
            list(lines["12"]) +
            list(reversed(list(lines["14"]))) +
            list(reversed(list(lines["6"]))) +
            list(lines["8"]) 
        ]


        polypoints = []
        for tuple in concatenated_line_string:
            print(tuple)
            for point in tuple:
                print(point)
                polypoints.append([point[0],point[1]])

        print(polypoints)
        
        # Concatenate LineStrings in the desired order to form the polygon boundary
        concatenated_line_string = LineString(
            lines["11"] +
            lines["14"] +
            list(reversed(lines["12"])) +
            list(reversed(lines["6"])) +
            lines["8"]
        )

        # Plot the concatenated LineString to visualize the boundary
        plt.plot(*concatenated_line_string.xy, label='Polygon Boundary', color='black')

        # Show legend
        plt.legend()

        # Convert the concatenated LineString to a Polygon
        self.polygon = Polygon(polypoints)
        plt.legend()  # Add legend
        plt.show()
        # # Create a polygon from all coordinates
        # self.polygon = Polygon(concatenated_line_string)
        # self.polygon = Polygon(polypoints)
        # self.polygon = Polygon(lines)

        # self.polygon =Polygon(
        # [
        #     [10.470650724002098, 55.38520996875883],
        #     [10.469291130631092, 55.38477524425895],
        #     [10.468346742421403, 55.3855938172706],
        #     [10.466164880005067, 55.384530131529345],
        #     [10.46587993528641, 55.38429426690277],
        #     [10.46378762692467, 55.381052141937516],
        #     [10.46446335297145, 55.38091801121436],
        #     [10.465041383686241, 55.380941137233464],
        #     [10.465733392288826, 55.3808902599732],
        #     [10.466262575337083, 55.3807653791485],
        #     [10.467736146595996, 55.38083475743292],
        #     [10.469681911959213, 55.38064974840299],
        #     [10.470015704344348, 55.381177021857354],
        #     [10.4717823615984, 55.38068212504581],
        #     [10.472189425482355, 55.380964263239036],
        #     [10.473744409517451, 55.38081625656886],
        #     [10.470553028670054, 55.381861541821166],
        #     [10.469535368961601, 55.38115852115337],
        #     [10.467695440207848, 55.381491532500746],
        #     [10.469193435299047, 55.38253217489091],
        #     [10.467003431605235, 55.3834109182535],
        #     [10.468371166253775, 55.384391387802054],
        #     [10.47115548321807, 55.38393815489968],
        #     [10.471619536045012, 55.384285017281144],
        #     [10.470650724002098, 55.38520996875883],
        # ]
        # )
        


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


# Path to the OSM file
osm_file_path = os.path.join(os.getcwd(), 'control', 'lanelet2_map_versimpeld.osm')

point = (5.513063, 52.459907)
# point = (10.46625, 55.382)

map = Map(point, osm_file_path)
map.plot_location()


print(map.check_point_in_polygon(point))
point2 = (5.511368, 52.458993)
# point2 = (10.46805, 55.3815)
print(map.check_point_in_polygon(point2))
map.plot_location(point2)
# point3 = (5.511435, 52.459028)
# print(map.check_point_in_polygon(point3))
# map.plot_location(point3)

