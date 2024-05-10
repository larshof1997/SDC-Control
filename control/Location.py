import osmnx as ox
import geopandas as gpd
import shapely.geometry as geometry
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon, MultiLineString
import os

class Map:
    def __init__(self, position, osm_file):
        # Position should be an (x, y) tuple
        # osm_file should be an OSM file path

        self.position = Point(position)  # Initial position [x, y]

        # Read the OSM file and create a graph
        self.graph = ox.graph_from_xml(osm_file)

        # Extract the edges (roads) from the graph as a GeoDataFrame
        self.edges_gdf = ox.graph_to_gdfs(self.graph, nodes=False, edges=True)

        # Preprocess the edges GeoDataFrame
        self.preprocess_edges()

    def preprocess_edges(self):
        # Filter out unnecessary columns and convert coordinates to local coordinate system
        self.edge_geometries = self.edges_gdf['geometry']
        # You may add more preprocessing steps here if needed

    

    def update_location(self, position):
        if self.check_point_in_polygon(position):
            self.position = Point(position)  # Update position
        else:
            raise ValueError("New location not in polygon")

    def check_point_in_geometry(self, position):
        # Create a point from the given position
        point = Point(position)

        # Check if the point is within any of the edge geometries
        for geometry in self.edge_geometries:
            if geometry.contains(point):
                return True
        return False

    def plot_polygon(self):
        # Plot the polygon
        self.edge_geometries.plot()
        plt.show()

    def plot_location(self):
        # Plot the polygon and current location
        fig, ax = plt.subplots()
        self.edge_geometries.plot(ax=ax)
        ax.scatter(self.position.x, self.position.y, color='red', label='Current Location')
        plt.title('Map with Current Location')
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.legend()
        plt.show()


# Path to the OSM file
osm_file_path = os.path.join(os.getcwd(), 'control', 'sdc_track_lelystad.osm')

point = (5.513063, 52.459907)

map = Map(point, osm_file_path)
map.plot_location()
print(map.check_point_in_geometry(point))
print(map.check_point_in_geometry((5.511368, 52.458993)))
print(map.check_point_in_geometry((5.511435, 52.459028)))

