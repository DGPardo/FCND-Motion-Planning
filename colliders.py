import re
import numpy as np
from planning_utils import create_grid


class Colliders:
    def __init__(self, colliders_file_path='colliders.csv'):
        self.fpath = colliders_file_path

    def get_reference_lat_lon(self):
        try:
            with open(self.fpath, 'r') as fp:
                first_line = fp.readline()
        except IOError:
            raise IOError(f'{self.fpath} could not be read')

        # Example of expected format: lat0 37.792480, lon0 -122.397450
        line_parsed = re.search('lat0 (.*\d+.\d+.*), lon0 (.*\d+.\d+.*)', first_line)

        if line_parsed is not None:
            return float(line_parsed.group(1)), float(line_parsed.group(2))  # First and second match
        else:
            return None

    def read_data(self):
        return np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

    def get_obstacles_as_polygons(self, altitude, safety_distance):
        from shapely.geometry import Polygon
        polygons = []

        data = self.read_data()

        # minimum and maximum north coordinates
        north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
        north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

        # minimum and maximum east coordinates
        east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
        east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

        # given the minimum and maximum coordinates we can
        # calculate the size of the grid.
        north_size = int(np.ceil(north_max - north_min))
        east_size = int(np.ceil(east_max - east_min))

        for i in range(data.shape[0]):
            north, east, alt, d_north, d_east, d_alt = data[i, :]
            if alt + d_alt + safety_distance > altitude:
                nmin = int(np.clip(north - d_north - safety_distance - north_min, 0, north_size - 1))
                nmax = int(np.clip(north + d_north + safety_distance - north_min, 0, north_size - 1))
                emin = int(np.clip(east - d_east - safety_distance - east_min, 0, east_size - 1))
                emax = int(np.clip(east + d_east + safety_distance - east_min, 0, east_size - 1))

                p = Polygon([
                    (nmin, emin),
                    (nmax, emin),
                    (nmax, emax),
                    (nmin, emax)
                ])

                polygons.append(p)
        return polygons
