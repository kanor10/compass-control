import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import math
from pyproj import Transformer
matplotlib.use('agg')
# Full script to read a CSV file, extract coordinates, convert them to linear distances, and plot

def plotPath(waypoint_path,csv_file_paths,colors):
    # Step 1: Read the CSV file
    OutputDict = {}
    i=0 #Iterator for colors
    plt.figure(figsize=(10, 8))
    transformer = Transformer.from_crs("epsg:4326", "epsg:32616", always_xy=True)
    ref_lat , ref_lon = 42.3289066666 , -83.076005333 #The Firehydrant coord
    ref_x, ref_y = transformer.transform(ref_lon, ref_lat)
    waypoints = pd.read_csv(waypoint_path)
    waypoints['X'], waypoints['Y'] = zip(*waypoints.apply(
            lambda row: latlon_to_linear_dist_csv(row['latitude'], row['longitude'], transformer, (ref_x, ref_y), 0), axis=1))
        
    for csv_file_path in csv_file_paths:
        csv_data = pd.read_csv(csv_file_path)

        # Step 2: Define a transformer for coordinate conversion

        # Step 3: Define the reference point
        #ref_lat, ref_lon = csv_data['latitude'].min(), csv_data['longitude'].min()
        #lat":42.32890666666667,"lng":-83.07600533333334

        # Step 4: Convert latitude and longitude to linear distances
        csv_data['X'], csv_data['Y'] = zip(*csv_data.apply(
            lambda row: latlon_to_linear_dist_csv(row['latitude'], row['longitude'], transformer, (ref_x, ref_y), 0), axis=1))
        
        csv_data['DistancetoRoute'] = calculate_distances(waypoints,csv_data)
        # Step 5: Plot the linear coordinates
        #labeled by file name
        OutputDict[f'{csv_file_path}'] = csv_data
        plt.scatter(csv_data['X'], csv_data['Y'], color=colors[i],label=csv_file_path) 
        i=i+1
    plt.title("Service Comparison (reference point of fire hydrant)")
    plt.xlabel('Distance East from Reference Point [cm]')
    plt.ylabel('Distance North from Reference Point [cm]')
    plt.legend(loc='upper left')
    plt.grid(True)
    plt.savefig("Combo.png")
    return OutputDict

def latlon_to_linear_dist_csv(lat, lon, transformer, ref_point, rotation):
    x, y = transformer.transform(lon, lat)
    new_x, new_y = (x - ref_point[0]) * 100, (y - ref_point[1]) * 100
    return xy_rotation(new_x,new_y,rotation)

def xy_rotation(x,y,theta):
    return (x*math.cos(theta) + y*math.sin(theta)),(y*math.cos(theta) - x*math.sin(theta))

def calculate_distances(waypoints, gps_coordinates):
    distances = []
    for row in gps_coordinates.itertuples(index=True, name='Pandas'):
        min_distance = float('inf')
        for i in range(len(waypoints)-1):
            distance = distance_point_to_segment(getattr(row, 'X'),getattr(row, 'Y'), 
                                                 waypoints.iloc[i,2],waypoints.iloc[i,3], waypoints.iloc[i + 1,2],waypoints.iloc[i+1,3])
            min_distance = min(min_distance, distance)
        distances.append(min_distance)
    return distances

def dot(v, w):
    """Dot product of two vectors."""
    return v[0] * w[0] + v[1] * w[1]

def length_squared(v):
    """Square of the length of a vector."""
    return dot(v, v)

def distance_point_to_segment(px, py, x1, y1, x2, y2):
    """Calculate the distance from a point (px, py) to a line segment (x1, y1)-(x2, y2)."""
    P = (px, py)
    A = (x1, y1)
    B = (x2, y2)
    
    # Vector from A to B
    AB = (B[0] - A[0], B[1] - A[1])
    
    # Vector from A to P
    AP = (P[0] - A[0], P[1] - A[1])
    
    # Calculate the squared distance from A to B
    AB_squared = length_squared(AB)
    if AB_squared == 0:
        # A and B are the same points, return distance from A to P
        return math.sqrt(length_squared(AP))
    
    # Projection of AP onto AB, clamped between 0 and 1
    t = max(0, min(1, dot(AP, AB) / AB_squared))
    
    # Closest point on the line segment to P
    closest_point = (A[0] + t * AB[0], A[1] + t * AB[1])
    
    # Distance from P to the closest point
    return math.sqrt(length_squared((P[0] - closest_point[0], P[1] - closest_point[1])))



#main(['logs/gpsLog_20231222_09-41-03.log','logs/gpsLog_20231222_09-45-17.log','logs/gpsLog_20231222_09-48-25.log','Figure8.csv'],['yellow','blue','green','red'])
#main(['logs/gpsLog_20231222_10-19-54.log','logs/gpsLog_20231222_10-15-13.log','logs/gpsLog_20231222_10-11-58.log','StraightLineLoop.csv'],['yellow','green','blue','red'])
#main(['logs/gpsLog_20231222_10-47-38.log','logs/gpsLog_20231222_10-44-41.log','logs/gpsLog_20231222_10-37-28.log','logs/gpsLog_20231222_10-34-10.log','StraightLineLoop.csv'],['orange','yellow','green','blue','red'])
#main(['logs/gpsLog_20231222_12-06-55.log','logs/gpsLog_20231222_12-01-17.log','logs/gpsLog_20231222_11-57-23.log','logs/gpsLog_20231222_11-54-11.log','logs/gpsLog_20231222_11-51-22.log','StraightLineLoop.csv'],['purple','orange','yellow','green','blue','red'])
# Run the main function with the path to your CSV file
# Example usage: main(['/path/to/your/file1.csv','/path/to/your/file2.csv'], ['red',blue'])
# This script should be run in an environment where the specified libraries are installed and the file path is accessible.

if __name__ == '__main__':
    db = plotPath('StraightLineLoop.csv',['logs/gpsLog_20231222_12-06-55.log','logs/gpsLog_20231222_12-01-17.log','logs/gpsLog_20231222_11-57-23.log','logs/gpsLog_20231222_11-54-11.log','logs/gpsLog_20231222_11-51-22.log','StraightLineLoop.csv'],['purple','orange','yellow','green','blue','red'])
    for i in db:
        datas = db[i]
        datas.drop(datas[(datas['X'] < -1250) | (datas['X'] > 0)].index, inplace=True)
        plt.figure(figsize=(10, 6))
        plt.scatter(datas['X'], datas['Y'], c=datas['DistancetoRoute'], cmap='viridis')
        plt.xlim([-1300, 100])  # Set appropriate values for x-axis scale
        plt.ylim([500, 3600])  # Set appropriate values for y-axis scale
        plt.clim(0,40)
        plt.colorbar(label='Distance to Route')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title(f'{i}')
        plt.grid(True)
        plt.savefig(f"{i}.png")
        db[i].to_csv(f"{i}.csv")