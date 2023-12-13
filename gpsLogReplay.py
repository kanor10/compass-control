"""
GPS Waypoints Logger and Replayer

This script is developed for logging and replaying GPS waypoints using a Viam robot client. 
It facilitates connecting to a robot client, logging waypoints with specified 
sample periods, and replaying waypoints from a log file.

Disclaimer:
This script was developed with the assistance of ChatGPT 4, an AI language model from OpenAI. 
While ChatGPT provided guidance on code structure, error handling, and Python best practices, 
the final implementation was the result of collaborative efforts.

Please note that this script is provided "as is", without warranty of any kind, express or implied, 
including but not limited to the warranties of merchantability, fitness for a particular purpose 
and noninfringement. In no event shall the authors, contributors, or OpenAI be liable for any 
claim, damages, or other liability, whether in an action of contract, tort or otherwise, 
arising from, out of, or in connection with the script or the use or other dealings in the script.

Basically, if you try to rerun a log file when the robot isn't at the original location and it
drives off a cliff, that's a you problem.

Author: Ann Larson, ann@intermode.io
Date: 2023-11-17
Collaboration: Developed with assistance from ChatGPT 4 (OpenAI)
"""

import csv
import sys
import os
import time
import argparse
import asyncio
import signal
import numpy as np
from datetime import datetime
from dotenv import load_dotenv

from viam.robot.client import RobotClient
from viam.components.base import Base
from viam.components.movement_sensor import MovementSensor
from viam.services.navigation import NavigationClient, Mode, GeoPoint

# Constants
DEFAULT_SAMPLE_PERIOD_LOG = 0.2  # seconds
DEFAULT_SAMPLE_PERIOD_REPLAY = 1.0  # seconds
MOVEMENT_SENSOR_NAME = "gps"
NAVIGATION_SERVICE_NAME = "navServ"
BASE_NAME = "intermode-base"
WAYPOINT_ANGLE_MAX = 30  # degrees


# Function to establish a connection to the robot client
async def connect():
    """
    Establishes a connection to the robot client using API keys and host address
    from environment variables.

    Raises:
        ValueError: If required environment variables are not set.

    Returns:
        RobotClient: An instance of the RobotClient connected to the given address.
    """
    api_key = os.environ.get('ENV_API_KEY')
    api_key_id = os.environ.get('ENV_API_KEY_ID')
    host = os.environ.get('ENV_HOST')

    if not all([api_key, api_key_id, host]):
        raise ValueError("Required environment variables are not set")

    opts = RobotClient.Options.with_api_key(api_key=api_key, api_key_id=api_key_id)
    return await RobotClient.at_address(host, opts)

def read_waypoint_csv_to_array(file_path):
    """
    Reads a waypoint log CSV file and converts the data into an array.

    Ignores any path data in the CSV file.

    Args:
        file_path (str): Path to the CSV file.

    Returns:
        list of tuples: Each tuple contains (timestamp, latitude, longitude, altitude).
    """
    data = []
    with open(file_path, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        next(csvreader)  # Skip the header row
        for i, row in enumerate(csvreader):
            # Convert each row to tuple and append to data array
            data.append((float(row[0]), float(row[1]), float(row[2]), float(row[3])))

            # Print a message every 100 rows to show progress
            if i % 100 == 0:
                print(f"Processed {i} rows...")

    print("Finished processing CSV file.")
    return data

def create_replay_file_writer(original_file_path):
    """
    Creates a file writer for a new file with '_replay' added to the original file name.

    Args:
        original_file_path (str): The path to the original file including its name.

    Returns:
        file: A file object opened for writing.
    """
    # Split the original file path into directory, file name and extension
    directory, file_name = os.path.split(original_file_path)
    name, extension = os.path.splitext(file_name)

    # Create new file name with '_replay' before the extension
    new_file_name = f"{name}_replay{extension}"
    new_file_path = os.path.join(directory, new_file_name)

    print(f"Creating replay log: {new_file_path}")

    # Open the new file for writing and return the file object
    return open(new_file_path, 'w')

async def log_cleanup_and_exit(nav_move, file):
    """
    Perform logging cleanup operations and exit the script.

    Args:
        nav_move (MovementSensor): Movement sensor client instance.
        file (file): File object for the replay log.
    """
    print("Exiting logging")
    try:
        await nav_move.set_mode(Mode.MODE_MANUAL)
    except:
        file.close()
        print("Cleanup done. Exiting.")
        sys.exit(0)

async def replay_cleanup_and_exit(nav_serv, base, file):
    """
    Perform replay cleanup operations and exit the script.

    Args:
        nav_serv (NavigationClient): Navigation service client instance.
        base (Base): Base client instance.
        file (file): File object for the replay log.
    """
    print("Aborting navigation. Stopping base")
    await nav_serv.set_mode(Mode.MODE_MANUAL)
    await base.stop()
    file.close()
    print("Cleanup done. Exiting.")
    sys.exit(0)

def calculate_angle(p1, p2, p3):
    """
    Calculate the angle at p2 formed by the line segments p1-p2 and p2-p3.
    The points are in the format [latitude, longitude].
    """
    # Convert to radians
    p1 = np.radians(p1)
    p2 = np.radians(p2)
    p3 = np.radians(p3)

    # Calculate the differences in the coordinates
    delta_p1_p2 = p2 - p1
    delta_p2_p3 = p3 - p2

    # Check for zero differences to avoid division by zero
    if np.isclose(delta_p1_p2[0], 0.0) or np.isclose(delta_p2_p3[0], 0.0):
        return 0  # Return a straight line angle if there's no significant change in latitude

    # Calculate the tangents
    tan1 = np.tan(delta_p1_p2[1]) / np.tan(delta_p1_p2[0])
    tan2 = np.tan(delta_p2_p3[1]) / np.tan(delta_p2_p3[0])

    # Calculate the angle in radians
    angle = np.arctan(np.abs((tan2 - tan1) / (1 + tan1 * tan2)))

    # Convert to degrees
    angle = np.degrees(angle)

    return angle

# Function to log waypoints with a given sample period
async def log_waypoints(mode, file_path, robot, sample_period):
    """
    Logs waypoints to a specified file at a given sample period.

    Args:
        mode (str): Mode of operation - 'log_gps' or 'log_paths'.
        file_path (str): Path to the output log file.
        robot (RobotClient): Robot client instance.
        sample_period (float): The time interval (in seconds) between logging waypoints.
    """
    if mode == 'log_paths':
        try:
            nav_serv = NavigationClient.from_robot(robot, NAVIGATION_SERVICE_NAME)
        except ValueError:
            print("Navigation service not found. Please try again or run this script in 'log_gps' mode.")
            sys.exit(1)

    nav_move = MovementSensor.from_robot(robot, MOVEMENT_SENSOR_NAME)
    print(f"Logging waypoints to {file_path} with period {sample_period} seconds")

    file = open(file_path, 'a')

    # Setup the signal handler
    def signal_handler(sig, frame):
        asyncio.create_task(log_cleanup_and_exit(nav_move, file))

    signal.signal(signal.SIGINT, signal_handler)

    file.write("timestamp, latitude, longitude, altitude\n")
    prevPoints = [[], []]
    prevLogStr = ""
    while True:
        timestamp = datetime.now().timestamp()
        position = await nav_move.get_position()
        positionStr = f"{position[0].latitude}, {position[0].longitude}, {position[1]}"

        if mode == 'log_paths':
            paths = await nav_serv.get_paths()
            positionStr = f"{positionStr}, {paths}"

        logStr = f"{timestamp}, {positionStr}\n"

        # Filter out points with an angle less than WAYPOINT_ANGLE_MAX
        if len(prevPoints[0]) == 0:
            prevPoints[0] = [position[0].latitude, position[0].longitude]
            file.write(logStr)
        else:
            if len(prevPoints[1]) != 0:
            # Calculate the angle between the three points
                angle = calculate_angle(prevPoints[0], prevPoints[1], [position[0].latitude, position[0].longitude])
                print(f"Angle: {angle}")
                # If the angle is greater than or equal to WAYPOINT_ANGLE_MAX, store position
                #  Otherwise, replace the most recent point with the current position
                if angle >= WAYPOINT_ANGLE_MAX:
                    prevPoints[0] = prevPoints[1]
                    file.write(prevLogStr)
            
            prevPoints[1] = [position[0].latitude, position[0].longitude]
        
        print(f"prevPoints: {prevPoints}")
        prevLogStr = logStr
        print(positionStr)
        
        time.sleep(sample_period)

# Function to replay waypoints from a file
async def replay_waypoints(file_path, robot, sample_period):
    """
    Replays waypoints from a given file.

    Args:
        file_path (str): Path to the input file containing waypoints.
        robot (RobotClient): Robot client instance.
        sample_period (float): The time interval (in seconds) between replaying waypoints.
    """
    nav_serv = NavigationClient.from_robot(robot, NAVIGATION_SERVICE_NAME)
    base = Base.from_robot(robot, BASE_NAME)

    # Setup the signal handler
    def signal_handler(sig, frame):
        asyncio.create_task(replay_cleanup_and_exit(nav_serv, base, file))

    signal.signal(signal.SIGINT, signal_handler)

    print(f"Loading waypoints from {file_path}")
    waypoints_replay = read_waypoint_csv_to_array(file_path)

    for i, waypoint in waypoints_replay:
        location = GeoPoint(waypoint[1], waypoint[2])
        await nav_serv.add_waypoint(location)
        print(f"Added waypoint {i}: {location.latitude}, {location.longitude}")

    file = create_replay_file_writer(file_path)

    print("Starting navigation replay")
    file.write("timestamp, path\n")

    waypoints_queued = await nav_serv.get_waypoints()
    paths = await nav_serv.get_paths()
    await nav_serv.set_mode(Mode.MODE_WAYPOINT)

    # Technically, the waypoints and paths are not from the same point in time, but
    #   they should be close enough for most analyses
    while len(waypoints_queued) > 0:
        timestamp = datetime.now().timestamp()
        pathsStr = f"{paths}"
        file.write(f"{timestamp}, {pathsStr}")
        print(pathsStr)
        time.sleep(sample_period)
        waypoints_queued = await nav_serv.get_waypoints()
        paths = await nav_serv.get_paths()

    replay_cleanup_and_exit(nav_serv, base, file)

# Main function to execute log or replay mode
async def main(mode, file_path=None, sample_period=None):
    """
    Main function to execute the log or replay mode based on the arguments provided.

    Args:
        mode (str): Mode of operation - 'log_gps,' 'log_paths,' or 'replay'.
        file_path (str, optional): Path to the file for logging or replaying waypoints.
        sample_period (float, optional): Sample period for logging waypoints.
    """
    robot = await connect()

    if mode == 'log_gps' or mode == 'log_paths':
        await log_waypoints(mode, file_path, robot, sample_period)
    elif mode == 'replay':
        await replay_waypoints(file_path, robot, sample_period)
    else:
        print(f"Invalid mode: {mode}")
        sys.exit(1)

if __name__ == "__main__":
    # Load environment variables
    load_dotenv()

    parser = argparse.ArgumentParser(description="GPS Waypoints Logger and Replayer")
    parser.add_argument("mode", help="Mode: 'log_gps,' 'log_paths,' or 'replay'")
    parser.add_argument("--period", type=float, help="Sample period for logging")
    parser.add_argument("--file", help="Path to the GPS data file (required for 'replay' mode)")

    args = parser.parse_args()

    mode = args.mode
    file = args.file
    period = args.period

    if mode == 'log_gps' or mode == 'log_paths':
        if period is None:
            print(f"No sample period provided, defaulting to {DEFAULT_SAMPLE_PERIOD_LOG} seconds")
            period = DEFAULT_SAMPLE_PERIOD_LOG
        if file is None:
            current_time = datetime.now().strftime("%Y%m%d_%H-%M-%S")
            fname = f'gpsLog_{current_time}'

            if mode == 'log_paths':
                fname = fname + '_paths'
                
            file = os.path.join('./logs', fname + '.csv')
            os.makedirs(os.path.dirname(file), exist_ok=True)
            print(f"No log file provided, defaulting to: {file}")
        else:
            directory = os.path.dirname(file)
            if not os.path.exists(directory):
                os.makedirs(directory)
                print(f"Directory {directory} does not exist, creating it")
    elif mode == 'replay':
        if period is None:
            print(f"No sample period provided, defaulting to {DEFAULT_SAMPLE_PERIOD_REPLAY} seconds")
            period = DEFAULT_SAMPLE_PERIOD_REPLAY
        if file is None:
            parser.error("replay mode requires an input file (--file)")

    asyncio.run(main(mode, file, period))
