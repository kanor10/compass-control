import asyncio
import time
import math
import csv
import os
from dotenv import load_dotenv
from boxbot import BoxBot
from boxbot import PIDController
from viam.robot.client import RobotClient
from viam.components.movement_sensor import MovementSensor

# PID parameters
kp = 0.004  # Proportional gain
ki = 0.001  # Integral gain
kd = 0.002 # Derivative gain

# Integral term saturation limits
integral_max = 150  # Adjust as needed
integral_min = -150  # Adjust as needed

async def connect():
    # Load environment variables
    load_dotenv()
    
    api_key = os.environ.get('ENV_API_KEY')
    api_key_id = os.environ.get('ENV_API_KEY_ID')
    host = os.environ.get('ENV_HOST')
    
    opts = RobotClient.Options.with_api_key(
        api_key=api_key,
        api_key_id=api_key_id
    )
    return await RobotClient.at_address(host, opts)

def extract_coordinates_from_csv(file_path):
    """
    Reads a CSV file with columns: timestamp, latitude, longitude, altitude.
    Extracts the latitude and longitude points and returns them in an array
    where each element is an array containing a pair of coordinates.

    Args:
    file_path (str): The path to the CSV file.

    Returns:
    list of lists: A list of [latitude, longitude] pairs.
    """
    coordinates = []

    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)
        next(csv_reader)  # Skip the header row
        for row in csv_reader:
            if len(row) >= 3:
                latitude = float(row[1])
                longitude = float(row[2])
                coordinates.append([latitude, longitude])

    return coordinates

async def main():
#################
    # pid_angular = PIDController(kp, ki, kd, integral_max, integral_min)
    # boxbot = BoxBot()
    # await boxbot.setheading(pid_angular)
#################

    robot = await connect()
    xsens = MovementSensor.from_robot(robot, "gps")
    pid_angular = PIDController(kp, ki, kd, integral_max, integral_min)
    boxbot = BoxBot(robot)
    gps = MovementSensor.from_robot(robot, "gps")
    data=[]

    replay_file = os.environ.get('ENV_FILEPATH')
    GPSarray = extract_coordinates_from_csv(replay_file)

    for x in GPSarray:
        print('next point: ')
        print(x[0])
        print(", ")
        print(x[1])
        await boxbot.gotopoint(boxbot,gps,pid_angular,xsens,x[0],x[1],data)

    print(data)

    with open("raster5", 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(['lat', 'long'])
        csv_writer.writerows(data)
                    


if __name__ == '__main__':
    asyncio.run(main())
