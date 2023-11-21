import asyncio
import time
import math
import csv
import os
from boxbot import BoxBot
from boxbot import PIDController
from viam.robot.client import RobotClient
from viam.components.movement_sensor import MovementSensor

# PID parameters
kp = 0.08  # Proportional gain
ki = 0.002  # Integral gain
kd = 0.2 # Derivative gain

# Integral term saturation limits
integral_max = 400  # Adjust as needed
integral_min = -400  # Adjust as needed

GPSarray = [
    [42.32902, -83.07572],
    [42.32918, -83.07581],
    [42.32913,-83.07605],
    [42.32899,-83.07596],
    [42.32902,-83.0757]
    ]    


data=[]

async def connect():
    api_key = os.environ.get('ENV_API_KEY')
    api_key_id = os.environ.get('ENV_API_KEY_ID')
    host = os.environ.get('ENV_HOST')
    
    opts = RobotClient.Options.with_api_key(
        api_key=api_key,
        api_key_id=api_key_id,
        refresh_interval=0
    )
    return await RobotClient.at_address(host, opts)

async def main():
    robot = await connect()
    xsens = MovementSensor.from_robot(robot, "gps")
    pid = PIDController(kp, ki, kd, integral_max, integral_min)
    boxbot = BoxBot(robot)
    gps = MovementSensor.from_robot(robot, "gps")    


    #while True:


    for x in GPSarray:
        print('next point: ')
        print(x[0])
        print(", ")
        print(x[1])
        await boxbot.gotopoint(boxbot,gps,pid,xsens,x[0],x[1],data)


    print(data)

    with open("raster5", 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(['lat', 'long'])
        csv_writer.writerows(data)
                    


if __name__ == '__main__':
    asyncio.run(main())
