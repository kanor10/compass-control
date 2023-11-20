import asyncio
import time
import math
import random
from viam.components.motor import Motor
from viam.robot.client import RobotClient

class BoxBot:
    """
    A **BoxBot** class (for a four wheeled, differential steering robot) which contains the main
    functions for GPS waypoint navigation of a robot using a compass.
    """
    def __init__(self, robot):
        self.left_front = Motor.from_robot(robot, "left-front")
        self.left_rear = Motor.from_robot(robot, "left-rear")
        self.right_front = Motor.from_robot(robot, "right-front")
        self.right_rear = Motor.from_robot(robot, "right-rear")

    async def drive(self, power_drive, power_spin):
        """
        This method sets the motor powers. power_drive will control forwards/reverse motion and
        power_spin will handle rotations. For example (1,0) would be forward full power and (0,1)
        would be spin clockwise.
        """
        left_power = power_drive + power_spin
        right_power = power_drive - power_spin

        await self.left_front.set_power(left_power)
        await self.left_rear.set_power(left_power)
        await self.right_front.set_power(right_power)
        await self.right_rear.set_power(right_power)

    async def setheading(self, boxbot, pid, xsens, gps, latD, longD):
        """
        This method is used to enter a control loop and turn the robot (this is implementation uses
        differential steering). This loop will run so that the heading error is constantly being
        measured and corrected. When the heading is reached (within 5 degrees), this control loop
        is broken. There is some commented out logic that will give some additional settling time
        if needed - although this is not neccessary.
        """
        reached_desired_heading = False
        #start_time = 0  # Initialize the timer

        start_time = time.time()  # Initialize the timer

        while True:
            # Read in heading
            raw_heading = await xsens.get_compass_heading()
            # Adjust heading to be within the desired range (0-360 degrees)
            actual_heading = (raw_heading) % 360

            coords = await gps.get_position()
            latitude = coords[0].latitude
            longitude = coords[0].longitude

            #get heading and distance to target 2
            heading_distance = await boxbot.calculate_heading_and_distance(latitude, longitude, latD, longD)
            desired_heading, distance = heading_distance

            # Angle wraparound logic
            if abs(desired_heading - actual_heading) > 180:
                if desired_heading > actual_heading:
                    actual_heading += 360
                else:
                    desired_heading += 360

            #calculate the control output
            control_output = pid.calculate(desired_heading, actual_heading)
            #print("turning")

            await boxbot.drive(0,control_output)  # Use the BoxBot for spinning

            # Stop spinning once the desired heading is reached
            if abs(actual_heading-desired_heading) < 5:
                break
            #    if not reached_desired_heading:
            #        reached_desired_heading = True
            #        start_time = time.time()  # Start the timer when the heading is first reached

            #if reached_desired_heading:
            #    elapsed_time = time.time() - start_time  # Calculate elapsed time

                # Keep running for 3 seconds after reaching the desired heading
            #    if elapsed_time <= 3.0:
            #        pass  # You can add additional actions here

                # After 2 seconds, stop the robot and exit the loop
            #    if elapsed_time > 3.0:
            #        await boxbot.drive(0, 0)  # Stop spinning
            #        print("Desired heading reached.")
            #        break



    async def goto_coord(self, boxbot, pid, xsens, gps, drive_speed,latD,longD,data):
        """
        This method is used to send the robot towards a coordinate using a control loop. It also
        has some data capture functionality which logs the GPS coordinates as the robot drives to a
        waypoint. If the robot gets within a small distance of the GPS location, it is considered
        to have achieved the goal and breaks out of the loop.
        """
        print("going to coord")
        start_time = time.time()  # Initialize the timer
        time_increment = 1

        while True:


            #data timing

            current_time = time.time() - start_time
            time_since_last_increment = current_time % time_increment

            #print(time_since_last_increment)


            coords = await gps.get_position()
            latitude = coords[0].latitude
            longitude = coords[0].longitude

            if time_since_last_increment >= 0.9:
                    #timestamp = round(current_time,1)
                    data.append([latitude,longitude])
                    #print(data)

            #get heading and distance to target 2
            heading_distance = await boxbot.calculate_heading_and_distance(latitude, longitude, latD, longD)
            desired_heading, distance = heading_distance


            # Read in heading
            raw_heading = await xsens.get_compass_heading()-15
            # Adjust heading to be within the desired range (0-360 degrees)
            actual_heading = (raw_heading) % 360

            # Angle wraparound logic
            if abs(desired_heading - actual_heading) > 180:
                if desired_heading > actual_heading:
                    actual_heading += 360
                else:
                    desired_heading += 360

            #calculate the control output for heading compensation and set forward power to 80%
            control_output = pid.calculate(desired_heading, actual_heading)
            await boxbot.drive(0.8,control_output)

            if distance<0.0008:
                print("here")
                #can uncomment this to delineate waypoints
                #data.append([999.9,999.9])
                #print(data)
                return data
                break


    async def calculate_heading_and_distance(self, lat1, lon1, lat2, lon2):
        """
        This method will accept a current GPS location and a desired GPS location, compute the
        heading and distance, and return these.
        """
        # Convert latitude and longitude from degrees to radians
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)

        # Calculate the difference in latitude and longitude
        dlat = lat2 - lat1
        dlon = lon2 - lon1

        # Calculate distance using Haversine formula
        a = (math.sin(dlat/2)**2) + math.cos(lat1) * math.cos(lat2) * (math.sin(dlon/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = 6371 * c  # Earth's radius in kilometers

        # Calculate the initial bearing
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        initial_bearing = math.atan2(y, x)

        # Convert the initial bearing to degrees
        initial_bearing = math.degrees(initial_bearing)

        # Normalize the initial bearing to the range [0, 360] degrees
        compass_bearing = (initial_bearing + 360) % 360

        return compass_bearing, distance

    async def gotopoint(self, boxbot,gps,pid,xsens,latD,longD,data):
        """
        This calls setheading and gotocoord together, such that the robot can be requested to turn
        and go to the correct GPS point.
        """
        #get current position
        coords = await gps.get_position()
        latitude = coords[0].latitude
        longitude = coords[0].longitude

        #get heading and distance to target
        heading_distance = await boxbot.calculate_heading_and_distance(latitude, longitude, latD, longD)
        desired_heading, distance = heading_distance

        #face target 1 and settle
        await boxbot.setheading(boxbot, pid, xsens, gps, latD, longD)

        #goto target 1 and stop when reached
        await boxbot.goto_coord(boxbot, pid, xsens, gps, 0.6, latD, longD,data)


class PIDController:
    """
    A **PIDController** class which is a simple PID controller than needs PID gains and integral
    limits that prevent integral control windup.
    """
    def __init__(self, kp, ki, kd, integral_max, integral_min):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max = integral_max
        self.integral_min = integral_min
        self.integral = 0
        self.previous_error = 0

    def calculate(self, desired_value, current_value):
        error = desired_value - current_value

        control_output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * (error - self.previous_error)
        )

        # Anti-windup: Limit the integral term to prevent excessive accumulation
        self.integral += error
        self.integral = max(min(self.integral, self.integral_max), self.integral_min)

        self.previous_error = error

        return control_output
