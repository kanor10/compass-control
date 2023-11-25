import time
import math
from viam.components.base import Base, Vector3

LOOP_PERIOD = 0.2  # seconds
PRINT_PERIOD = 1  # seconds
HEADING_TOLERANCE = 5  # degrees
LINEAR_COMMAND = 0.3  # fractional power
LINEAR_COMMAND_MINTURN = 0.2  # fractional power to enforce minimum turn radius
LINEAR_COMMAND_MAX = 0.5  # fractional power to enforce maximum speed
ANGULAR_COMMAND_MAX = 0.2  # fractional power to enforce maximum spin speed

class BoxBot:
    """
    A **BoxBot** class (for a four wheeled, differential steering robot) which contains the main
    functions for GPS waypoint navigation of a robot using a compass.
    """
    def __init__(self, robot):
        self.base = Base.from_robot(robot,"intermode-base")

    async def drive(self, power_drive, power_spin):
        """
        This method sets the motor powers. power_drive will control forwards/reverse motion and
        power_spin will handle rotations. For example (1,0) would be forward full power and (0,1)
        would be spin clockwise.
        """
        linear_command = power_drive
        angular_command = -1*power_spin # Negative sign to make clockwise negative

        # Limit both commands to the range of -max to max
        linear_command = max(min(linear_command, LINEAR_COMMAND_MAX), -LINEAR_COMMAND_MAX)
        angular_command = max(min(angular_command, ANGULAR_COMMAND_MAX), -ANGULAR_COMMAND_MAX)
        
        print(f"Linear: {linear_command}, Angular: {angular_command}")

        linearVec = Vector3(x=0.0, y=linear_command, z=0.0)
        angularVec = Vector3(x=0.0, y=0.0, z=angular_command)
        await self.base.set_power(linearVec, angularVec)

#################
    # async def setheading(self, pid):
#################
    async def setheading(self, boxbot, pid, xsens, gps, latD, longD):
        """
        This method is used to enter a control loop and turn the robot (this is implementation uses
        differential steering). This loop will run so that the heading error is constantly being
        measured and corrected. When the heading is reached (within 5 degrees), this control loop
        is broken. There is some commented out logic that will give some additional settling time
        if needed - although this is not neccessary.
        """
        TARGET_HEADING = 180
        MAX_ANGULAR_VELOCITY = 360
        last_time = 0
        actual_heading_global = 0
        control_output = 0

        while True:
            start_time = time.time()

            # Read in heading
            raw_heading = await xsens.get_compass_heading()
            # Adjust heading to be within the desired range (0-360 degrees)
            actual_heading = (raw_heading) % 360

            coords = await gps.get_position()
            latitude = coords[0].latitude
            longitude = coords[0].longitude

            print(f"Coordinates: {latitude}, {longitude}, {actual_heading}")

            # get heading and distance to target 2
            heading_distance = await boxbot.calculate_heading_and_distance(latitude, longitude, latD, longD)
            desired_heading, _ = heading_distance

#################
            # desired_heading = TARGET_HEADING
            # actual_heading = actual_heading_global
            # if last_time != 0:
            #     actual_heading = actual_heading_global + control_output *(start_time-last_time) * MAX_ANGULAR_VELOCITY
            #     actual_heading_global = actual_heading
            # last_time = start_time

            # print(f"Actual: {actual_heading_global:07.3f}, Target: {TARGET_HEADING:07.3f}, Command: {control_output:07.3f}")
#################
            print(f"Actual: {actual_heading:07.3f}, Target: {desired_heading:07.3f}, Command: {control_output:07.3f}")

            # Angle wraparound logic
            if abs(desired_heading - actual_heading) > 180:
                if desired_heading > actual_heading:
                    actual_heading += 360
                else:
                    desired_heading += 360

            #calculate the control output
            control_output = pid.calculate(desired_heading, actual_heading)

            control_output = max(min(control_output, ANGULAR_COMMAND_MAX), -ANGULAR_COMMAND_MAX)
            await boxbot.drive(0,control_output)  # Use the BoxBot for spinning
            
            # Stop spinning once the desired heading is reached
            if abs(actual_heading-desired_heading) < HEADING_TOLERANCE:
                print("Target heading reached")
                break

            time.sleep(max(start_time + LOOP_PERIOD - time.time(), 0))



    async def goto_coord(self, boxbot, pid, xsens, gps, drive_speed,latD,longD,data):
        """
        This method is used to send the robot towards a coordinate using a control loop. It also
        has some data capture functionality which logs the GPS coordinates as the robot drives to a
        waypoint. If the robot gets within a small distance of the GPS location, it is considered
        to have achieved the goal and breaks out of the loop.
        """
        print("going to coord")

        time_next_update = 0
        while True:
            start_time = time.time()

            coords = await gps.get_position()
            latitude = coords[0].latitude
            longitude = coords[0].longitude

            if start_time >= time_next_update:
                time_next_update = start_time + PRINT_PERIOD
                data.append([latitude,longitude])

            #get heading and distance to target 2
            heading_distance = await boxbot.calculate_heading_and_distance(latitude, longitude, latD, longD)
            desired_heading, distance = heading_distance

            # Read in heading
            raw_heading = await xsens.get_compass_heading()
            # Adjust heading to be within the desired range (0-360 degrees)
            actual_heading = (raw_heading) % 360

            # Angle wraparound logic
            if abs(desired_heading - actual_heading) > 180:
                if desired_heading > actual_heading:
                    actual_heading += 360
                else:
                    desired_heading += 360

            #calculate the control output for heading compensation and set forward power
            control_output = pid.calculate(desired_heading, actual_heading)
            await boxbot.drive(drive_speed, control_output)

            if distance<0.0008:
                print("here")
                return data
            
            time.sleep(max(start_time + LOOP_PERIOD - time.time(), 0))


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
        #face target 1 and settle
        await boxbot.setheading(boxbot, pid, xsens, gps, latD, longD)

        #goto target 1 and stop when reached
        await boxbot.goto_coord(boxbot, pid, xsens, gps, LINEAR_COMMAND, latD, longD,data)


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
        self.previous_calc_time = 0

    def calculate(self, desired_value, current_value):
        error = desired_value - current_value
        error_slope = 0
        
        # Anti-windup: Limit the integral term to prevent excessive accumulation
        if(self.previous_calc_time != 0):
        #     time_step = time.time() - self.previous_calc_time
        #     self.integral += error * time_step
        #     self.integral = max(min(self.integral, self.integral_max), self.integral_min)
        #     error_slope = (self.previous_error - error) / time_step
            self.integral += error
            self.integral = max(min(self.integral, self.integral_max), self.integral_min)
            error_slope = (self.previous_error - error)

        self.previous_error = error
        self.previous_calc_time = time.time()
            
        p_component = self.kp * error
        i_component = self.ki * self.integral
        d_component = self.kd * error_slope

        control_output = p_component + i_component + d_component

        # Print the PID components with three decimal places
        print(f"P: {p_component:06.3f}, I: {i_component:06.3f}, D: {d_component:06.3f}")

        # error = desired_value - current_value

        # control_output = (
        #     self.kp * error
        #     + self.ki * self.integral
        #     + self.kd * (error - self.previous_error)
        # )

        # # Anti-windup: Limit the integral term to prevent excessive accumulation
        # self.integral += error
        # self.integral = max(min(self.integral, self.integral_max), self.integral_min)

        # self.previous_error = error

        return control_output
