import time
import math
import numpy as np
import csv
from viam.components.base import Base, Vector3

LOOP_PERIOD = 0.2  # seconds
PRINT_PERIOD = 1  # seconds
HEADING_OFFEST = 0  # degrees
HEADING_TOLERANCE = 3  # degrees
LINEAR_COMMAND = 0.3  # fractional power
LINEAR_COMMAND_MINTURN = 0.2  # fractional power to enforce minimum turn radius
LINEAR_COMMAND_MAX = 1.0  # fractional power to enforce maximum speed
ANGULAR_COMMAND_MAX = 0.5  # fractional power to enforce maximum spin speed
WAYPOINT_TOLERANCE = 0.001  # kilometers
STEER_ANGLE_MAX = 55  # degrees
STEERING_ANGLE_OFFSET = -5  # offset steering commands by this many degrees
STEERING_FILTER_ALPHA = 1.0  # smoothing factor for steering angle

def low_pass_filter(new_value, prev_filtered_value, alpha=0.5):
    """
    Apply a low-pass filter to the new_value.
    :param new_value: The current raw value.
    :param prev_filtered_value: The previously filtered value.
    :param alpha: Smoothing factor, between 0 and 1. A higher alpha gives more weight to the new value.
    :return: Filtered value.
    """
    return alpha * new_value + (1 - alpha) * prev_filtered_value


class AckermannBot:
    """
    An **AckermannBot** class (for a four wheeled, ackermann steering robot) which contains the main
    functions for GPS waypoint navigation of a robot using a compass.
    """
    def __init__(self, robot):
        self.base = Base.from_robot(robot,"intermode-base")

    async def drive(self, robot, linear_command, angular_command):
        """
        This method sets the base's power.
        """
        # Limit both commands to the range of -max to max
        linear_command = max(min(linear_command, LINEAR_COMMAND_MAX), -LINEAR_COMMAND_MAX)
        angular_command = max(min(angular_command, ANGULAR_COMMAND_MAX), -ANGULAR_COMMAND_MAX)
        
        print(f"Linear: {linear_command}, Angular: {angular_command}")

        linearVec = Vector3(x=0.0, y=linear_command, z=0.0)
        angularVec = Vector3(x=0.0, y=0.0, z=angular_command)
        await robot.base.set_power(linearVec, angularVec)

    def calculate_heading(self, lat1, lon1, lat2, lon2):
        """
        Calculates the heading from one GPS coordinate to another.
        """
        # Convert latitude and longitude from degrees to radians
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)

        # Calculate the difference in latitude and longitude
        dlon = lon2 - lon1

        # Calculate the initial bearing
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        initial_bearing = math.atan2(y, x)

        # Convert the initial bearing to degrees
        initial_bearing = math.degrees(initial_bearing)

        # Normalize the initial bearing to the range [0, 360] degrees
        compass_bearing = (initial_bearing + 360) % 360

        return compass_bearing
    
    async def calculate_distance(self, current_lat, current_lon, target_lat, target_lon):
        """
        Calculate the distance between two GPS coordinates.
        """
        current_lat = math.radians(current_lat)
        current_lon = math.radians(current_lon)
        target_lat = math.radians(target_lat)
        target_lon = math.radians(target_lon)

        delta_latitude = target_lat - current_lat
        delta_longitude = target_lon - current_lon

        # Calculate distance using Haversine formula
        a = (math.sin(delta_latitude/2)**2) + math.cos(current_lat) * math.cos(target_lat) * (math.sin(delta_longitude/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = 6371 * c  # Earth's radius in kilometers
        return distance

    async def calculate_steering_angle(self, robot, current_heading, current_lat, current_lon, target_lat, target_lon):
        """
        Calculate the steering angle required to orient the robot towards the target position.
        """
        target_bearing = robot.calculate_heading(current_lat, current_lon, target_lat, target_lon)
        steering_angle = target_bearing - current_heading
        steering_angle *= -1  # Flip because heading is positive in the opposite direction to steering

        # Normalize steering angle to -180 to 180
        if steering_angle > 180:
            steering_angle -= 360
        elif steering_angle < -180:
            steering_angle += 360

        # Offset steering angle to account for homing error
        steering_angle += STEERING_ANGLE_OFFSET
        # Limit angle to maximum steering angle
        steering_angle = max(-STEER_ANGLE_MAX, min(STEER_ANGLE_MAX, steering_angle))

        print(f"Target Heading: {target_bearing:07.3f}, Current Heading: {current_heading:07.3f}, Steering Angle: {steering_angle:07.3f}")

        return steering_angle
    
    async def navigate(self, robot, sensor_motion, sensor_gps_right, sensor_gps_left, latitude_target, longitude_target, coord_log):
        """
        This method is used to enter a control loop and turn the robot (this is implementation uses
        differential steering). This loop will run so that the heading error is constantly being
        measured and corrected. When the heading is reached (within 5 degrees), this control loop
        is broken. There is some commented out logic that will give some additional settling time
        if needed - although this is not neccessary.
        """
        filtered_steering_angle = 0

        while True:
            start_time = time.time()

            # Read in heading
            raw_heading = await sensor_motion.get_compass_heading() + HEADING_OFFEST
            # Adjust heading to be within the desired range (0-360 degrees)
            actual_heading = (raw_heading) % 360

            coords_right = await sensor_gps_right.get_position()
            coords_left = await sensor_gps_left.get_position()
            latitude_current = (coords_right[0].latitude + coords_left[0].latitude)/2
            longitude_current = -0.000005 + (coords_right[0].longitude + coords_left[0].longitude)/2

            gps_distance = await robot.calculate_distance(coords_right[0].latitude, coords_right[0].longitude, coords_left[0].latitude, coords_left[0].longitude)
            print(f"GPS Distance: {gps_distance:09.6f}")
            print(f"Coordinates: {latitude_current}, {longitude_current}, {actual_heading}")
            coord_log.append([latitude_current, longitude_current])

            # Determine steering angle needed to reach target and convert to a decimal percentage
            # TODO: Add handling for target not being reachable due to minimum turning radius
            # TODO: Improve handling for angles greater than the maximum
            steering_angle_raw = await robot.calculate_steering_angle(robot, actual_heading, latitude_current, longitude_current, latitude_target, longitude_target)
            steering_angle_raw /= STEER_ANGLE_MAX
            steering_angle = low_pass_filter(steering_angle_raw, filtered_steering_angle, STEERING_FILTER_ALPHA)
            filtered_steering_angle = steering_angle

            distance = await robot.calculate_distance(latitude_current, longitude_current, latitude_target, longitude_target)

            print(f"Steer Angle: {steering_angle*STEER_ANGLE_MAX:07.3f}, Distance: {distance:07.3f}")

            # Move to target
            # TODO: Make linear command more dynamic
            await robot.drive(robot, LINEAR_COMMAND, steering_angle)
            
            # Stop spinning once the desired heading is reached
            if abs(distance) < WAYPOINT_TOLERANCE:
                print("Target coordinates reached")
                break

            time.sleep(max(start_time + LOOP_PERIOD - time.time(), 0))

        await robot.base.stop()

        return coord_log



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
            raw_heading = await xsens.get_compass_heading() + HEADING_OFFEST
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



    async def goto_coord(self, boxbot, pid_angular, pid_linear, xsens, gps,latD,longD,data):
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

            raw_heading = await xsens.get_compass_heading() + HEADING_OFFEST

            # Adjust heading to be within the desired range (0-360 degrees)
            actual_heading = (raw_heading) % 360

            print(f"Coordinates: {latitude}, {longitude}, {actual_heading}")

            # Angle wraparound logic
            if abs(desired_heading - actual_heading) > 180:
                if desired_heading > actual_heading:
                    actual_heading += 360
                else:
                    desired_heading += 360

            #calculate the control output for heading compensation and set forward power
            control_output_angular = pid_angular.calculate(desired_heading, actual_heading)
            print(f"Distance: {distance}")
            control_output_linear = pid_linear.calculate(0, distance*-1000)
            await boxbot.drive(control_output_linear, control_output_angular)

            if distance < WAYPOINT_TOLERANCE:
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

    async def gotopoint(self, boxbot,gps,pid_heading, pid_target, pid_linear,xsens,latD,longD,data):
        """
        This calls setheading and gotocoord together, such that the robot can be requested to turn
        and go to the correct GPS point.
        """
        try:
            await boxbot.setheading(boxbot, pid_heading, xsens, gps, latD, longD)
            await boxbot.goto_coord(boxbot, pid_target, pid_linear, xsens, gps, latD, longD,data)
            await boxbot.base.stop()

        except:
            await boxbot.base.stop()

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
        self.pid_log = np.zeros(1,4)

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
        self.pid_log.append([time.time(),p_component,i_component,d_component])
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
    def print_Log(self, name):
        with open(name, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Write the header
            writer.writerow(['Time', 'P', 'I', 'D'])
            for i in self.pid_log:
                writer.writerow(i)
            
