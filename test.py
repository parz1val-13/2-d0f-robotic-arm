#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_C, OUTPUT_D, SpeedPercent
from ev3dev2.sensor.lego import TouchSensor
import math
import sys
from time import sleep

# Define the lengths of your links
link1 = 16.8  # length of link 1 in cm
link2 = 10  # length of link 2 in cm

# Initialize motors and sensors
motor1 = LargeMotor(OUTPUT_C)
motor2 = LargeMotor(OUTPUT_D)
touch_sensor = TouchSensor()

def forward_kinematics(theta1, theta2):
    """
    Given two angles (in degrees), this function calculates the (x,y) position
    of the end effector (tip of the second link).
    """
    # Convert angles from degrees to radians
    theta1_rad = math.radians(theta1)
    theta2_rad = math.radians(theta2)

    # Calculate position of end effector
    x = link1 * math.cos(theta1_rad) + link2 * math.cos(theta1_rad + theta2_rad)
    y = link1 * math.sin(theta1_rad) + link2 * math.sin(theta1_rad + theta2_rad)

    '''# Offset correction for x-coordinate
    x_offset = 26.0  # Adjust this value based on your calibration
    x = x - x_offset'''

    return x, y

def move_to_angles(theta1, theta2):
    """
    Moves the robot arm to the specified joint angles.
    """
    motor1.on_for_degrees(SpeedPercent(5), theta1)
    sleep(2)
    motor2.on_for_degrees(SpeedPercent(5), theta2)

def measure_distance():
    """
    Measures the distance between two points in the robot's workspace.
    """
    print('Move to first point and press touch sensor', file=sys.stderr)
    while not touch_sensor.is_pressed:
        pass  # wait for touch sensor press

    # Record first point
    x1, y1 = forward_kinematics(motor1.position, motor2.position)
    print('First point recorded: (' + str(x1) + ',' + str(y1) + ') centimeters', file=sys.stderr)

    # Record second point
    print('Move to second point and press touch sensor', file=sys.stderr)
    sleep(5)
    while not touch_sensor.is_pressed:
        pass  # wait for touch sensor press

    # Record second point
    x2, y2 = forward_kinematics(motor1.position, motor2.position)
    print('Second point recorded: (' + str(x2) + ',' + str(y2) + ') centimeters', file=sys.stderr)

    # Calculate and return distance between points
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    print('The distance is' + str(round(distance,3)) + ' centimeters', file=sys.stderr)


def measure_angle():
    """
    Measures the angle between two lines in the robot's workspace.
    """

    print('Move to first point and press touch sensor', file=sys.stderr)

    while not touch_sensor.is_pressed:
        pass  # wait for touch sensor press

    # Record first point
    x0, y0 = forward_kinematics(motor1.position, motor2.position)

    print('Move to second point and press touch sensor', file=sys.stderr)
    sleep(5)
    while not touch_sensor.is_pressed:
        pass  # wait for touch sensor press

    # Record second point
    x1, y1 = forward_kinematics(motor1.position, motor2.position)

    print('Move to third point and press touch sensor', file=sys.stderr)
    sleep(5)
    while not touch_sensor.is_pressed:
        pass  # wait for touch sensor press

    # Record third point
    x2, y2 = forward_kinematics(motor1.position, motor2.position)

    # Calculate vectors
    vector_0_to_1 = [x1 - x0, y1 - y0]
    vector_0_to_2 = [x2 - x0, y2 - y0]

    # Calculate angle between vectors using dot product formula
    dot_product = vector_0_to_1[0]*vector_0_to_2[0] + vector_0_to_1[1]*vector_0_to_2[1]
    magnitude_product = math.sqrt(vector_0_to_1[0]**2 + vector_0_to_1[1]**2) * math.sqrt(vector_0_to_2[0]**2 + vector_0_to_2[1]**2)

    angle_rad = math.acos(dot_product / magnitude_product)

    angle_deg = math.degrees(angle_rad)

    print('The angle is ' + str(angle_deg) +  ' degrees', file=sys.stderr)


def main():
    # Test forward kinematics function
    theta1 = 45  # angle of joint 1 in degrees
    theta2 = 45  # angle of joint 2 in degrees
    target_x = 10.4
    target_y = 22.6

    motor1.reset()
    motor2.reset()

    x, y = forward_kinematics(motor1.position, motor2.position)
    print("The end effector is at position ( " + str(round(x,1)) + ',' + str(round(y,1)) + ")", file=sys.stderr)

    '''move_to_angles(theta1, theta2)
    x, y = forward_kinematics(motor1.position, motor2.position)
    print("The end effector is at position ( " + str(round(x,1)) + ',' + str(round(y,1)) + ")", file=sys.stderr)

    error = math.sqrt((x - target_x)**2 + (y - target_y)**2)
    print("The Euclidean distance error between target and actual positions is: " + str(round(error,2)) + " cm", file=sys.stderr)'''

    #measure_distance()
    measure_angle()

if __name__ == "__main__":
    main()
