#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_C, OUTPUT_D, SpeedPercent
from ev3dev2.sensor.lego import TouchSensor
#import numpy as np
import math
import sys
from time import sleep


# Lengths of the links
link1 = 16.8  # length of link 1 in cm
link2 = 10  # length of link 2 in cm

# Initialize motors and sensors
motor1 = LargeMotor(OUTPUT_C)
motor2 = LargeMotor(OUTPUT_D)
touch_sensor = TouchSensor()


# Function to implement foward kinematics
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


#  Function to move robot arm given link angles
def move_to_angles(theta1, theta2):
    """
    Moves the robot arm to the specified joint angles.
    """
    motor1.on_for_degrees(SpeedPercent(5), theta1)
    sleep(2)
    motor2.on_for_degrees(SpeedPercent(5), theta2)


#  Function to measure distance between 2 points
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


#  Function to measure angle between 3 points
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


#  Analytic method to implement inverse kinematics
def inverse_kinematics_analytic(x, y):
    """
    Given an (x, y) position in cm, this function calculates the joint angles required
    to reach that position.
    """
    # Calculate the distance from the origin to the point
    r = math.sqrt(x**2 + y**2)
    
    # Check if the target is reachable, otherwise return current position
    if r > link1 + link2:
        print("Target is not reachable")
        return motor1.position, motor2.position

    # Calculate joint angles using inverse kinematics formulas
    cos_theta2 = (r**2 - link1**2 - link2**2) / (2 * link1 * link2)
    
    # Two possible solutions for theta2 (elbow up and elbow down)
    theta2_up = math.acos(cos_theta2)
    theta2_down = -math.acos(cos_theta2)

    # Calculate theta1 for both solutions
    theta1_up = math.atan2(y, x) - math.atan2(link2 * math.sin(theta2_up), link1 + link2 * math.cos(theta2_up))
    theta1_down = math.atan2(y, x) - math.atan2(link2 * math.sin(theta2_down), link1 + link2 * math.cos(theta2_down))

    # Convert joint angles from radians to degrees
    theta1_up = math.degrees(theta1_up)
    theta2_up = math.degrees(theta2_up)
    theta1_down = math.degrees(theta1_down)
    theta2_down = math.degrees(theta2_down)

    # Choose solution based on smallest total joint angle
    if abs(theta1_up) + abs(theta2_up) <= abs(theta1_down) + abs(theta2_down):
        return theta1_up, theta2_up
    else:
        return theta1_down, theta2_down


#  Helper function for analytic inverse kinematics
def move_to_position_analytic(x, y):
    """
    Moves the robot arm to the specified (x, y) position.
    """
    theta1, theta2 = inverse_kinematics_analytic(x, y)
    move_to_angles(theta1, theta2)


#  Function to implement Jacobian matrix
def jacobian(theta1, theta2):
    # Convert angles from degrees to radians
    theta1_rad = math.radians(theta1)
    theta2_rad = math.radians(theta2)

    # Calculate the elements of the Jacobian matrix
    J11 = -link1 * math.sin(theta1_rad) - link2 * math.sin(theta1_rad + theta2_rad)
    J12 = -link2 * math.sin(theta1_rad + theta2_rad)
    J21 = link1 * math.cos(theta1_rad) + link2 * math.cos(theta1_rad + theta2_rad)
    J22 = link2 * math.cos(theta1_rad + theta2_rad)

    return [[J11, J12], [J21, J22]]


#  Function to implement numerical method for inverse kinematics
def inverse_kinematics_newton(target_x, target_y, initial_theta1, initial_theta2):
    # Set a tolerance level
    tolerance = 0.0001

    # Initialize thetas
    theta1 = initial_theta1
    theta2 = initial_theta2
    #print(theta1, theta2, file=sys.stderr)
    while True:
        current_x, current_y = forward_kinematics(theta1, theta2)
        error = [target_x - current_x, target_y - current_y]
        #print(error, file=sys.stderr)
        # If the error is within the tolerance level, break the loop
        if math.sqrt(error[0] ** 2 + error[1] ** 2) < tolerance:
            break

        # Calculate the Jacobian
        J = jacobian(theta1, theta2)
        #print(J, file=sys.stderr)
        # Calculate the determinant of the Jacobian
        detJ = J[0][0] * J[1][1] - J[0][1] * J[1][0]
        #print(detJ, file=sys.stderr)
        # Avoid division by zero
        if abs(detJ) < 1e-6:
            print("Singularity reached. Unable to calculate inverse.")
            break

        # Calculate the pseudo-inverse of the Jacobian
        J_inv = [[J[1][1] / detJ, -J[0][1] / detJ], [-J[1][0] / detJ, J[0][0] / detJ]]

        # Calculate change in thetas
        delta_theta = [J_inv[0][0] * error[0] + J_inv[0][1] * error[1], J_inv[1][0] * error[0] + J_inv[1][1] * error[1]]
        #print(delta_theta, file=sys.stderr)
        # Update thetas
        theta1 += delta_theta[0]
        theta2 += delta_theta[1]
        #print(theta1, theta2, file=sys.stderr)

    return theta1, theta2


#  Helper function for Newtonian method for inverse kinematics
def move_to_position_newton(x, y):
    # Use your initial joint angles here
    initial_theta1 = 1
    initial_theta2 = 1

    theta1, theta2 = inverse_kinematics_newton(x, y, initial_theta1, initial_theta2)
    #print(theta1, theta2, file=sys.stderr)


    # Convert joint angles to motor commands and execute them
    move_to_angles(theta1, theta2)


'''def broydens_method(target_x, target_y, initial_theta1, initial_theta2):
    # Set a tolerance level
    tolerance = 0.01

    # Initialize thetas
    theta = np.array([initial_theta1, initial_theta2])

    # Initialize Jacobian approximation (identity matrix)
    J_inv = np.eye(2)

    while True:
        current_x, current_y = forward_kinematics(theta[0], theta[1])
        error = np.array([target_x - current_x, target_y - current_y])

        # If the error is within the tolerance level, break the loop
        if np.linalg.norm(error) < tolerance:
            break

        # Calculate change in thetas
        delta_theta = J_inv @ error

        # Update thetas
        theta_new = theta + delta_theta

        # Calculate change in end effector position
        delta_x, delta_y = forward_kinematics(theta_new[0], theta_new[1]) - forward_kinematics(theta[0], theta[1])
        delta_f = np.array([delta_x, delta_y])

        # Update Jacobian approximation
        y = delta_f - J_inv @ error
        J_inv += np.outer(delta_theta - J_inv @ y, delta_theta) / (delta_theta @ y)

        theta = theta_new

    return theta[0], theta[1]


def move_to_position_broyden(x, y):
    # Use your initial joint angles here
    initial_theta1 = 0
    initial_theta2 = 0

    theta1, theta2 = broydens_method(x, y, initial_theta1, initial_theta2)

    # Convert joint angles to motor commands and execute them
    move_to_angles(theta1, theta2)'''


#  Function to move to midpoint of 2 recorded points
def measure_midpoint():
    """
    Measures the midpoint between two points in the robot's workspace.
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

    # Calculate the midpoint between the points
    mid_x = (x1 + x2) / 2
    mid_y = (y1 + y2) / 2
    print('Midpoint calculated: (' + str(mid_x) + ',' + str(mid_y) + ') centimeters', file=sys.stderr)

    # Move to origin
    sleep(8)
    # Move to the calculated midpoint
    move_to_position_newton(mid_x, mid_y)


def main():
    # Test forward kinematics function
    theta1 = 20  # angle of joint 1 in degrees
    theta2 = 35  # angle of joint 2 in degrees
    target_x = 10.4
    target_y = 22.6

    motor1.reset()
    motor2.reset()

    x, y = forward_kinematics(motor1.position, motor2.position)
    print("The end effector is at position ( " + str(round(x,1)) + ',' + str(round(y,1)) + ")", file=sys.stderr)

    move_to_angles(theta1, theta2)
    x, y = forward_kinematics(motor1.position, motor2.position)
    print("The end effector is at position ( " + str(round(x,1)) + ',' + str(round(y,1)) + ")", file=sys.stderr)

    error = math.sqrt((x - target_x)**2 + (y - target_y)**2)
    print("The Euclidean distance error between target and actual positions is: " + str(round(error,2)) + " cm", file=sys.stderr)

    #measure_distance()
    #measure_angle()
    #move_to_position_newton(18, -15.7)
    #move_to_position_newton(8, 18)
    #measure_midpoint()
    #x, y = forward_kinematics(motor1.position, motor2.position)
    #print("The end effector is at position ( " + str(round(x,1)) + ',' + str(round(y,1)) + ")", file=sys.stderr)

if __name__ == "__main__":
    main()
