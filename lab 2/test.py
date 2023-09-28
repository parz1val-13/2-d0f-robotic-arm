from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent
from ev3dev2.sensor.lego import TouchSensor
import math

# Define the lengths of your links
link1 = 10  # length of link 1 in cm
link2 = 10  # length of link 2 in cm

# Initialize motors and sensors
motor1 = LargeMotor(OUTPUT_A)
motor2 = LargeMotor(OUTPUT_B)
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

    return x, y

def move_to_angles(theta1, theta2):
    """
    Moves the robot arm to the specified joint angles.
    """
    motor1.on_to_position(SpeedPercent(50), theta1)
    motor2.on_to_position(SpeedPercent(50), theta2)

def measure_distance():
    """
    Measures the distance between two points in the robot's workspace.
    """
    print('Move to first point and press touch sensor')
    while not touch_sensor.is_pressed:
        pass  # wait for touch sensor press

    # Record first point
    x1, y1 = forward_kinematics(motor1.position, motor2.position)

    print('Move to second point and press touch sensor')
    while not touch_sensor.is_pressed:
        pass  # wait for touch sensor press

    # Record second point
    x2, y2 = forward_kinematics(motor1.position, motor2.position)

    # Calculate and return distance between points
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    print(f'The distance is {distance} centimeters')

def measure_angle():
    """
    Measures the angle between two lines in the robot's workspace.
    """
    
    print('Move to first point and press touch sensor')
    
    while not touch_sensor.is_pressed:
        pass  # wait for touch sensor press

    # Record first point
    x0, y0 = forward_kinematics(motor1.position, motor2.position)

    print('Move to second point and press touch sensor')
    
    while not touch_sensor.is_pressed:
        pass  # wait for touch sensor press

    # Record second point
    x1, y1 = forward_kinematics(motor1.position, motor2.position)

    print('Move to third point and press touch sensor')
    
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
     
    print(f'The angle is {angle_deg} degrees')


def main():
    # Test forward kinematics function
    theta1 = 45  # angle of joint 1 in degrees
    theta2 = 45  # angle of joint 2 in degrees

    move_to_angles(theta1, theta2)
    x, y = forward_kinematics(motor1.position, motor2.position)
    print(f"The end effector is at position ({x}, {y})")

    measure_distance()
    measure_angle()

if __name__ == "__main__":
    main()
