"""
This challenge is designed to test the robot’s ability to perform repeated movements accurately.

This is a key feature for a robot as, due to manufacturing tolerances, no two motors are exactly the same. As such, your robot must account for the differences in the motors to move in a straight line. This is often achieved by adjusting the power of the motors or, in some cases, by using external references.

To complete this challenge, your robot must:

Autonomously complete 3 continuous circuits of a triangular path, returning its starting position to within 300 mm.
The path must be an isosceles right-angled triangle with shorter side length of 1500±200 mm.
The direction of travel around the path and orientation of the robot are inconsequential.
Teams are encouraged to include in their submission video objects which establish the scale of the path traversed by the robot, for example a metre ruler.
"""

from sr.robot3 import *
# here is our code

robot = Robot()

MOTOR_BOARD_NAME = 'SR0TEA'
left_motor = robot.motor_boards[MOTOR_BOARD_NAME].motors[0]
right_motor = robot.motor_boards[MOTOR_BOARD_NAME].motors[1]

left_motor_correction = 0.0
right_motor_correction = -0.001
motor_constant = 0.32

left_90_time = 0.75

def stop_robot(pause_time = 1):
    left_motor.power = 0
    right_motor.power = 0
    robot.sleep(pause_time)

def drive_straight(time, initial_power = motor_constant):
    if initial_power > 0:
        left_motor.power = max((initial_power + left_motor_correction), 0)
        right_motor.power = max((initial_power + right_motor_correction), 0)
    else:
        left_motor.power = min((initial_power - left_motor_correction), 0)
        right_motor.power = min((initial_power - right_motor_correction), 0)
    robot.sleep(time)
    stop_robot()


def turn_left(time, power = motor_constant):
    left_motor.power = -power + left_motor_correction
    right_motor.power = power + right_motor_correction
    robot.sleep(time)
    stop_robot()

# Change all these times completely as required

drive_straight(4.51)
turn_left(0.77)

drive_straight(4.13)
turn_left(1.13)

drive_straight(6.05)
turn_left(1.16)

###
drive_straight(4.48)
turn_left(0.77)

drive_straight(4.13)
turn_left(1.10)

drive_straight(6.05)
turn_left(1.13)

###
drive_straight(4.48)
turn_left(0.75)

drive_straight(4.13)
turn_left(1.10)

drive_straight(6.05)
# turn_left(1.00)