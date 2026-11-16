"""
- Distance sensing. Perform the folflowing steps:
    - Start with your robot centred on the wall marker about 2000 mm from the wall.
    - Move towards the wall until you are less than 100 mm from the wall, then move back to 2000 mm
    - During this, illuminate the LEDs based on the following conditions:
        - LED B in red when the robot is more than 1500 mm from the wall.
        - LED B in blue when the robot is between 200 mm and 1500 mm from the wall.
        - LED B in green when the robot is less than 200 mm from the wall.
"""

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
from numpy import median
# here is our code

robot = Robot()

# MOTOR_BOARD_NAME = 'SR0TEA'
# left_motor = robot.motor_boards[MOTOR_BOARD_NAME].motors[0]
# right_motor = robot.motor_boards[MOTOR_BOARD_NAME].motors[1]

# left_motor_correction = 0.0
# right_motor_correction = -0.001
# motor_constant = 0.32

trigger_pin = 9
echo_pin = 10

front_pins = (9,10)

robot.arduino.pins[front_pins[0]].mode = OUTPUT
robot.arduino.pins[front_pins[1]].mode = INPUT

reading = median([robot.arduino.ultrasound_measure(*front_pins) for _ in range(3)])
print(reading)

while True:
    reading = median([robot.arduino.ultrasound_measure(*front_pins) for _ in range(3)])
    if reading <= 200:
        robot.kch.leds[LED_B].colour = Colour.BLUE
    else:
        robot.kch.leds[LED_B].colour = Colour.RED
    