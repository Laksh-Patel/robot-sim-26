"""    
- Angle sensing. Perform the following steps:
    - Start with your robot centred on the wall marker and between 1 and 3 metres from the wall.
    - Turn your robot to the left, to the centre, then to the right, then reverse this.
    - During this, illuminate the LEDs based on the following conditions:
        - LED A in blue when the marker is more than 15° left from square on.
        - LED B in red when the marker is approximately square on (within a 30° arc).
        - LED C in blue when the marker is more than 15° right from square on.

The robot may move autonomously or may be moved manually to complete this challenge.

Note: if moving the robot manually then the Arduino, Motor and Servo Boards must be disconnected from the Power Board as
well as any mechanical components secured for the duration of the demonstration.
"""

from sr.robot3 import Robot, LED_A, LED_B, LED_C, Colour
from numpy import pi

robot = Robot()

markers = robot.camera.see()

while not markers:

    robot.kch.leds[LED_B].colour = Colour.YELLOW

    markers = robot.camera.see()

robot.kch.leds[LED_B].colour = Colour.OFF

marker = markers[0]

while True:

    markers = robot.camera.see()

    if not markers:
        continue

    marker = markers[0]

    if abs(marker.position.horizontal_angle) <= (3.14159 / 12):

        robot.kch.leds[LED_A].colour = Colour.OFF
        robot.kch.leds[LED_B].colour = Colour.RED
        robot.kch.leds[LED_C].colour = Colour.OFF

    elif marker.position.horizontal_angle < -(3.14159 / 12):

        robot.kch.leds[LED_A].colour = Colour.BLUE
        robot.kch.leds[LED_B].colour = Colour.OFF
        robot.kch.leds[LED_C].colour = Colour.OFF

    elif marker.position.horizontal_angle > (3.14159 / 12):

        robot.kch.leds[LED_A].colour = Colour.OFF
        robot.kch.leds[LED_B].colour = Colour.OFF
        robot.kch.leds[LED_C].colour = Colour.BLUE