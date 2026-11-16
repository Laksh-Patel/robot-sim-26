from sr.robot3 import *
from numpy import median

# here is our code

robot = Robot()

MOTOR_BOARD_NAME = 'SR0TEA'
left_motor = robot.motor_boards[MOTOR_BOARD_NAME].motors[0]
right_motor = robot.motor_boards[MOTOR_BOARD_NAME].motors[1]

left_motor_correction = 0.0
right_motor_correction = -0.001
motor_constant = 0.32

def stop_robot(pause_time = 1):
    left_motor.power = 0
    right_motor.power = 0
    robot.sleep(pause_time)


while True:
    if initial_power > 0:
        left_motor.power = max((initial_power + left_motor_correction), 0)
        right_motor.power = max((initial_power + right_motor_correction), 0)
    else:
        left_motor.power = min((initial_power - left_motor_correction), 0)
        right_motor.power = min((initial_power - right_motor_correction), 0)
    
    robot.arduino.pins[front_pins[0]].mode = OUTPUT
    robot.arduino.pins[front_pins[1]].mode = INPUT

    reading = median([robot.arduino.ultrasound_measure(*front_pins) for _ in range(3)])
    if reading <= 100:
        break

stop_robot()