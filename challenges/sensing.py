from sr.robot3 import *
from numpy import median

robot = Robot()

MOTOR_BOARD_NAME = 'SR0TEA'
left_motor = robot.motor_boards[MOTOR_BOARD_NAME].motors[0]
right_motor = robot.motor_boards[MOTOR_BOARD_NAME].motors[1]

left_motor_correction = -0.02
right_motor_correction = 0.0
motor_constant = 0.40

def stop_robot(pause_time = 1):
    left_motor.power = 0
    right_motor.power = 0
    robot.sleep(pause_time)

initial_power = motor_constant

while True:

    left_motor.power = max((initial_power + left_motor_correction), 0)
    right_motor.power = max((initial_power + right_motor_correction), 0)
    
    robot.arduino.pins[9].mode = OUTPUT
    robot.arduino.pins[10].mode = INPUT

    reading = median([robot.arduino.ultrasound_measure(9,10) for _ in range(3)])
    print(reading)
    if reading <= 500 and reading > 0:
        break

stop_robot()