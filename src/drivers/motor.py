from numpy import sign

from sr.robot3 import *
from src.utils import *
from src.constants import *

class MotorDriver:
    """Controls motors of robot for movement"""

    def __init__(self, bot):

        self.bot = bot
        self.left_power = 0
        self.right_power = 0
        
        if bot.is_simulated:
            self.motor_board = bot.motor_board
            self.left_multiplier = 1 * 0.82
            self.right_multiplier = 1.0 * 0.82
        else:
            self.motor_board = bot.motor_boards[MOTORS.MOTOR_BOARD_NAME]
            self.left_multiplier = 1 * (-1) # Wiring is reversed
            self.right_multiplier = 0.9
        self.last_uL = 0.0
        self.last_uR = 0.0

    def power(self, left, right=None):

        if right is None:
            right = left

        if abs(right) > 1:
            right = sign(right)
        if abs(left) > 1:
            left = sign(left)

        # print("updated MOTOR POWER VALUE:", left, right)

        # TODO Check if the lines below are the exact same thing
        self.motor_board.motors[0].power = self.left_multiplier * left
        self.motor_board.motors[1].power = self.right_multiplier * right

        # if left == right and left == 0:
        #     self.bot.localiser.update_zero_velocity()

        self.left_power = self.left_multiplier * left
        self.right_power = self.right_multiplier * right
        
        # Latest motor powers for Kalman filter without correction multiplier
        self.last_uL = left
        self.last_uR = right

    def read_powers(self):
        return f'left: {self.motor_board.motors[0].power}, right: {self.motor_board.motors[1].power}'
    
    def get_powers(self):
        return self.last_uL, self.last_uR
