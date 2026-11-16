print("STATUS >> RUNNING ROBOT.py")

import sys
import os
import traceback

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
sys.path.append(project_root)

# from src import main

from src.main import LeBotJames


def main():

    bot = ChalkToss()

    error_count = 0
    while True:

        # finished = bot.tick()
        # if finished:
        #     return bot
        
        try:
            finished = bot.tick()
            if finished:
                return bot

        except Exception as e:
            error_count += 1
            print(f"ERROR !! [x{error_count}] {e}")
            traceback.print_exc()

            if error_count >= 50:
                print("STATUS >> Exceeded error threshold, restarting")
                bot = ChalkToss()

        else:
            # reset count if no error
            error_count = 0

def ChalkToss(existing = None):
    # Called it ChalkToss to pay homage to the GOAT LeBron James' iconic pre-match ritual of tossing chalf into the air (and also because it sounds cool ngl)
    # Above comment is written by copilot btw

    bot = LeBotJames(existing)

    from src.drivers.motor import MotorDriver
    from src.drivers.ultrasound import UltrasoundDriver
    from src.drivers.servo import ArmDriver

    print("STATUS >> initialising drivers...")
    bot.MotorDriver = MotorDriver(bot.bot)
    bot.UltrasoundDriver = UltrasoundDriver(bot.bot)
    bot.ArmDriver = ArmDriver(bot.bot)

    print("STATUS >> drivers initialised!")
    
    if bot.position == None:
        print("STATUS >> calculating initial position...")
        bot.update_position(0) # Initial position calculation (if not pre-loaded in the main.py file), called with dt=0 since it's the start
    return bot


print("STATUS >> attempting to run main...")
bot = main()


# from sr.robot3 import *

# bot = Robot()
# motor_board = bot.motor_boards['SR0TEA']
# left_motor = motor_board.motors[0]
# right_motor = motor_board.motors[1]

# left_multiplier = -1
# right_multiplier = 0.90

# power = 0.40

# left_motor.power = left_multiplier * power
# right_motor.power = -right_multiplier * power

# bot.sleep(8)