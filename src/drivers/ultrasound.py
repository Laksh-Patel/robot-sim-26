from sr.robot3 import OUTPUT, INPUT
from numpy import median

class UltrasoundDriver:
        def __init__(self, bot):
            self.bot = bot
            self.arduino = bot.arduino
        
            if bot.is_simulated:
                self.front_pins = (2,3)
            else:
                self.front_pins = (9,10)

            self.arduino.pins[self.front_pins[0]].mode = OUTPUT
            self.arduino.pins[self.front_pins[1]].mode = INPUT
            # Initialises the pins to be input and output

        def read(self, sensor_num=1): # front by default
            """
            Reads the distance from a specified sensor (1-4)
            1 - front, 2 - left, 3 - right, 4 - back
            """
            pins = self.front_pins
            reading = median([self.arduino.ultrasound_measure(*pins) for _ in range(3)])
            return reading
            # Returns the median of 3 readings to reduce noise