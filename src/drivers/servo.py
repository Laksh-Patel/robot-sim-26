class ServoDriver:
    def __init__(self, bot, params_dict):
        """
        Initialises each servo driver. The state 0 is the initial position, and state 1 is the alternate position.
        """
        self.bot = bot
        self.arm_motor_board = bot.motor_boards['SR0KG17'] # TODO fill in the correct motor board name
        self.params_dict = params_dict
        
        self.arm_motor = self.arm_motor_board.motors[self.params_dict['motor_number']]
        self.revert_initial_time = self.params_dict['revert_initial_time']
        self.revert_initial_speed = self.params_dict['revert_initial_speed']
        self.move_alternate_time = self.params_dict['move_alternate_time']
        self.move_alternate_speed = self.params_dict['move_alternate_speed']
        self.state = self.params_dict['state']
    
    def initial(self):
        if self.state != 0:
            self.arm_motor.power = self.revert_initial_speed
            self.bot.sleep(self.revert_initial_time)
            self.state = 0
            self.arm_motor.power = 0
     
    def alternate(self):
        if self.state != 1:    
            self.arm_motor.power = self.move_alternate_speed
            self.bot.sleep(self.move_alternate_time)
            self.state = 1
            self.arm_motor.power = 0
        
class ArmDriver:
    def __init__(self, bot):
        """
        Hello
        """
        self.bot = bot
        self.arm_motor_board = bot.motor_boards['SR0KG17'] # TODO fill in the correct motor board name
        self.BottomArm = ServoDriver(bot, {
            'motor_number': 1,
            'revert_initial_time': 0.57,
            'revert_initial_speed': -0.18,
            'move_alternate_time': 0.65,
            'move_alternate_speed': 0.25,
            'state': 0
        })
        self.TopArm = ServoDriver(bot, {
            'motor_number': 0,
            'revert_initial_time': 0.4,
            'revert_initial_speed': 0.35,
            'move_alternate_time': 0.45,
            'move_alternate_speed': -0.35,
            'state': 0
        })
