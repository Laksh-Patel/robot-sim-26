from abc import ABC, abstractmethod
import numpy as np
from sr.robot3 import OUT_H0

from src.utils import *

from src.constants import *

class Process(ABC):
    """Abstract base class for robot processes."""
    
    @abstractmethod
    def __init__(self, bot, *args, **kwargs):
        """Initialise the process"""
        self.bot = bot
        
    @abstractmethod
    def tick(self):
        """Run the process. Called every tick."""
        raise NotImplementedError()
        
    @abstractmethod
    def done(self):
        """Check if process should be killed"""
        raise NotImplementedError()
    
    @abstractmethod
    def complete(self):
        """finish the process before killing it"""
        raise NotImplementedError()
    
class Move(Process):
    """Takes in the bot, and the marker_id to drive to, and whether we are picking it up (defaults to True)"""
    def __init__(self, bot, marker_id, separation = 400):
        self.bot = bot
        self.marker_id = marker_id
        self.separation = separation

        self.seen_once = False
        self.missing_iterations = 0

        self.target = self.bot.objects.get(self.marker_id, None)
        self.completed = False
        
    def tick(self):
        
        self.target = self.bot.objects.get(self.marker_id, None)

        if self.target:
            self.missing_iterations = 0
            self.seen_once = True

            self.motor_power = calculate_motor_power(angle=self.target.position.horizontal_angle, distance=self.target.position.distance - self.separation)
            self.bot.MotorDriver.power(*self.motor_power)

            self.bot.tried_to_move = True
            
        else:
            print("!!! CANNOT SEE TARGET IN MOVE!!!")
            self.missing_iterations += 1

            self.bot.tried_to_move = False

    def done(self):
        
        if self.missing_iterations == 0:

            dist = self.target.position.distance
                
            if dist <= self.separation or (self.motor_power[0] <= 0.1 and self.motor_power[1] <= 0.1) or min(self.motor_power) <= 0.05:
                self.completed = True
                return True
        
        else:

            if (self.seen_once == False and self.missing_iterations >= 2) or (self.missing_iterations >= 4):
                return True
            
        return False
    
    def complete(self):
        self.bot.MotorDriver.power(0)
        return self.completed
            
class Align(Process):
    
    def __init__(self, bot, marker_id):
        self.bot = bot
        self.marker_id = marker_id

    def tick(self):
        
        if not isinstance(self.bot.objects, dict):
            self.bot.update_objects()
        self.target = self.bot.objects.get(self.marker_id, None)
        
        if self.target:
            
            constant_power = 0.24 #was 0.27
            if self.target.position.horizontal_angle > 0:
                angle_sign = 1
                motor_power = (constant_power, -constant_power)
            else:
                angle_sign = -1
                motor_power = (-constant_power, constant_power)
            
            self.bot.MotorDriver.power(*motor_power)
            
            while self.target and angle_sign*self.target.position.horizontal_angle > CAMERA.ACCEPTANCE_RANGE:
                self.bot.update_objects()
                # self.bot.update_position() TODO check if we want to
                self.target = self.bot.objects.get(self.marker_id, None)
            if not self.target:
                print('stopped seeing target during align')
            self.bot.MotorDriver.power(0)
            self.bot.tried_to_move = True
            
        else:
            print("!!! NEVER SAW TARGET IN ALIGN !!!")
            self.bot.tried_to_move = False


    def done(self):
        return True

    def complete(self):
        self.bot.MotorDriver.power(0)
        if self.marker_id in self.bot.objects:
            dist = self.bot.objects[self.marker_id].position.distance
        else:
            dist = None
        
        self.bot.complete_ret = dist
        return

class FindMarker(Process):
    def __init__(self, bot, marker_ids=[], align=True, direction_to_turn=None, ret_all = False, marker_id=None):
        self.bot = bot
        self.marker_ids = marker_ids
        if self.marker_ids == []:
            self.marker_ids = [marker_id] if marker_id is not None else []
        self.complete_ret = None
        self.ret_all = ret_all
        self.align = align
        self.num_ticks = 0
        
        if self.ret_all:
            self.align = False

        if direction_to_turn is None:
            self.direction_to_turn = -1 # defaults left
            if self.bot.position[0] is not None and self.bot.position[1] is not None:
                angle = calculate_angle_to_coord(target_coords=(0, 0), position_vector=self.bot.position)
                if not np.isnan(angle):
                    self.direction_to_turn = 1 if angle > 0 else -1
        else: self.direction_to_turn = direction_to_turn
    
        # Perform the check at the start in case we can already see the markers we want to see
        self.is_complete = self.done()            

    def tick(self):
        if self.is_complete:
            return
        sub_process = DumbMove(self.bot, theta=self.direction_to_turn * 0.2, motor_power=0.2)
        sub_process.tick()
        sub_process.complete()
        self.num_ticks += 1
        self.bot.tried_to_move = True
        self.bot.bot.sleep(0.5)
        
        # motor_power = 0.2
        # self.bot.MotorDriver.power(self.direction_to_turn * motor_power, -self.direction_to_turn * motor_power)
        
        # while self.check() == False:
        #     pass
        
        # self.bot.MotorDriver.power(0)
        
    def check(self):
        
        self.bot.update_objects()

        """Proposed AV change: use a set intersection to check if any of the marker ids we are looking for are in the objects we can see, instead of checking each one individually in a loop. From there determine if we want to return all the marker_ids we can see of just one. Particularly pertinant when we are looking for multiple box markers or home zone markers at once and might be able to see multiple of them at once. Big issue is also if we are aligning to a marker, but can see multiple of the markers we wanted to find."""
        if self.ret_all:
            intersection_ids = list(set(self.bot.objects.keys()) & (set(self.marker_ids)))
            print('Intersection IDs:', intersection_ids)
            if len(intersection_ids) > 0:
                print('FOUND MARKERS IN FIND MARKER, IDs:', intersection_ids)
                self.bot.MotorDriver.power(0)
                self.is_complete = True
                self.complete_ret = intersection_ids # TODO check if we want to return all of them or just one (e.g. the closest or lowest horizontal angle)
                return True
        else:
            # LP method from before
            print(self.bot.objects.keys())
            for marker_id in self.marker_ids:
                if marker_id in self.bot.objects:
                    self.complete_ret = marker_id
                    self.bot.MotorDriver.power(0)
                    return True
            
        if self.num_ticks >= 20: # TODO check if this threshold is good
            self.complete_ret = None
            self.bot.MotorDriver.power(0)
            return True
            
        return False
    
    def done(self):
        # return True
        return self.check()
    
    def complete(self):
        
        if self.align and self.complete_ret is not None and not self.ret_all:
            self.bot.bot.sleep(0.25)
            sub_process = Align(self.bot, marker_id=self.complete_ret)
            sub_process.tick()
            sub_process.complete()
        
        self.bot.MotorDriver.power(0)
        self.bot.bot.sleep(0.25)
        self.bot.complete_ret = self.complete_ret # TODO check what we are doing with this
        return self.complete_ret
    
class DumbMove(Process):
    def __init__(self, bot, theta=None, dist=None, time=None, motor_power = MOTORS.MOTOR_CONSTANT):
        """
        Can specify angle, distance and time to drive
        Theta in rad, dist in millimetres, time in seconds
        Positive theta means turn right, negative means turn left
        If more than 1 are specified, they are executed in the order of theta, dist, time
        """
        self.bot = bot
        self.dist = dist
        self.theta = theta
        self.time = time
        self.motor_power = motor_power
        
    def tick(self):
        start_time = self.bot.bot.time()
        curr_time = start_time
        prev_time = start_time
        sleep_dt = 0.05
        
        if self.theta is not None:
            start_time = self.bot.bot.time()
            curr_time = start_time
            prev_time = start_time
            
            self.theta = (self.theta + np.pi) % (2*np.pi) - np.pi # wraps to (-pi, pi] if it isn't already
            multiplier = 1 if self.theta > 0 else -1 # 1 for turning right, -1 for turning left
            # if self.theta > 0:
            #     self.theta *= 0.80
            turn_time = calculate_turn_time(self.theta)

            if not turn_time:
                turn_time=0
            if abs(self.theta) <= MOTORS.SMALL_ANGLE_THRESHOLD:
                self.bot.MotorDriver.power(multiplier * MOTORS.SLOW_MOTOR_CONSTANT, multiplier * -MOTORS.SLOW_MOTOR_CONSTANT)
            else:
                self.bot.MotorDriver.power(multiplier * self.motor_power, multiplier * -self.motor_power)
            
            while curr_time - start_time < turn_time:
                self.bot.bot.sleep(sleep_dt)
                curr_time = self.bot.bot.time()
                dt = curr_time - prev_time
                self.bot.update_position(dt) # dt since last update
                prev_time = curr_time

            self.bot.tried_to_move = True

        if self.dist is not None:
            start_time = self.bot.bot.time()
            curr_time = start_time
            prev_time = start_time
            
            multiplier = 1 if self.dist > 0 else -1 # determines forward or backwards
            drive_time = calculate_drive_time(abs(self.dist)) # power not build, could remove power built thing from the util functions
            if not drive_time:
                drive_time = 0
            self.bot.MotorDriver.power(multiplier * self.motor_power)

            while curr_time - start_time < drive_time:
                self.bot.bot.sleep(sleep_dt)
                curr_time = self.bot.bot.time()
                dt = curr_time - prev_time
                self.bot.update_position(dt) # dt since last update
                prev_time = curr_time

            self.bot.tried_to_move = True

        if self.time is not None:
            start_time = self.bot.bot.time()
            curr_time = start_time
            prev_time = start_time
            
            time = abs(time)
            self.bot.MotorDriver.power(self.motor_power)

            while curr_time - start_time < time:
                self.bot.bot.sleep(sleep_dt)
                curr_time = self.bot.bot.time()
                self.bot.update_position(curr_time - prev_time) # dt since last update
                prev_time = curr_time

            self.bot.tried_to_move = True

            
    def done(self):
        return True
    
    def complete(self):
        self.bot.MotorDriver.power(0)
        self.bot.localiser.update_zero_velocity()
        self.bot.bot.sleep(0.25)
        return None

class MoveArm(Process):
    """
    Takes in the bot, the arm to move ('top' or 'bottom'), and the new state to move to (0 or 1) where 0 is the initial state and 1 is the alternate state
    """
    def __init__(self, bot, arm, new_state):
        self.bot = bot
        self.arm = arm
        self.new_state = new_state
    
    def tick(self):
        if self.arm == 'bottom':
            if self.new_state == 1:
                self.bot.ArmDriver.BottomArm.alternate()
            if self.new_state == 0:
                self.bot.ArmDriver.BottomArm.initial()
        elif self.arm == 'top':
            if self.new_state == 1:
                self.bot.ArmDriver.TopArm.alternate()
            if self.new_state == 0:
                self.bot.ArmDriver.TopArm.initial()

        self.bot.tried_to_move = False      
    
    def done(self):
        return True
    
    def complete(self):
        self.bot.bot.sleep(0.5)
        return None

class SimPickPlace(Process):
    def __init__(self, bot, pick=True):
        self.bot = bot
        self.pick = pick

        self.bot.bot.servo_board.servos[0].position = -1
        self.bot.bot.sleep(1)
        self.bot.bot.power_board.outputs[OUT_H0].is_enabled = self.pick
        self.bot.bot.sleep(1)    
        self.bot.bot.servo_board.servos[0].position = 1

    def tick(self):
        self.bot.tried_to_move = False
        pass

    def done(self):
        return True
    
    def complete(self):
        return True

class MoveUltrasound(Process):
    def __init__(self, bot, sep=MOTORS.ULTRASOUND_STOPPING_DISTANCE):
        """
        Takes in the bot and the separation from the front of the ultrasound to stop at
        """
        self.bot = bot
        self.stopping_dist = sep
        self.log = []
        self.complete_ret = False
        self.start_time = self.bot.bot.time()
        self.curr_time = self.start_time
        self.prev_time = self.start_time
        
    def tick(self):
        self.curr_time = self.bot.bot.time()
        dt = self.curr_time - self.prev_time
        self.bot.MotorDriver.power(MOTORS.ULTRASOUND_MOTOR_CONSTANT)
        self.bot.update_position(dt)
        self.prev_time = self.curr_time
        self.bot.tried_to_move = True
    
    def done(self):
        dist = self.bot.UltrasoundDriver.read()
        self.log.append(dist)
        
        if dist <= self.stopping_dist and dist != 0:
            return True
        
        # add other criteria for returning True such as no change in distance for several iterations

        # check to see if no change in distance for 5 iterations
        if len(self.log) >= 5:
            if all(self.log[-i] == self.log[-1] for i in range(2, 5)):
                return True
            
        return False
    
    def complete(self):
        self.bot.MotorDriver.power(0)
        self.bot.bot.sleep(0.25)
        return None

class CollectFloorSample(Process):
    """Generator which takes an ID of a sample (on the floor) that it can see, and just aims to collect it"""
    def __init__(self, bot, marker_id):
        self.bot = bot
        self.marker_id = marker_id
        self.complete_ret = None

        self.start_time = self.bot.bot.time()
        self.current_time = self.start_time
        self.previous_time = self.start_time

        self.process_gen = self.process_generator()  # <- Keep a persistent generator
        self.current_process = None
        self.is_complete = False
        self.complete_ret = False

        # Check to see if can see marker_id
        self.target = self.bot.objects.get(self.marker_id, None)
        if not self.target:
            sub_process = FindMarker(self.bot, marker_ids=[self.marker_id], align=True)
            sub_process.tick()
            while not sub_process.done():
                sub_process.tick()
            wasComplete = sub_process.complete()
            if not wasComplete:
                print("!!! COULD NOT FIND TARGET MARKER IN COLLECT FLOOR SAMPLE INIT, GIVING UP ON THIS PROCESS !!!")
                self.is_complete = True
                return
            
        # If marker is at too much of an angle, align to it
        if abs(self.target.position.horizontal_angle) > 0.3: # TODO check if this threshold is good
            sub_process = Align(self.bot, marker_id=self.marker_id)
            sub_process.tick()
            while not sub_process.done():
                sub_process.tick()
            sub_process.complete()

    def process_generator(self):
        
        yield Move(self.bot, marker_id=self.marker_id, separation=400)
        
        yield MoveArm(self.bot, arm='bottom', new_state=1)
        # yield Align(self.bot, marker_id=self.marker_id)
        # yield DumbMove(self.bot, dist=400)
        yield MoveUltrasound(self.bot)
        self.bot.num_cubes_in_robot += 1
        self.bot.cubes_in_robot_ids.append(self.marker_id)
        #yield MoveUltrasound(self.bot)
        # if not self.bot.is_simulated:
        yield MoveArm(self.bot, arm='bottom', new_state=0)
        # else:
        #     yield SimPickPlace(self.bot, pick=True)

    def tick(self):
        self.current_time = self.bot.bot.time()
        self.delta_time = self.current_time - self.previous_time

        if not self.current_process:
            try:
                self.current_process = next(self.process_gen)
                self.is_complete = False
            except StopIteration:
                print("STATUS >> all processes in CollectFloorSample process generator complete")
                self.is_complete = True
                self.complete_ret = True
                return
            
        self.current_process.tick()
        if self.current_process.done():
            self.current_process.complete()
            self.current_process = None
        
        self.previous_time = self.current_time
    
    def done(self):
        return self.is_complete

    def complete(self):
        self.bot.MotorDriver.power(0)
        return self.complete_ret

class CollectRaisedSample(Process):
    """Generator which takes an ID of a sample (on the platform) that it can see, and just aims to collect it"""
    def __init__(self, bot, marker_id):
        self.bot = bot
        self.marker_id = marker_id
        self.complete_ret = None

        self.start_time = self.bot.bot.time()
        self.current_time = self.start_time
        self.previous_time = self.start_time

        self.process_gen = self.process_generator()  # <- Keep a persistent generator
        self.current_process = None
        self.is_complete = False
        self.complete_ret = False

        # Check to see if can see marker_id
        self.target = self.bot.objects.get(self.marker_id, None)
        if not self.target:
            sub_process = FindMarker(self.bot, marker_ids=[self.marker_id], align=True)
            sub_process.tick()
            while not sub_process.done():
                sub_process.tick()
            wasComplete = sub_process.complete()
            if not wasComplete:
                print("!!! COULD NOT FIND TARGET MARKER IN COLLECT RAISED SAMPLE INIT, GIVING UP ON THIS PROCESS !!!")
                self.is_complete = True
                return
            
        # If marker is at too much of an angle, align to it
        if abs(self.target.position.horizontal_angle) > 0.3: # TODO check if this threshold is good
            sub_process = Align(self.bot, marker_id=self.marker_id)
            sub_process.tick()
            while not sub_process.done():
                sub_process.tick()
            sub_process.complete()

    def process_generator(self):
        
        yield Move(self.bot, marker_id=self.marker_id, separation=400)
        # yield Align(self.bot, marker_id=self.marker_id)
        yield DumbMove(self.bot, dist=1000, motor_power=0.2)
        #yield MoveUltrasound(self.bot)
        # if not self.bot.is_simulated:
        yield KnockSample(self.bot, marker_id=self.marker_id, direction=1) # TODO: work out direction?
        if self.sub_complete_ret:
            self.bot.num_cubes_in_robot += 1
            self.bot.cubes_in_robot_ids.append(self.marker_id)
        # else:
        #     yield SimPickPlace(self.bot, pick=True)

    def tick(self):
        self.current_time = self.bot.bot.time()
        self.delta_time = self.current_time - self.previous_time

        if not self.current_process:
            try:
                self.current_process = next(self.process_gen)
                self.is_complete = False
            except StopIteration:
                print("STATUS >> all processes in CollectRaisedSample process generator complete")
                self.is_complete = True
                self.complete_ret = True
                return
            
        self.current_process.tick()
        if self.current_process.done():
            self.sub_complete_ret = self.current_process.complete()
            self.current_process = None
        
        self.previous_time = self.current_time
    
    def done(self):
        return self.is_complete

    def complete(self):
        self.bot.MotorDriver.power(0)
        return self.complete_ret

class DepositSamplesGen(Process):
    """
    Generator that tries to plan a path from the current position back to the home zone and deposit the samples that the robot is holding
    Will hopefully include SmartMove if that works, and should be able to track whether a box that we deposited earlier can be locked onto to navigate towards in a findmark type approach
    """
    def __init__(self, bot, reverse=True):
        self.bot = bot
        self.reverse = reverse
        self.zone = self.bot.zone
        self.complete_ret = None
        self.previous_time = self.bot.bot.time()

        self.current_process = None
        self.is_complete = False
        self.stuck = False

        self.zone_deposit_markers = list(set(ZONES.WALL_MARKER_IDS[self.zone]) | set(self.bot.cubes_in_laboratory))
        self.found_valid_deposit_marker = False
        self.valid_deposit_marker = None

        self.process_gen = self.process_generator()  # <- Keep a persistent generator, must be last
    
    def process_generator(self):
        """
        First we calculate our anchor point to be used to navigate back to our lab, which can be None, meaning that we should be able to see our lab and navigate to it directly, or an index from 0-3 corresponding to the corner anchor points.
        If we have an anchor point, we can try to navigate to it directly, since the straight line check is done in the util.
        Once we should be able to see our lab, we can turn towards it, and then use find marker to find the wall markers in our lab, or the markers of the samples we have deposited previously.
        """
        
        print('GENERATOR STARTED - depositing')

        anchor_idx = calculate_anchor_point(self.zone, self.bot.position)
        if anchor_idx is not None:
            if anchor_idx == -1:
                print("!!! ERROR IN CALCULATING ANCHOR POINT, NO VALID ANCHOR POINTS AND BOTH BLOCKED!!!")
                # TODO Try to move to another position and recalculate another point probably
            else:
                print(f"Calculated anchor point {anchor_idx} for straight line path to zone")
                anchor_coords = ANCHOR_POINTS.CORNER_ANCHOR_POINTS[anchor_idx]
                angle_to_turn = calculate_angle_to_coord(target_coords=anchor_coords, position_vector=self.bot.position)
                yield DumbMove(self.bot, theta=angle_to_turn)
                
                relevant_corner_markers = list(set(ZONES.WALL_MARKER_IDS[anchor_idx]) & set(self.bot.objects.keys()))
                if len(relevant_corner_markers) > 0:
                    print('can see at least 1 relevant wall marker in the anchor point procedure')
                    if len(relevant_corner_markers) == 1:
                        good_corner_id = relevant_corner_markers[0]
                    else:
                        good_corner_id = min(relevant_corner_markers, key=lambda m: distance(WALLS.WALL_MARKER_IDS[m], ZONES.CENTRE_COORDINATES[self.zone]))
                else:
                    yield FindMarker(self.bot, marker_ids=ZONES.WALL_MARKER_IDS[anchor_idx])
                    good_corner_id = self.bot.complete_ret
                    
                yield Move(self.bot, marker_id = good_corner_id, separation=600)
                
                
                
                # dist_to_drive = distance(self.bot.position[:-1], anchor_coords) * 1000 # mm
                # yield DumbMove(self.bot, dist=dist_to_drive)

        # Then we can continue to drive towards our zone as normal
        # See if we really need to DumbMove or not


        angle_to_turn = calculate_angle_to_coord(target_coords=ARENA.CORNER_COORDS[self.zone], position_vector=self.bot.position)
        yield DumbMove(self.bot, theta=angle_to_turn*1.25)

        yield FindMarker(self.bot, marker_ids=self.zone_deposit_markers, align=False, ret_all=True, direction_to_turn=1)
        found_markers = self.bot.complete_ret
        print('Found markers for deposit:', found_markers)
        
        if found_markers:
            # Check that the box markers are still in our own zone, and if not then remove them from the list of markers to find in the future since they won't be there anymore. Should run this regardless of whether we see the zone wall markers or not since it is important
            found_wall_ids = [marker_id for marker_id in found_markers if marker_id in WALLS.WALL_MARKER_IDS]
            found_sample_ids = [marker_id for marker_id in found_markers if marker_id in SAMPLES.SAMPLE_IDS]
            
            if found_sample_ids:
                for marker_id in found_sample_ids[:]:  # iterate over copy to allow safe removal
                    if marker_id in self.bot.cubes_in_laboratory:
                        if check_sample_zone(self.bot.position, self.bot.objects[marker_id]) != self.zone:
                            print(f"Marker {marker_id} is no longer in our zone, removing from deposit markers")
                            self.bot.cubes_in_laboratory.remove(marker_id)
                            self.bot.stolen_cube_ids.append(marker_id)
                            self.bot.ph += -1 if marker_id in SAMPLES.BASIC_IDS else 1
                            self.zone_deposit_markers.remove(marker_id)
                            found_sample_ids.remove(marker_id)

            if found_wall_ids:
                print('Found wall markers in deposit, using them for alignment and moving towards them')
                self.found_valid_deposit_marker = True
                # Use the marker most face-on to us; min() on a single element is fine
                if ZONES.WALL_MARKER_IDS[self.zone][1] in found_wall_ids:
                    self.valid_deposit_marker = ZONES.WALL_MARKER_IDS[self.zone][1]
                else:
                    self.valid_deposit_marker = min(found_wall_ids, key=lambda m_id: abs(self.bot.objects[m_id].orientation.yaw))              
                
            elif found_sample_ids:  # still-valid sample markers remaining after stolen check
                self.found_valid_deposit_marker = True
                self.valid_deposit_marker = min(found_sample_ids, key=lambda m_id: abs(self.bot.objects[m_id].position.horizontal_angle))
                print('Found a valid sample marker to deposit to, picking the one with lowest horizontal angle to align to', self.valid_deposit_marker)
                
            if self.valid_deposit_marker != ZONES.WALL_MARKER_IDS[self.zone][1]:
                print('Trying to see if we can go to the middle marker')
                yield Move(self.bot, self.valid_deposit_marker, separation=1300)
                self.bot.update_objects()
                if ZONES.WALL_MARKER_IDS[self.zone][1] in self.bot.objects.keys():
                    self.valid_marker = ZONES.WALL_MARKER_IDS[self.zone][1]
                
            if self.found_valid_deposit_marker:
                yield Align(self.bot, self.valid_deposit_marker)
                if self.valid_deposit_marker not in self.bot.objects.keys():
                    print('DIDNT SEE THE MARKER WE ALIGNED TO IN DEPOSIT - GOING TO TRY TO FIND THE OTHER ONES AGAIN BECAUSE THIS IS SOME BS - screw gustavo')
                    yield FindMarker(self.bot, marker_ids=self.zone_deposit_markers, align=False, ret_all=True, direction_to_turn=1)
                    self.valid_deposit_marker = self.bot.complete_ret
                    yield Align(self.bot, marker_id = self.valid_deposit_marker)
                    
                sep = 350 # baseline
                dumb=False
                
                if self.valid_deposit_marker in SAMPLES.SAMPLE_IDS:
                    dumb=True
                    sep = 200
                else:
                    if abs(self.bot.objects[self.valid_deposit_marker].orientation.yaw) > 1.0: # TODO check the yaw angle here
                        print('The deposit marker is at quite a large yaw angle to us, we need to move further into the zone')
                        dumb=True
                
                yield Move(self.bot, self.valid_deposit_marker, separation=sep) # TODO check the separation distance
                
                if dumb:
                    yield DumbMove(self.bot, dist=150)
                
            else:
                print('!!! NO VALID MARKERS FOUND TO DEPOSIT TO AT THE END, lowkey giving up for now')
        
        if self.reverse:
            # Lift the arm, reverse, lower the arm
            yield MoveArm(self.bot, arm='bottom', new_state=1)
            yield DumbMove(self.bot, dist=-300)
            yield MoveArm(self.bot, arm='bottom', new_state=0)
  
    def tick(self):
        print('IN DEPOSIT TICK')
        while not self.is_complete: # Should only tick once until the process generator is empty
            self.subtick()
        # Done so can just continue

    def subtick(self):
        print('IN DEPOSIT SUBTICK')

        if self.bot.stuck and False:
            print("!!! STUCK !!!")
            self.bot.un_stick()

        self.current_time = self.bot.bot.time()
        dt = self.current_time - self.previous_time
        self.bot.update_position(dt)

        if not self.current_process:
            try:
                self.current_process = next(self.process_gen)
                self.is_complete = False
            except StopIteration:
                print("All Processes Complete")
                self.is_complete = True

        if not self.is_complete:
            self.current_process.tick()

            if self.current_process.done():
                self.complete_ret = self.current_process.complete()
                self.current_process = None
        
        self.previous_time = self.current_time
    
    def done(self):
        print('in deposit done')
        return self.is_complete # should return True always since tick only breaks when is_complete is True
    
    def complete(self):
        self.bot.MotorDriver.power(0)

        # Handle all the bot variable updates
        self.bot.cubes_in_laboratory.extend(self.bot.cubes_in_robot_ids)
        self.bot.ph += sum(1 if c in SAMPLES.BASIC_IDS else -1 for c in self.bot.cubes_in_robot_ids)
        self.bot.num_cubes_in_robot = 0
        for item in self.bot.cubes_in_robot_ids:
            self.bot.red_boxes_remaining.remove(item)
        self.bot.cubes_in_robot_ids.clear()
        
        self.bot.localiser.update_zero_velocity()
        self.bot.complete_ret = self.complete_ret
        self.bot.bot.sleep(0.25)
        return self.complete_ret


class KnockSample(Process):
    def __init__(self, bot, marker_id, direction=1):
        """
        Takes in the bot, the marker_id to knock down, and the direction to turn if known
        If direction is not known, it will default to turning in the direction of the angle to the marker if the position is known
        Tries to turn sufficiently to knock the sample off the platform
        """
        self.bot = bot
        self.marker_id = marker_id
        self.complete_ret = None

        self.start_time = self.bot.bot.time()
        self.current_time = self.start_time
        self.previous_time = self.start_time

        self.process_gen = self.process_generator()  # <- Keep a persistent generator
        self.current_process = None
        self.is_complete = False
        self.complete_ret = False

        self.target = self.bot.objects.get(self.marker_id, None)
        if not self.target:
            print("!!! CANNOT SEE TARGET MARKER IN KNOCK SAMPLE INIT, GIVING UP ON THIS PROCESS !!!")
            self.is_complete = True
            return
        
        self.direction = direction
        # if self.direction is None:
        #     angle = self.target.position.horizontal_angle
        #     if angle >= 0:
        #         self.direction = 1
        #     else:
        #         self.direction = -1
        
    def process_generator(self):
        yield MoveArm(self.bot, arm="top", new_state=1)
        yield DumbMove(self.bot, theta=np.pi/4, motor_power=0.87) # TODO check if this angle is good
        yield MoveArm(self.bot, arm="top", new_state=0)
        yield DumbMove(self.bot, theta=-1 * np.pi/2.25)
        yield DumbMove(self.bot, dist=-150)
        #yield DumbMove(self.bot, theta=-self.direction * np.pi/6) # TODO check if this angle is good
        #yield DumbMove(self.bot, dist=-500)

        self.bot.bot.sleep(0.5)
        self.target = self.bot.objects.get(self.marker_id, None)
        if self.target:
            self.bot.bot.sleep(0.25)
            yield Align(self.bot, marker_id=self.marker_id)
            self.bot.bot.sleep(0.25)
            yield MoveArm(self.bot, arm="bottom", new_state=1)
            # yield DumbMove(self.bot, dist=dist + 200)
            if self.bot.num_cubes_in_robot == 0:
                yield MoveUltrasound(self.bot)
            else: 
                #yield DumbMove(self.bot, dist=500)
                yield MoveUltrasound(self.bot) # TODO: Verify if this is right
            yield MoveArm(self.bot, arm="bottom", new_state=0)
            self.complete_ret = True
        else:
            yield FindMarker(self.bot, marker_ids=[self.marker_id], align=True, direction_to_turn=-1)
            if self.sub_process_ret:
                #dist = self.bot.objects[self.sub_process_ret].position.distance
                yield Align(self.bot, marker_id=self.marker_id)
                yield MoveArm(self.bot, arm="bottom", new_state=1)
                # yield DumbMove(self.bot, dist=dist + 200)
                if self.bot.num_cubes_in_robot == 0:
                    yield MoveUltrasound(self.bot)
                else: 
                    # yield DumbMove(self.bot, dist=500)
                    yield MoveUltrasound(self.bot) # TODO: Verify if this is right
                yield MoveArm(self.bot, arm="bottom", new_state=0)
                self.complete_ret = True
            else: self.complete_ret = False


    def tick(self):
        self.current_time = self.bot.bot.time()
        self.delta_time = self.current_time - self.previous_time

        if not self.current_process:
            try:
                self.current_process = next(self.process_gen)
                self.is_complete = False
            except StopIteration:
                print("STATUS >> all processes in PickPalletGen process generator complete")
                self.is_complete = True
                self.complete_ret = True
                return
            
        self.current_process.tick()
        if self.current_process.done():
            self.sub_process_ret = self.current_process.complete()
            self.current_process = None
        
        self.previous_time = self.current_time
    
    def done(self):
        return self.is_complete

    def complete(self):
        self.bot.MotorDriver.power(0)
        return self.complete_ret

class TestProcess(Process):
    
    def __init__(self, bot):
        self.bot = bot

    def tick(self):

        print("\nSTATUS >> IN TICK")

        print("ZONE:", self.bot.zone)
        print("START_TIME:", self.bot.start_time)
        print("CURRENT_PROCESS:", self.bot.current_process)

        print("OBJECTS:", self.bot.objects)
        print("POSITION:", self.bot.position)

        print("STUCK:", self.bot.stuck)
        print("PROCESS_GEN:", self.bot.process_gen)

        print("IS_SIMULATED:", self.bot.is_simulated)
        self.bot.tried_to_move = False

    def done(self):

        print("\nSTATUS >> IN DONE")

        return True
    
    def complete(self):
        
        print("\nSTATUS >> IN COMPLETE")

        return "Complete Ret"