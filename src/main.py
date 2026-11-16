from sr.robot3 import *
import numpy as np
from collections import deque

from src import process
from src import utils
from src import constants
from src.filter.localiser import UKFLocaliser, localisation_tick
from src.filter.kf_utils import count_wall_markers

class LeBotJames:
    """The Superclass of our Robot"""

    def __init__(self, existing = None):

        if existing is None:
            self.bot = Robot()
        else:
            self.bot = existing

        self.MotorDriver = None
        self.ServoDriver = None
        self.UltrasoundDriver = None

        self.zone = self.bot.zone
        self.start_time = self.bot.time()
        self.previous_time = self.bot.time()
        self.current_time = self.bot.time()
        self.delta_time = 0
        
        # Kalman filter stuff
        self.localiser = UKFLocaliser() 
        self.wall_marker_missing_iterations = 0

        self.objects = {}
        self.position = None # TODO Determine where we want to start and initialise accordingly

        self.cubes_in_laboratory = [] # Stores the marker ids for the cubes stored in our area
        self.num_cubes_in_robot = 0 # Varies between 0, 1 and 2 as we pick up cubes
        self.cubes_in_robot_ids = [] # The ids of the cubes currently in our robot
        self.stolen_cube_ids = [] # Number of cubes that have been stolen from our laboratory
        self.ph = 7

        self.red_boxes_remaining = list(constants.SAMPLES.ACIDIC_IDS.copy())
        
        self.stuck = False
        self.iterations_since_last_stuck = 0
        self.historic_objects = deque(5 * [{}], maxlen=5)
        self.common_objects_history = deque(20 * [True], maxlen=10)
        self.tried_to_move = False

        self.complete_ret = None
        self.is_complete = False

        self.process_gen = self.process_generator()
        self.current_process = None

        self.is_simulated = bool(str(self.bot.mode) == "RobotMode.DEV")

        
    def process_generator(self):
        # TODO: MAKE SURE YOU SWAP THE PROCESS GENERATORS BEFORE THE COMPETITION

        """Uncomment from here"""
        
        # TRY AND GET FIRST BOX
        
        yield process.DumbMove(self, dist=100)
        yield process.DumbMove(self, theta=(np.pi/9))

        yield process.FindMarker(self, marker_ids=constants.SAMPLES.ACIDIC_IDS, align=False, ret_all=True)
        red_markers_seen = self.complete_ret
        red_floor_markers = [m for m in red_markers_seen if utils.calculate_box_height(self.objects[m]) == constants.SAMPLES.HALF_LENGTH / 1000]
        # I'm banking on the fact that we can hard engineer the initial move procedure so it can consistently see the intended target
        if red_floor_markers == []:
            print("WHAT THE ACTUAL FUDGE, HOW DID THIS HAPPEN, WE'RE COOKED, THE THING IS GEEKED, SCREW ANTON")
        else:  
            least_angle_on_floor = min(red_floor_markers, key = lambda x: self.objects[x].position.horizontal_angle)
            yield process.CollectFloorSample(self, marker_id = least_angle_on_floor)

            yield process.DepositSamplesGen(self)
        
        # TRY AND GET SECOND BOX

        # # Dumb Move twice because function is weird
        # yield process.DumbMove(self, theta=np.pi/3)
        # yield process.DumbMove(self, theta=np.pi/3)

        wall_marker = (self.zone * 5) + 1
        yield process.FindMarker(self, marker_id=wall_marker, direction_to_turn=1, align=False)
        yield process.Move(self, marker_id=wall_marker, separation=500)
<<<<<<< HEAD
        yield process.DumbMove(self, theta=0.5)
        self.bot.sleep(1)
=======
        # yield process.DumbMove(self, theta=0.49)
        # self.bot.sleep(1)
>>>>>>> 479688af32a2bb004ec346da5d4153ab06146c33
        yield process.FindMarker(self, marker_ids=self.red_boxes_remaining, ret_all=True, direction_to_turn=1)

        red_markers_seen = self.complete_ret
        red_floor_markers = [m for m in red_markers_seen if utils.calculate_box_height(self.objects[m], floor_true=True)]

        if red_floor_markers == []:
            print("WHAT THE ACTUAL FUDGE, HOW DID THIS HAPPEN, WE'RE COOKED, THE THING IS GEEKED, SCREW jake")
        else:
            least_angle_on_floor = min(red_floor_markers, key = lambda x: self.objects[x].position.horizontal_angle)
<<<<<<< HEAD
            yield process.Align(self, marker_id=least_angle_on_floor)
            yield process.CollectFloorSample(self, marker_id = least_angle_on_floor)
            yield process.DepositSamplesGen(self, find_wall_direction=-1)

        # TRY AND GET THIRD BOX
        
        # wall_marker = (self.zone * 5) + 1
        # yield process.FindMarker(self, marker_id=wall_marker, direction_to_turn=1, align=False)
        # yield process.Move(self, marker_id=wall_marker, separation=500)
        
        # # yield process.DumbMove(self, theta=0.59)
        # # self.bot.sleep(1)
        # yield process.FindMarker(self, marker_ids=self.red_boxes_remaining, ret_all=True, direction_to_turn=1)

        # red_markers_seen = self.complete_ret
        # red_elevated_markers = [m for m in red_markers_seen if not(utils.calculate_box_height(self.objects[m], floor_true=True))]

        # if red_elevated_markers == []:
        #     print("WHAT THE ACTUAL FUDGE, HOW DID THIS HAPPEN, WE'RE COOKED, THE THING IS GEEKED, SCREW jake")
        # else:
        #     least_angle_on_floor = min(red_elevated_markers, key = lambda x: self.objects[x].position.horizontal_angle)
        #     yield process.CollectRaisedSample(self, marker_id = least_angle_on_floor)
        #     yield process.DepositSamplesGen(self)
        
        # Other side of arena
        wall_marker = ((self.zone + 1) * 5) % 20
        yield process.FindMarker(self, marker_id=wall_marker, direction_to_turn=1, align=False)
        yield process.Move(self, marker_id=wall_marker, separation=500)
        yield process.FindMarker(self, marker_ids=self.red_boxes_remaining, ret_all=True, direction_to_turn=1)

        red_markers_seen = self.complete_ret
        red_floor_markers = [m for m in red_markers_seen if utils.calculate_box_height(self.objects[m], floor_true=True)]

        if red_floor_markers == []:
            print("WHAT THE ACTUAL FUDGE, HOW DID THIS HAPPEN, WE'RE COOKED, THE THING IS GEEKED, SCREW jake")
        else:
            least_angle_on_floor = min(red_floor_markers, key = lambda x: self.objects[x].position.horizontal_angle)
            yield process.Align(self, marker_id=least_angle_on_floor)
            yield process.CollectFloorSample(self, marker_id = least_angle_on_floor)
            yield process.DepositSamplesGen(self, reverse=False)
        
        quit()

        # yield process.DumbMove(self, theta=np.pi-0.01)

        # yield process.FindMarker(self, marker_ids=self.red_boxes_remaining, direction_to_turn=1, ret_all = True)
        
        # seen_red_ids = self.complete_ret
        # if seen_red_ids:
        #     elevated_reds_seen = [m for m in seen_red_ids if not utils.calculate_box_height(self.objects[m], floor_true = True)]
            
        #     if elevated_reds_seen:
        #         least_horz_angle = min(elevated_reds_seen, key=lambda x: self.objects[x].position.horizontal_angle) # returns the marker_id for this
        #         print(least_horz_angle)
        #         yield process.CollectRaisedSample(self, marker_id=least_horz_angle)
        #         yield process.DepositSamplesGen(self)
        #     else:
        #         print('Saw a floor red marker instead of a raised one')
        #         # Might as well try and get the floor one then LOL
        #         least_angle_on_floor = min(seen_red_ids, key = lambda x: self.objects[x].position.horizontal_angle)
        #         yield process.CollectFloorSample(self, marker_id = least_angle_on_floor)
        #         yield process.DepositSamplesGen(self)
        # else:
        #     print('find marker returned None because it timed out')            
        
        # for m_id in seen_red_ids:
        #     elevated_true = not (utils.caculate_box_height(marker=self.objects[seen_red_ids], floor_true=True))
        #     if elevated_true:
        #         elevated_red_found = m_id
        
        # yield process.DumbMove(self, theta=(np.pi-0.02))
        
        # yield process.FindMarker(self, marker_ids=self.red_boxes_remaining, align=False, ret_all=True, direction_to_turn=1)
        # red_markers_seen = self.complete_ret
        # red_elevated_markers = [m for m in red_markers_seen if utils.calculate_box_height(self.objects[m]) != constants.SAMPLES.HALF_LENGTH / 1000]
        # if red_elevated_markers == []:
        #     print("COULDNT SEE ANY ELEVATED MARKERS")
        # else:
        #     least_angle_elevated = min(red_elevated_markers, key = lambda x: self.objects[x].position.horizontal_angle)
        #     yield process.CollectRaisedSample(self, marker_id=least_angle_elevated)
        #     yield process.DepositSamplesGen(self)

        # TRY AND GET THIRD BOX

        # yield process.FindMarker(self, marker_ids=self.red_boxes_remaining, align=False, ret_all=True, direction_to_turn=1)
        # red_markers_seen = self.complete_ret
        # red_floor_markers = [m for m in red_markers_seen if utils.calculate_box_height(self.objects[m]) == constants.SAMPLES.HALF_LENGTH / 1000]

        # if red_floor_markers == []:
        #     print("WHAT THE ACTUAL FUDGE, HOW DID THIS HAPPEN, WE'RE COOKED, THE THING IS GEEKED, SCREW jake")
        # else:
        #     least_angle_on_floor = min(red_floor_markers, key = lambda x: self.objects[x].position.horizontal_angle)
        #     yield process.CollectFloorSample(self, marker_id = least_angle_on_floor)
        #     yield process.DepositSamplesGen(self)

=======
            yield process.CollectFloorSample(self, marker_id = least_angle_on_floor)
            yield process.DepositSamplesGen(self)
>>>>>>> 479688af32a2bb004ec346da5d4153ab06146c33

        # TRY AND GET THIRD BOX
        
        wall_marker = (self.zone * 5) + 1
        yield process.FindMarker(self, marker_id=wall_marker, direction_to_turn=1, align=False)
        yield process.Move(self, marker_id=wall_marker, separation=500)
        
        # yield process.DumbMove(self, theta=0.59)
        # self.bot.sleep(1)
        yield process.FindMarker(self, marker_ids=self.red_boxes_remaining, ret_all=True, direction_to_turn=1)

        red_markers_seen = self.complete_ret
        red_elevated_markers = [m for m in red_markers_seen if not(utils.calculate_box_height(self.objects[m], floor_true=True))]

        if red_elevated_markers == []:
            print("WHAT THE ACTUAL FUDGE, HOW DID THIS HAPPEN, WE'RE COOKED, THE THING IS GEEKED, SCREW jake")
        else:
            least_angle_on_floor = min(red_elevated_markers, key = lambda x: self.objects[x].position.horizontal_angle)
            yield process.CollectRaisedSample(self, marker_id = least_angle_on_floor)
            yield process.DepositSamplesGen(self)
        
        # Other side of arena
        wall_marker = ((self.zone + 1) * 5) % 20
        yield process.FindMarker(self, marker_id=wall_marker, direction_to_turn=1, align=False)
        yield process.Move(self, marker_id=wall_marker, separation=500)
        yield process.FindMarker(self, marker_ids=self.red_boxes_remaining, ret_all=True, direction_to_turn=1)

        red_markers_seen = self.complete_ret
        red_floor_markers = [m for m in red_markers_seen if utils.calculate_box_height(self.objects[m], floor_true=True)]

        if red_floor_markers == []:
            print("WHAT THE ACTUAL FUDGE, HOW DID THIS HAPPEN, WE'RE COOKED, THE THING IS GEEKED, SCREW jake")
        else:
            least_angle_on_floor = min(red_floor_markers, key = lambda x: self.objects[x].position.horizontal_angle)
            yield process.CollectFloorSample(self, marker_id = least_angle_on_floor)
            yield process.DepositSamplesGen(self, reverse=False)
        
        quit()

        # yield process.DumbMove(self, theta=np.pi-0.01)

        # yield process.FindMarker(self, marker_ids=self.red_boxes_remaining, direction_to_turn=1, ret_all = True)
        
        # seen_red_ids = self.complete_ret
        # if seen_red_ids:
        #     elevated_reds_seen = [m for m in seen_red_ids if not utils.calculate_box_height(self.objects[m], floor_true = True)]
            
        #     if elevated_reds_seen:
        #         least_horz_angle = min(elevated_reds_seen, key=lambda x: self.objects[x].position.horizontal_angle) # returns the marker_id for this
        #         print(least_horz_angle)
        #         yield process.CollectRaisedSample(self, marker_id=least_horz_angle)
        #         yield process.DepositSamplesGen(self)
        #     else:
        #         print('Saw a floor red marker instead of a raised one')
        #         # Might as well try and get the floor one then LOL
        #         least_angle_on_floor = min(seen_red_ids, key = lambda x: self.objects[x].position.horizontal_angle)
        #         yield process.CollectFloorSample(self, marker_id = least_angle_on_floor)
        #         yield process.DepositSamplesGen(self)
        # else:
        #     print('find marker returned None because it timed out')            
        
        # for m_id in seen_red_ids:
        #     elevated_true = not (utils.caculate_box_height(marker=self.objects[seen_red_ids], floor_true=True))
        #     if elevated_true:
        #         elevated_red_found = m_id
        
        # yield process.DumbMove(self, theta=(np.pi-0.02))
        
        # yield process.FindMarker(self, marker_ids=self.red_boxes_remaining, align=False, ret_all=True, direction_to_turn=1)
        # red_markers_seen = self.complete_ret
        # red_elevated_markers = [m for m in red_markers_seen if utils.calculate_box_height(self.objects[m]) != constants.SAMPLES.HALF_LENGTH / 1000]
        # if red_elevated_markers == []:
        #     print("COULDNT SEE ANY ELEVATED MARKERS")
        # else:
        #     least_angle_elevated = min(red_elevated_markers, key = lambda x: self.objects[x].position.horizontal_angle)
        #     yield process.CollectRaisedSample(self, marker_id=least_angle_elevated)
        #     yield process.DepositSamplesGen(self)

        # TRY AND GET THIRD BOX

        # yield process.FindMarker(self, marker_ids=self.red_boxes_remaining, align=False, ret_all=True, direction_to_turn=1)
        # red_markers_seen = self.complete_ret
        # red_floor_markers = [m for m in red_markers_seen if utils.calculate_box_height(self.objects[m]) == constants.SAMPLES.HALF_LENGTH / 1000]

        # if red_floor_markers == []:
        #     print("WHAT THE ACTUAL FUDGE, HOW DID THIS HAPPEN, WE'RE COOKED, THE THING IS GEEKED, SCREW jake")
        # else:
        #     least_angle_on_floor = min(red_floor_markers, key = lambda x: self.objects[x].position.horizontal_angle)
        #     yield process.CollectFloorSample(self, marker_id = least_angle_on_floor)
        #     yield process.DepositSamplesGen(self)


        while True:
        
            yield process.DumbMove(self, theta=np.pi/3 * 0.60)
            yield process.DumbMove(self, dist=300)
            
            
            # Second box
            print("\n>>>TRYING TO GET SECOND BOX<<<\n")
            yield process.FindMarker(self, marker_ids=self.red_boxes_remaining, align=False, ret_all=True, direction_to_turn=-1)
            red_markers_seen = self.complete_ret
            print("RED MARKERS SEEN:", red_markers_seen)
            red_floor_markers = [m for m in red_markers_seen if utils.calculate_box_height(self.objects[m]) == constants.SAMPLES.HALF_LENGTH / 1000]
            print("RED FLOOR MARKERS:", red_floor_markers)
            # I'm banking on the fact that we can hard engineer the initial move procedure so it can consistently see the intended target
            if red_floor_markers == []:
                print("WHAT THE ACTUAL FUDGE, HOW DID THIS HAPPEN, WE'RE COOKED, THE THING IS GEEKED, SCREW jake")
            else:  
                least_angle_on_floor = min(red_floor_markers, key = lambda x: self.objects[x].position.horizontal_angle)
                print("LEAST ANGLE ON FLOOR:", least_angle_on_floor)
                yield process.CollectFloorSample(self, marker_id = least_angle_on_floor)

                yield process.DepositSamplesGen(self)
            
        # yield process.DepositSamplesGen(self)

    def spare(self):
        yield process.FindMarker(self, marker_id=103)
        # yield process.KnockSample(self, marker_id=103)
        
        # yield process.FindMarker(self, marker_ids=constants.SAMPLES.ACIDIC_IDS, align=False, ret_all=True)
        # red_markers_seen = self.complete_ret
        # red_elevated_markers = [m for m in red_markers_seen if utils.calculate_box_height(self.objects[m]) == constants.SAMPLES.HALF_LENGTH / 1000]
        # if red_elevated_markers == []:
        #     print("COULDNT SEE ANYTHING")
        #     quit()
        # else:
        #     least_angle_elevated = min(red_elevated_markers, key = lambda x: self.objects[x].position.horizontal_angle)
        #     yield process.CollectRaisedSample(self, marker_id=least_angle_elevated)
        #     yield process.DepositSamplesGen(self)


    def update_objects(self):
        """Update objects the robot can see"""

        self.objects.clear()
        markers = self.bot.camera.see()
        
        # If there are multiple markers with the same id, we take the one with the lowest absolute yaw value
        # This is useful for when we are aligning to a sample since we want to align to the one that is the most front on to us
        for marker in markers:
            id = marker.id
            if id not in self.objects:
                # First time seeing this id
                self.objects[id] = marker
            else:
                # Duplicate id, keep lowest absolute yaw
                existing = self.objects[id]
                if abs(marker.orientation.yaw) < abs(existing.orientation.yaw):
                    self.objects[id] = marker

        self.historic_objects.append(self.objects)

        return markers
    
    def update_position(self, dt):
        if dt == 0:
            markers = self.bot.camera.see()
            if len(markers) > 1:
                self.position = utils.triangulation(markers)
                # print("INITIAL POSITION CALCULATION (TRIANGULATION):", self.position)
            elif len(markers) == 1:
                self.position = utils.pose_estimation(markers)
                # print("INITIAL POSITION CALCULATION (POSE ESTIMATION):", self.position)
            else:
                print('!!! NO MARKERS DETECTED FOR INITIAL POSITION CALCULATION !!!')
                pass
            return self.position
        
        if dt <= 1e-6:
            dt = 1e-3

        # Kalman filter updates:
        # 1) Sense: get markers and then derive pose measurements in here
        markers = self.update_objects()
        
        # Measurements for the UKF:
        wall_pose = None
        tri_pose = None
        wall_n = 0

        # Only run pose_estimation if we can see wall markers
        wall_n = count_wall_markers(markers)
        if wall_n >= 2:
            # wall_pose = utils.pose_estimation(markers)
            # Triangulation not wired yet
            tri_pose = utils.triangulation(markers)
            self.wall_marker_missing_iterations = 0
        elif wall_n == 1:
            wall_pose = utils.pose_estimation(markers)
            self.wall_marker_missing_iterations = 0
        elif wall_n == 0:
            self.wall_marker_missing_iterations += 1

        # 2) Predict+Update UKF using last motor command we actually sent
        uL, uR = self.MotorDriver.get_powers() if self.MotorDriver else (0.0, 0.0)
        localisation_tick(
            self.localiser,
            uL=uL,
            uR=uR,
            dt=dt,
            triangulation_pose=tri_pose,
            wall_pose=wall_pose,
            wall_marker_count=wall_n,
            wall_marker_missing_iterations=self.wall_marker_missing_iterations,
        )

        # 3) Output pose estimate every tick
        self.position = tuple(round(v, 3) for v in self.localiser.pose())
        print("POSITION:", self.position, 'Markers detected', wall_n, 'Wall missing', self.wall_marker_missing_iterations)
        print()
        return self.position
    
    def tick(self):
        """Called every tick"""
        
        # No need to call update_objects since it is called in update_position anyway which happens before the process tick where we need the updated objects
        # self.stuck = self.is_stuck() and False

        # if self.stuck and False:
        #     print("!!! STUCK !!!")
        #     self.un_stick()

        self.current_time = self.bot.time()
        dt = self.current_time - self.previous_time

        self.update_position(dt)

        if not self.current_process:
            try:
                self.current_process = next(self.process_gen)
                self.is_complete = False
            except StopIteration:
                print("All Processes Complete")
                quit()

        self.current_process.tick()

        if self.current_process.done():
            self.complete_ret = self.current_process.complete()
            self.current_process = None
        
        self.previous_time = self.bot.time()  # capture actual end time so next tick's dt doesn't span the entire DumbMove duration
    
    def is_stuck(self):
        """Check is the robot is stuck"""

        self.iterations_since_last_stuck += 1
        if self.iterations_since_last_stuck < 5:
            # Too soon to check if we're stuck
            return False
        
        old_objects, new_objects = self.historic_objects[0], self.historic_objects[-1]
        common_objects = [(id, old_objects[id], new_objects[id]) for id in old_objects if id in new_objects]
        self.common_objects_history.append(bool(common_objects))

        if not common_objects:
            if sum(self.common_objects_history) < 5:
                self.iterations_since_last_stuck = 0
                print("No Common Objects")
                return True
            
        elif self.tried_to_move:
            threshold = len(common_objects) * 0.75
            num_items_not_changed = 0

            for id, old, new in common_objects:
                if abs(old.position.distance - new.position.distance) < 10 and abs(old.position.horizontal_angle - new.position.horizontal_angle) < 0.05:
                    num_items_not_changed += 1
            
            if num_items_not_changed > threshold:
                self.iterations_since_last_stuck = 0
                print("Items are lowk not moving")
                return True        

        return False

    def un_stick(self):
        """Tries to un-stick the robot if it is stuck"""

        # Wiggle
        wiggle_one = process.DumbMove(self, theta=np.pi/10, dist=0)
        wiggle_two = process.DumbMove(self, theta=-np.pi/10, dist=0)
        for _ in range(3):
            wiggle_one.tick()
            wiggle_two.tick()
        wiggle_one.complete()
        wiggle_two.complete()
        
        # Reverse
        temp_process = process.DumbMove(self, theta=0, dist=-200)
        temp_process.tick()
        temp_process.complete()

        # Turn
        temp_process = process.DumbMove(self, theta=np.pi/4, dist=0)
        temp_process.tick()
        temp_process.complete()
