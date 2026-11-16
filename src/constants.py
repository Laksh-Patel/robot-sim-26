from numpy import array, pi


MAD_THRESHOLD = 5
"""Number of median absolute deviations away from the median which will cause values to be rejected when calculating a robust mean"""

DISTANCE_STUCKNESS_THRESHOLD = 25  # mm
"""Distance which is considered to be stuck"""

ANGLE_STUCKNESS_THRESHOLD = 0.05  # rad
"""Angle which is considered to be stuck"""

"""Constants for camera"""


class CAMERA:

    ACCEPTANCE_RANGE = 0.1  # rad
    """Angle in radians which is considered to be straight ahead"""

    CAMERA_HEIGHT = 320  # mm
    """Height of the camera above the ground
    320 for the real robot and 87.5 for the sim
    """

    CAMERA_DECLINE = 0  # rad
    """Downwards angle of the camera relative to the horizontal"""

    CAMERA_FOV = 0.42  # rad
    """Field of view of the camera"""

    CAMERA_CORRECTION = 0.0  # rad
    """Correction factor for the camera angle"""


"""Constants for robot"""


class ROBOT:

    ROBOT_HALF_WIDTH = 165  # mm
    """Approximate half width of the robot"""

    ROBOT_SIDE_TOLERANCE = 180  # mm
    """Space to leave on the sides of the robot to avoid collisions"""


"""Constants for motors"""


class MOTORS:

    MOTOR_BOARD_NAME = "SR0TEA"

    MOTOR_CONSTANT = 0.4
    """Maximum power given to the motors"""

    MAX_POWER_CHANGE = 0.25

    LINEAR_VELOCITY = 0.85 # m/s
    """Linear velocity of the robot when power in both motors is 0.40 (update MOTOR_CONSTANT accordingly)"""

    LINEAR_DECELERATION = 6.25 # m/s^2
    LINEAR_ACCELERATION = 6.25 # m/s^2
    """Linear acceleration/deceleration of the robot"""

    # ANGULAR_VELOCITY = 5.75 * 1.0 # rad/s
    # ANGULAR_VELOCITY = 7.25 # rad / s
    ANGULAR_VELOCITY = 7.75 # rad / s
    """Estimated angular velocity of the robot, trying to account for deceleration"""

    ANGULAR_ACCELERATION = 78 * 1.0 # rad/s^2
    ANGULAR_DECELERATION = 78 * 1.0 # rad/s^2
    """Angular acceleration/deceleration of the robot"""

    ULTRASOUND_MOTOR_CONSTANT = 0.25
    """Motor power for both motors when driving with ultrasound"""

    SLOW_MOTOR_CONSTANT = 0.27
    SLOW_ANGULAR_VELOCITY = 0.85
    SLOW_ANGULAR_ACCELERATION = 35 # rad / s
    SLOW_ANGULAR_DECELERATION = 35 # rad / s
    SMALL_ANGLE_THRESHOLD = 0.5
    
    SLOW_LINEAR_VELOCITY = 2 / 7.80
    """Values for slow driving for distances less than 500mm"""

    LEFT_MOTOR_CORRECTION = 1
    RIGHT_MOTOR_CORRECTION = 0.90
    """Correction factors for the left and right motors"""

    ULTRASOUND_STOPPING_DISTANCE = 80  # mm
    """The distance that the front ultrasound should read to signal
    that the robot is ready to pick up the pallet"""

    ALIGNMENT_DISTANCE = 1000  # mm
    """The distance away from a pallet the robot should stop to align to the pallet, before driving with ultrasound only"""

    ALIGNMENT_POWER = 0.105
    """Power to use on each motor when aligning to a pallet"""

    MOTOR_CONSTANT_DISTANCE = 1
    MAX_ANGLE_POWER = 0.125
    MAX_ANGLE = 0.42
    """Arguments for the sigmoid function used to calculate motor power"""

    BUILD_DISTANCE = 500  # mm
    """Distance that the robot travels when building power to MOTOR CONSTANT"""


"""Constants for Servos"""


class SERVO:
    pass


"""Constants for arena"""


class ARENA:

    ARENA_HALF_EDGE = 2.2875  # m
    """Half the length of the arena"""

    CORNER_COORDS = array([
        (2.2875, -2.2875), (-2.2875, -2.2875), (-2.2875, 2.2875), (2.2875, 2.2875)
    ])

"""Constants for markers"""


"""Constants for the anchor points"""
class ANCHOR_POINTS:
    coord = 1.55
    CORNER_ANCHOR_POINTS = array([
        (coord,-coord), (-coord,-coord), (-coord,coord), (coord,coord)
    ])
    
    ZONE_ANCHOR_POINTS = array([(1,3),(0,2),(1,3),(0,2)])
    # The anchor points that should be chosen between when trying to navigate to zone i are stored in zone_anchor_points[i]
    # So the points for zone 0/2 are 1,3 and for 1/3 are 0,2

class WALLS:

    WALL_MARKER_POSITIONS = array(
        [
            [1.525, -2.2875],
            [0.7625, -2.2875],
            [0.0, -2.2875],
            [-0.7625, -2.2875],
            [-1.525, -2.2875],
            [-2.2875, -1.525],
            [-2.2875, -0.7625],
            [-2.2875, 0.0],
            [-2.2875, 0.7625],
            [-2.2875, 1.525],
            [-1.525, 2.2875],
            [-0.7625, 2.2875],
            [0.0, 2.2875],
            [0.7625, 2.2875],
            [1.525, 2.2875],
            [2.2875, 1.525],
            [2.2875, 0.7625],
            [2.2875, 0.0],
            [2.2875, -0.7625],
            [2.2875, -1.525]
        ]
    )
    """
    Positions of the wall markers (WALL_MARKERS[MARKER_ID])
    (x, y)
    """

    WALL_MARKER_IDS = array(range(20))
    """Set of ids of markers found on walls"""


"""Constants for Zones"""


class ZONES:

    BOUNDARIES = array([
        [(2.2875, -2.2875), (1.2875, -0.2875)],
        [(-2.2875, -2.2875), (-0.2875, -1.2875)],
        [(-2.2875, 2.2875), (-1.2875, 0.2875)],
        [(2.2875, 2.2875), (1.2875, 0.2875)]
    ])

    CENTRE_COORDINATES = array([
        (1.7875, -1.2875), (-1.2875, -1.7875), (-1.7875, 1.2875), (1.2875, 1.7875)
    ])

    WALL_MARKER_IDS = [
        [18, 19, 0],
        [3, 4, 5],
        [8, 9, 10],
        [13, 14, 15]
    ]


"""Constants for Centre"""


class CENTRE:

    HEIGHT = 0.18  # metres
    LENGTH = 1.220  # metres, side lengths form a square
    CORNERS = array([
        (0.61, -0.61),
        (-0.61, -0.61),
        (-0.61, 0.61),
        (0.61, 0.61)
    ])
    TOLERANCE = 0.15 # m
    INFLATED_CORNERS = array([
        (0.61 + TOLERANCE, 0.61 + TOLERANCE),
        (-0.61 - TOLERANCE, -0.61 - TOLERANCE)
    ])


"""Constants for Samples"""


class SAMPLES:

    ACIDIC_IDS = list(array(range(100, 140)))  # red
    BASIC_IDS = list(array(range(140, 180)))  # blue
    SAMPLE_IDS = list(array(range(100, 180)))  # all

    ACIDIC_POSITIONS = array([
        (1, -1), (-1, -1), (-1, 1), (1, 1), (0.5, 0), (0, 0.5), (-0.5, 0), (0, -0.5)
    ])

    BASIC_POSITIONS = array([
        (1, 0), (0, 1), (0, -1), (-1, 0), (0.5,
                                           0.5), (-0.5, -0.5), (0.5, -0.5), (-0.5, 0.5)
    ])

    HALF_LENGTH = 65  # mm
    """Half the length of the sample blocks, which are square"""
