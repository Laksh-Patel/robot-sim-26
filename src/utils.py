import numpy as np
from itertools import combinations
from math import sqrt
from src.constants import *
# from constants import *

def distance(p1, p2):
    """Gets the distance between two points in 2D space"""
    return np.hypot(p1[0] - p2[0], p1[1] - p2[1])

def calculate_object_coords(robot_coordinates, object_distance, object_angle):
    """Calculates the position of an object using its relative data from the robot, and the robot's position"""

    object_distance = object_distance / 1000  # convert to m

    x_r, y_r, theta_r = robot_coordinates

    theta_object = theta_r - object_angle

    x_obj = x_r + object_distance * np.cos(theta_object)
    y_obj = y_r + object_distance * np.sin(theta_object)

    return x_obj, y_obj

def calculate_angle_to_coord(target_coords, position_vector):
    """
    Calculates angle required to turn by the robot such that if faces a particular coordinate, given the 
    Parameters:
        target_coords - The coordinates of the point towards which to turn (x0, y0)
        position_vector - the outputs of the triangulation function, the position of the robot (x, y, theta)
    Returns:
        alpha - the angle through which the robot needs to rotate (in the range -pi to pi, +ve to right, -ve to left (matches camera output))
    """
    if not position_vector:
        return None
    x0, y0 = target_coords

    x, y, theta = position_vector

    # 1. Robot's heading vector r = (cos(theta), sin(theta)) - Unit vector pointing in that direction
    bx, by = x0 - x, y0 - y
    cos_t, sin_t = np.cos(theta), np.sin(theta)

    # dot = cos_t*bx + sin_t*by, cross = cos_t*by - sin_t*bx
    # Negate to convert to clockwise (matching camera horizontal_angle convention)
    return -np.arctan2(cos_t * by - sin_t * bx, cos_t * bx + sin_t * by)

def power_function(distance: float) -> float:
    """
    Calculates the power to apply to the motors to drive the robot towards a target.
    Args:
        distance (float): The distance to the target.
    Returns:
        float: The power to apply to the motors.
    """
    distance = distance / 1000  # converts mm to metres
    a = MOTORS.MOTOR_CONSTANT_DISTANCE
    b = MOTORS.MOTOR_CONSTANT
    c = 3  # 1.75 # Controls how quickly function approaches 0, if this number is higher, a smaller distance returns a higher power

    def f(x): return (b / (a**3 + a*c)) * (x**3 + c*x)
    power = min(max(f(distance), -b), b)

    return power

def angle_function(angle: float) -> tuple[float, float]:
    """
    Calculates the power to apply to the motors to turn the robot towards a target.
    Args:
        angle (float): The angle to turn by.
    Returns:
        tuple: A tuple containing the power to apply to the left and right motors.
    """
    a = MOTORS.MAX_ANGLE_POWER
    b = MOTORS.MAX_ANGLE
    c = 3  # Controls how quickly function approaches 0, if this number is higher, a smaller angle returns a higher power

    def f(x): return (a / (b**3 + b*c)) * (x**3 + c*x)
    power = min(max(f(angle), -a), a)

    left_power = power
    right_power = -power

    return left_power, right_power

def calculate_motor_power(angle: float = 0, distance: float = 0) -> tuple[float, float]:
    drive_power = power_function(distance)
    left_turn, right_turn = angle_function(angle)

    left_power = drive_power + left_turn
    right_power = drive_power + right_turn

    left_power = round(left_power, 3)
    right_power = round(right_power, 3)

    return left_power, right_power

# def calculate_turn_time(theta: float):
#     """
#     Calculates the time it takes to turn a certain angle
#     Parameters:
#         - theta : angle to turn to (radians)
#     Returns:
#         - time : time to turn the angle (s)    
#     """
#     theta = abs(theta)

#     # Time to accelerate form 0 to ANGULAR_VELOCITY
#     t_accel = MOTORS.ANGULAR_VELOCITY / MOTORS.ANGULAR_ACCELERATION
#     theta_accel = 0.5 * MOTORS.ANGULAR_ACCELERATION * t_accel**2

#     # Time to decelerate from ANGULAR_VELOCITY to 0
#     t_decel = MOTORS.ANGULAR_VELOCITY / MOTORS.ANGULAR_DECELERATION
#     theta_decel = 0.5 * MOTORS.ANGULAR_DECELERATION * t_decel**2

#     if theta_accel + theta_decel > theta:
#         # If the turn is too short to reach max velocity, use kinematic equation to solve for time
#         t_total = sqrt(
#             2 * theta / (MOTORS.ANGULAR_ACCELERATION + MOTORS.ANGULAR_DECELERATION))
#     else:
#         # Remaining angle to be turned at constant velocity
#         theta_constant = theta - (theta_accel + theta_decel)
#         t_constant = theta_constant / MOTORS.ANGULAR_VELOCITY

#         # Total time is sum of all phases
#         t_total = t_accel + t_constant + t_decel

#     return t_total

def calculate_turn_time(theta: float) -> float:
    """
    Calculates the time it takes to turn a certain angle.

    Parameters:
        theta: angle to turn (radians)

    Returns:
        time to turn the angle (s)
    """
    theta = abs(theta)

    # Use slower turning constants for small turns
    if theta <= MOTORS.SMALL_ANGLE_THRESHOLD:
        w_max = MOTORS.SLOW_ANGULAR_VELOCITY
        a = MOTORS.SLOW_ANGULAR_ACCELERATION
        d = MOTORS.SLOW_ANGULAR_DECELERATION
    else:
        w_max = MOTORS.ANGULAR_VELOCITY
        a = MOTORS.ANGULAR_ACCELERATION
        d = MOTORS.ANGULAR_DECELERATION

    # Time to accelerate to max angular velocity
    t_accel = w_max / a
    theta_accel = 0.5 * a * t_accel**2

    # Time to decelerate from max angular velocity to 0
    t_decel = w_max / d
    theta_decel = 0.5 * d * t_decel**2

    if theta <= theta_accel + theta_decel:
        # Triangular profile: cannot reach max angular velocity
        w_peak = sqrt(2 * theta / ((1 / a) + (1 / d)))
        t_total = (w_peak / a) + (w_peak / d)
    else:
        # Trapezoidal profile: accelerate, constant speed, decelerate
        theta_constant = theta - theta_accel - theta_decel
        t_constant = theta_constant / w_max
        t_total = t_accel + t_constant + t_decel

    return t_total

def calculate_drive_time(distance: float):
    distance = abs(distance)
    distance /= 1000  # Convert mm to m

    # Time to accelerate from 0 to LINEAR_VELOCITY
    t_accel = MOTORS.LINEAR_VELOCITY / MOTORS.LINEAR_ACCELERATION
    d_accel = 0.5 * MOTORS.LINEAR_ACCELERATION * t_accel**2

    # Time to decelerate from LINEAR_VELOCITY to 0
    t_decel = MOTORS.LINEAR_VELOCITY / MOTORS.LINEAR_DECELERATION
    d_decel = 0.5 * MOTORS.LINEAR_DECELERATION * t_decel**2

    if d_accel + d_decel > distance:
        # If the distance is too short to reach max velocity, use kinematic equation to solve for time
        t_total = sqrt(
            2 * distance / (MOTORS.LINEAR_ACCELERATION + MOTORS.LINEAR_DECELERATION))
    else:
        # Remaining distance to be traveled at constant velocity
        d_constant = distance - (d_accel + d_decel)
        t_constant = d_constant / MOTORS.LINEAR_VELOCITY

        # Total time is sum of all phases
        t_total = t_accel + t_constant + t_decel

    return t_total

def robust_mean(data: np.array):
    """Calculate mean values from a set of data points, using the median absolute deviation to discard outliers."""
    # New claude optimised version using numpy vectorisation
    data = np.asarray(data)
    medians = np.median(data, axis=0)
    mad = np.median(np.abs(data - medians), axis=0)

    if data.ndim > 1:
        mask = np.all(np.abs(data - medians) <= MAD_THRESHOLD * mad, axis=1)
    else:
        mask = np.abs(data - medians) <= MAD_THRESHOLD * mad
    cleaned = data[mask]

    if len(cleaned) == 0:
        return medians  # fallback if all points filtered as outliers

    return np.mean(cleaned, axis=0)

    """
    Previous unoptimised version according to Claude
    medians = np.median(data, axis=0)
    # calculates the median absolute deviation
    mad = np.median(np.absolute(data - medians), axis=0)

    cleaned_data = []
    for values in data:
        # if any values in a set of values deviate from the median by more than MAD_THRESHOLD times the median absolute
        # deviation, the entire set is discarded
        if np.all(np.absolute(values - medians) <= MAD_THRESHOLD * mad):
            cleaned_data.append(values)

    return np.mean(np.array(cleaned_data), axis=0)
    """

def find_intersection(x1, y1, r1, x2, y2, r2):
    """
    Finds the intersection point of two circles. Filters out solutions not within the boundary of the arena
    Used in triangulating the position of the robot.
    """
    dx, dy = x2-x1, y2-y1
    d = sqrt(dx*dx+dy*dy)
    if d > r1+r2:
        return None # no solutions, the circles are separate
    if d < abs(r1-r2):
        return None # no solutions because one circle is contained within the other
    if d == 0 and r1 == r2:
        return None # circles are coincident and there are an infinite number of solutions

    a = (r1*r1-r2*r2+d*d)/(2*d)
    h = sqrt(r1*r1-a*a)
    xm = x1 + a*dx/d
    ym = y1 + a*dy/d
    xs1 = xm + h*dy/d
    xs2 = xm - h*dy/d
    ys1 = ym - h*dx/d
    ys2 = ym + h*dx/d

    if -ARENA.ARENA_HALF_EDGE <= xs1 <= ARENA.ARENA_HALF_EDGE and -ARENA.ARENA_HALF_EDGE <= ys1 <= ARENA.ARENA_HALF_EDGE:
        return (xs1, ys1)
    if -ARENA.ARENA_HALF_EDGE <= xs2 <= ARENA.ARENA_HALF_EDGE and -ARENA.ARENA_HALF_EDGE <= ys2 <= ARENA.ARENA_HALF_EDGE:
        return (xs2, ys2)

    return None

def triangulation(markers):
    """
    Triangulates the position of the robot
    """

    data = []
    for marker in markers:
        if marker.id in WALLS.WALL_MARKER_IDS:
            marker_position = WALLS.WALL_MARKER_POSITIONS[marker.id]
            data.append(
                (marker_position[0], marker_position[1], marker.position.distance/1000))

    if len(data) < 2:
        return None

    combs = tuple(combinations(range(len(data)), 2))

    x_coordinates = []
    y_coordinates = []

    for comb in combs:

        first_point = data[comb[0]]
        second_point = data[comb[1]]

        intersection = find_intersection(*first_point, *second_point)

        if intersection:
            x_coordinates.append(intersection[0])
            y_coordinates.append(intersection[1])

    if not x_coordinates:
        return None

    x = robust_mean(np.array(x_coordinates))
    y = robust_mean(np.array(y_coordinates))

    theta_data = []

    for marker in markers:
        if marker.id in WALLS.WALL_MARKER_IDS:

            marker_position = WALLS.WALL_MARKER_POSITIONS[marker.id]
            relative_position = (
                marker_position[0] - x, marker_position[1] - y)

            alpha = np.arctan2(
                abs(relative_position[1]), abs(relative_position[0]))

            if relative_position[0] > 0:
                if relative_position[1] < 0:
                    alpha = (2 * np.pi) - alpha
            else:
                if relative_position[1] > 0:
                    alpha = np.pi - alpha
                else:
                    alpha = np.pi + alpha

            theta = alpha + marker.position.horizontal_angle

            theta_data.append(np.arctan2(np.sin(theta), np.cos(theta)))

    if not theta_data:
        return None

    theta = robust_mean(np.array(theta_data))

    return x, y, theta

def pose_estimation(markers):
    """
    Calculates the position of the robot from an individual marker
    using the marker's position, distance from robot, yaw, and horizontal angle to robot.
    
    :param markers: output of robot.camera.see()
    """

    def rotate_line(line, angle, centre):
        """
        Rotates a line Ax + By = C by angle (radians) anticlockwise
        about a point centre:(X, Y)
        """
        a, b, c = line
        X, Y = centre

        A = a*np.cos(angle) + b*np.sin(angle)
        B = -a*np.sin(angle) + b*np.cos(angle)
        C = c - a*X - b*Y + A*X + B*Y

        return (A, B, C)

    markers = [m for m in markers if m.id in WALLS.WALL_MARKER_IDS]

    if not markers:
        return None

    x_pos = []
    y_pos = []
    thetas = []

    for marker in markers:
        x_m, y_m = WALLS.WALL_MARKER_POSITIONS[marker.id]
        psi = marker.orientation.yaw
        alpha = marker.position.horizontal_angle
        d = marker.position.distance / 1000

        # Circle is defined as (A, B, C) where (x-A)^2 + (y-B)^2 = C^2
        circle = (x_m, y_m, d)
        # Straight line is defined as (A, B, C) where Ax + By = C
        match marker.id:
            case i if i in range(0, 5):
                wall_line = (0, 1, -2.2875)
                theta_wall = -np.pi / 2
            case i if i in range(5, 10):
                wall_line = (1, 0, -2.2875)
                theta_wall = np.pi
            case i if i in range(10, 15):
                wall_line = (0, 1, 2.2875)
                theta_wall = np.pi / 2
            case i if i in range(15, 20):
                wall_line = (1, 0, 2.2875)
                theta_wall = 0
            case _: continue

        t = theta_wall + psi
        thetas.append(np.arctan2(np.sin(t), np.cos(t)))

        # Rotate wall line by yaw, to get line parallel to robot, going through marker
        par_line = rotate_line(wall_line, psi, (x_m, y_m))
        # Rotate parallel line by -horizontal_angle (as anti-clockwise), to get the tangent line shifted to go through the marker
        tan_line = rotate_line(par_line, -alpha, (x_m, y_m))

        # Solve for point(s) on circle with same gradient as tangent line
        m = -tan_line[0] / tan_line[1]
        k = -1 / m

        # More optimised quadratic solving
        # Direct quadratic solution: (1+k²)(x - x_m)² = d²  →  x = x_m ± d/√(1+k²)
        half_range = circle[2] / np.sqrt(1.0 + k * k)
        x_vals = (circle[0] + half_range, circle[0] - half_range)
        
        """
        Previous unoptimised version
        # Quadratic coefficients
        Aq = 1 + k**2
        Bq = -2 * circle[0] * (1 + k**2)
        Cq = circle[0]**2 * (1 + k**2) - circle[2]**2

        x_vals = np.roots([Aq, Bq, Cq])
        """
        
        y_vals = map(
            lambda x: circle[1] + k * (x - circle[0]),
            x_vals
        )
        points = list(zip(x_vals, y_vals))
        points = ((points[1][0], points[0][1]), (points[0][0], points[1][1]))
        inside = next(
            (p for p in points if abs(p[0]) <= 2.2875 and abs(p[1]) <= 2.2875),
            min(points, key=lambda p: p[0]**2 + p[1]**2)
        )
        x_pos.append(inside[0])
        y_pos.append(inside[1])

    if not x_pos:
        return None

    x = robust_mean(x_pos)
    y = robust_mean(y_pos)
    theta = np.arctan2(
        robust_mean(np.array([np.sin(t) for t in thetas])),
        robust_mean(np.array([np.cos(t) for t in thetas]))
    )

    return (x, y, theta)

def average_positions(*positions):
    """Averages multiple position estimates together, using the median absolute deviation to discard outliers."""
    x = robust_mean(np.array([p[0] for p in positions]))
    y = robust_mean(np.array([p[1] for p in positions]))
    theta = np.arctan2(
        robust_mean(np.array([np.sin(p[2]) for p in positions])),
        robust_mean(np.array([np.cos(p[2]) for p in positions]))
    )
    return (x, y, theta)

def calculate_box_height(marker, floor_true = False):
    """
    Takes in the entire marker object
    Calculates the height of a box using the distance to the marker and the vertical angle of the marker.
    Accounts for the camera height above the ground and the decline angle of the camera.
    Returns the height of the centre of the box above the ground in metres.
    """
    # if marker.id in WALLS.WALL_MARKER_IDS:
    #     return 0.05 + 0.178/2  # TODO change to the height of the wall markers' centre above the ground
    """
    DO EVERYTHING IN MILLIMETRES
    """
    distance = marker.position.distance # mm
    theta = marker.position.vertical_angle

    # Adjust vertical angle by camera decline (subtract since camera is angled down)
    adjusted_theta = theta - CAMERA.CAMERA_DECLINE

    height_below_camera = distance * np.tan(adjusted_theta)
    calculated_height = CAMERA.CAMERA_HEIGHT + height_below_camera # mm

    # print('Calculated height:', calculated_height)
    
    if marker.id in SAMPLES.SAMPLE_IDS:
        if calculated_height <= (CENTRE.HEIGHT * 1000):
            # Less than the centre height, 
            height = SAMPLES.HALF_LENGTH / 1000
            floor = True
        else:
            height = CENTRE.HEIGHT + SAMPLES.HALF_LENGTH / 1000
            floor = False
    # return is in m
    if floor_true == True:
        return floor
    else:
        return height

def check_sample_zone(position_vector, marker):
    """
    Takes in the robot's position and the marker object of the sample to be checked
    Checks if a sample is currently placed in a zone by checking if the position of the sample is within the boundaries of the zone.
    Returns the zone number if the sample is in a zone, and -1 if it is not.    
    """
    sx, sy = calculate_object_coords(
        position_vector, marker.position.distance, marker.position.horizontal_angle)

    tolerance = 0.15  # metres TODO tune this value
    for zone_num, zone_boundary in enumerate(ZONES.BOUNDARIES):
        x0, x1 = zone_boundary[0][0], zone_boundary[1][0] - np.sign(zone_boundary[1][0]) * tolerance
        y0, y1 = zone_boundary[0][1], zone_boundary[1][1] - np.sign(zone_boundary[1][1]) * tolerance
        if min(x0, x1) <= sx <= max(x0, x1) and min(y0, y1) <= sy <= max(y0, y1):
            return zone_num
    return -1

def _segment_intersects_aabb(x0, y0, x1, y1, min_x, max_x, min_y, max_y):
    dx = x1 - x0
    dy = y1 - y0

    t_enter = 0.0
    t_exit = 1.0

    for p, q in (
        (-dx, x0 - min_x),
        ( dx, max_x - x0),
        (-dy, y0 - min_y),
        ( dy, max_y - y0),
    ):
        if p == 0.0:
            if q < 0.0:
                return False
        else:
            t = q / p
            if p < 0.0:
                if t > t_exit:
                    return False
                if t > t_enter:
                    t_enter = t
            else:
                if t < t_enter:
                    return False
                if t < t_exit:
                    t_exit = t

    return True

def check_straight_line(position_vector, zone_num=None, target_coords=None):
    CENTRE_MIN_X = CENTRE.INFLATED_CORNERS[1][0]
    CENTRE_MAX_X = CENTRE.INFLATED_CORNERS[0][0]
    CENTRE_MIN_Y = CENTRE.INFLATED_CORNERS[1][1]
    CENTRE_MAX_Y = CENTRE.INFLATED_CORNERS[0][1]
    
    if (zone_num is None) == (target_coords is None):
        raise ValueError("Provide exactly one of zone_num or target_coords.")

    rx = float(position_vector[0])
    ry = float(position_vector[1])

    if target_coords is not None:
        tx = float(target_coords[0])
        ty = float(target_coords[1])

        return not _segment_intersects_aabb(
            rx, ry, tx, ty,
            CENTRE_MIN_X, CENTRE_MAX_X,
            CENTRE_MIN_Y, CENTRE_MAX_Y
        )

    zone_boundaries = ZONES.BOUNDARIES[zone_num]

    x_a = float(zone_boundaries[0][0])
    y_a = float(zone_boundaries[0][1])
    x_b = float(zone_boundaries[1][0])
    y_b = float(zone_boundaries[1][1])

    zone_min_x = x_a if x_a < x_b else x_b
    zone_max_x = x_b if x_a < x_b else x_a
    zone_min_y = y_a if y_a < y_b else y_b
    zone_max_y = y_b if y_a < y_b else y_a

    centre_x = float(ZONES.CENTRE_COORDINATES[zone_num][0])
    centre_y = float(ZONES.CENTRE_COORDINATES[zone_num][1])

    if not _segment_intersects_aabb(
        rx, ry, centre_x, centre_y,
        CENTRE_MIN_X, CENTRE_MAX_X,
        CENTRE_MIN_Y, CENTRE_MAX_Y
    ):
        return True

    candidates = (
        (zone_min_x, zone_min_y),
        (zone_min_x, zone_max_y),
        (zone_max_x, zone_min_y),
        (zone_max_x, zone_max_y),
    )

    for tx, ty in candidates:
        if not _segment_intersects_aabb(
            rx, ry, tx, ty,
            CENTRE_MIN_X, CENTRE_MAX_X,
            CENTRE_MIN_Y, CENTRE_MAX_Y
        ):
            return True

    return False

def calculate_anchor_point(target_zone, position_vector):
    """
    Calculates the best anchor point to move to in order to get a straight line path to the centre of the zone.
    Chooses the corner of the zone that is closest to the robot's current position.
    """
    if check_straight_line(position_vector, zone_num=target_zone):
        return None

    # Needs more nuance to ensure that we are not selecting the opposite corner to the one we are going to. 
    useful_anchors = ANCHOR_POINTS.ZONE_ANCHOR_POINTS[target_zone]

    x = float(position_vector[0])
    y = float(position_vector[1])
    
    target_anchors = [p for p in useful_anchors if check_straight_line(position_vector=position_vector, target_coords=ANCHOR_POINTS.CORNER_ANCHOR_POINTS[p])]
    print(target_anchors)
    if len(target_anchors) == 1:
      return target_anchors[0]
    elif len(target_anchors) == 2:
            return min(target_anchors, key=lambda p: distance(ANCHOR_POINTS.CORNER_ANCHOR_POINTS[p], (x, y)))
    else:
      print("Both anchor points are blocked, which is some BS fr")
      return -1

pose = (1.892, -1.126, -1.544)
calculate_anchor_point(2, pose)