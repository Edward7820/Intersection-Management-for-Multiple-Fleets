import math
from typing import Tuple

def vector_length(x: float, y: float):
    return math.sqrt(x**2+y**2)

def quadratic(a:float, b:float, c:float):
    #solve ax^2+bx+c=0 and return the bigger root
    if a==0:
        return -c/b
    key = b**2-4*a*c
    root = None
    if key>0:
        root=(-b+math.sqrt(key))/(2*a)
    elif key==0:
        root=-b/(2*a)
    return root

def euclidean_dist(x1: Tuple[float], x2: Tuple[float]):
    return math.sqrt((x1[0]-x2[0])**2 + (x1[1]-x2[1])**2)

def vector_add(x1: Tuple[float], x2: Tuple[float]):
    return (x1[0]+x2[0], x1[1]+x2[1])

def vector_mul_scalar(x: Tuple[float], c: float):
    return (x[0]*c, x[1]*c)

def vector_sub(x1: Tuple[float], x2: Tuple[float]):
    return (x1[0]-x2[0], x1[1]-x2[1])

def inner_product(x1: Tuple[float], x2: Tuple[float]):
    return x1[0]*x2[0] + x1[1]*x2[1]

def projection(x1: Tuple[float], x2: Tuple[float]):
    # project x1 onto the direction of x2
    scalar = inner_product(x1, x2)/vector_length(x2[0],x2[1])
    return vector_mul_scalar(x2, scalar)

def arrival_time(distance: float, speed: float, acceleration: float):
    t = quadratic(acceleration/2, speed, -distance)
    assert t != None
    assert t >= 0
    return t

def arrival_time_for_zone(cur_loc: Tuple[float], speed, conflict_zones, zone_idx_list, idx):
    # conflict_zones: a list of tuples (x_min, y_min, x_max, y_max)
    # print(zone_idx_list, idx)
    zone = conflict_zones[idx]
    north_point = ((zone[0]+zone[2])/2, zone[3])
    south_point = ((zone[0]+zone[2])/2, zone[1])
    east_point = (zone[2],(zone[1]+zone[3])/2)
    west_point = (zone[0],(zone[1]+zone[3])/2)
    if zone_idx_list[idx] == 0 and idx == 0:
        dist = euclidean_dist(west_point, cur_loc)
    elif zone_idx_list[idx] == 1 and idx >= 1:
        dist = euclidean_dist(west_point, cur_loc)
    elif  zone_idx_list[idx] == 2 and idx == 0:
        dist = euclidean_dist(east_point, cur_loc)
    elif zone_idx_list[idx] == 3 and idx >= 1:
        dist = euclidean_dist(east_point, cur_loc)
    elif zone_idx_list[idx] == 1 and idx == 0:
        dist = euclidean_dist(south_point, cur_loc)
    elif zone_idx_list[idx] == 2 and idx >= 1:
        dist = euclidean_dist(south_point, cur_loc)
    elif zone_idx_list[idx] == 3 and idx == 0:
        dist = euclidean_dist(north_point, cur_loc)
    elif zone_idx_list[idx] == 0 and idx >= 1:
        dist = euclidean_dist(north_point, cur_loc)
    return dist/speed

def get_conflict_zone_idx(lane_id: int, des_lane_id: int):
    ## conflict_zones: a list of tuples (x_min, y_min, x_max, y_max)
    '''
    if lane_id == 0 and des_lane_id == 1:
        return [0]
    if lane_id == 0 and des_lane_id == 2:
        return [0,1]
    if lane_id == 0 and des_lane_id == 3:
        return [0,1,2]
    if lane_id == 1 and des_lane_id == 0:
        return [1,2,3]
    if lane_id == 1 and des_lane_id == 2:
        return [1]
    if lane_id == 1 and des_lane_id == 3:
        return [1,2]
    '''
    conflict_zones = []
    for i in range((des_lane_id-lane_id)%4):
        conflict_zones.append((lane_id+i)%4)
    return conflict_zones

def get_cur_conflict_zone_id(conflict_zones, cur_loc):
    for idx, conflict_zone in enumerate(conflict_zones):
        x_min = conflict_zone[0]
        x_max = conflict_zone[2]
        y_min = conflict_zone[1]
        y_max = conflict_zone[3]
        if cur_loc[0] >= x_min and cur_loc[0] <= x_max and cur_loc[1] >= y_min and cur_loc[1] <= y_max:
            return idx
    return -1

def get_min_arrival_time(conflict_zones, lane_id, des_lane_id, location, speed):
    zone_size = abs(conflict_zones[0][2]-conflict_zones[0][0])
    zone_idx_list = get_conflict_zone_idx(lane_id, des_lane_id)
    min_arrival_time = arrival_time_for_zone(location,speed,conflict_zones,zone_idx_list,0)
    min_arrival_time += len(zone_idx_list)*zone_size/speed
    return min_arrival_time