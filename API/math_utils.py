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