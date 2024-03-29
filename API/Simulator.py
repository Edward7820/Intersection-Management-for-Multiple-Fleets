import math
from typing import List, Dict
import copy
from . import math_utils
CONFLICT_ZONE_SIZE = 4

def simulate_passing_order(order: list[tuple], conflict_zones: list[tuple], states: list[dict], safety_gap: float, lane_id, fleet_id, alpha):
    ## order: a list of tuples (lane_id, des_lane_id, fleet_id, vehicle_id)
    ## conflict_zones: a list of tuples (x_min, y_min, x_max, y_max)
    ## states: a list of dict {"location": (...), "velocity": (...), "acceleration": (...)}
    ## safety_gap: min safety gap between two consecutive vehicles passing through the same conflict zone
    ## Return t_assign: list, total_delay: float
    t_assign = []
    t_max = [None]*len(conflict_zones) ## the latest time that each conflict zone has been occupied
    for idx, veh in enumerate(order):
        t_assign.append([-1]*len(conflict_zones))
        zone_idx_list = math_utils.get_conflict_zone_idx(veh[0], veh[1])
        for zone_idx_idx, zone_idx in enumerate(zone_idx_list):
            location = states[idx]['location']
            velocity = states[idx]['velocity']
            speed = math_utils.vector_length(velocity[0],velocity[1])
            if zone_idx_idx == 0:
                t_min = math_utils.arrival_time_for_zone(location,speed,conflict_zones,zone_idx_list,zone_idx_idx)
            else:
                t_min = t_assign[-1][zone_idx_list[zone_idx_idx-1]] + 2*CONFLICT_ZONE_SIZE/speed
            if t_max[zone_idx] == None:
                t_assign[-1][zone_idx] = t_min
            else:
                t_assign[-1][zone_idx] = max(t_min, t_max[zone_idx] + safety_gap + 2*CONFLICT_ZONE_SIZE/speed)
                if t_assign[-1][zone_idx] > t_min:
                    t_assign[-1][zone_idx_list[zone_idx_idx-1]] = t_max[zone_idx] + safety_gap
        for zone_idx in zone_idx_list:
            t_max[zone_idx] = t_assign[-1][zone_idx]
    assert len(t_assign) == len(order)

    total_delay = 0
    for idx in range(len(order)):
        speed = math_utils.vector_length(states[idx]['velocity'][0],states[idx]['velocity'][1])
        location = states[idx]['location']
        t_min =math_utils.get_min_arrival_time(conflict_zones,order[idx][0],order[idx][1],location,speed)
        if order[idx][0] == lane_id and order[idx][2] == fleet_id:
            total_delay += alpha * (max(t_assign[idx]) - t_min)
        else:
            total_delay += (max(t_assign[idx]) - t_min)
    return total_delay, t_assign


