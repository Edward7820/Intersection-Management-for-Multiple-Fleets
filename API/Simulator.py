import sys
import time
import argparse
import itertools
import math
from typing import List, Dict
import copy
from . import math_utils

def simulate_passing_order(order: list[tuple], conflict_zones: list[tuple], states: list[dict], safety_gap: float):
    ## order: a list of tuples (lane_id, des_lane_id, fleet_id, vehicle_id)
    ## conflict_zones: a list of tuples (x_min, y_min, x_max, y_max)
    ## states: a list of dict {"location": (...), "velocity": (...), "acceleration": (...)}
    ## safety_gap: min safety gap between two consecutive vehicles passing through the same conflict zone
    ## Return t_assign: list, total_delay: float
    t_assign = []
    t_max = [None]*4 ## the largest arrival time that each conflict zone has been occupied
    for veh in order:
        t_assign.append([])
        for zone_idx in math_utils.get_conflict_zone_idx():
            if t_max[zone_idx] == None:
                pass



