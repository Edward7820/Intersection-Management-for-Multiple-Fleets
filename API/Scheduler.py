import math
from datetime import datetime
from typing import List, Dict
import copy
from . import math_utils
from . import Simulator

FLEET_LENGTH = 5
FLEET_DIRECTION = ['right', 'down'] 
# right: toward the direction of positive x-axis
# down: toward the direction of negative y-axis
MAX_ACCELERATE = 1 #max acceleration of each vehicle

def arrival_time(distance: float, speed: float, acceleration: float):
    t = math_utils.quadratic(acceleration/2, speed, -distance)
    return t

class Node():
    def __init__(self, passing_order):
        self.passing_order = passing_order
        self.parent = None
        self.children = []
        self.visits = 0
        self.score = 0
        self.total_delay = -1
        self.best_total_delay = -1

    def select(self):
        best_child = None
        best_ucb = -math.inf

        for child in self.children():
            ucb = child.score + math.sqrt(math.log(self.visits)/child.visits)
            if ucb > best_ucb:
                best_ucb = ucb
                best_child = child
        return best_child
    
    def expand(self, passing_order):
        new_child = Node(passing_order)
        self.children.append(new_child)
        new_child.parent = self

    def normalize_delay(self):
        raise NotImplementedError

    def backpropagate(self, score):
        self.visits += 1
        self.score = score
        if self.parent != None:
            self.parent.backpropagate(score)


class Scheduler():
    def __init__(self, conflict_zones: list, states: dict):
        ## states: (lane_id, fleet_id, veh_id) -> state
        self.conflict_zones = conflict_zones
        self.states = states
        self.all_veh = list(states.keys())
        self.num_veh = len(states.keys())
        self.root = Node([])

    def expandable(self, node):
        #return len(node.passing_order) + len(node.children)


 