import math
from datetime import datetime
from typing import List, Dict
from copy import *
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
        if self.parent != None:
            all_total_delay = [child.total_delay for child in self.parent.children]
            all_best_total_delay = [child.best_total_delay for child in self.parent.children]
            max_delay = max(all_total_delay)
            min_delay = min(all_total_delay)
            if max_delay-min_delay>0:
                score1 = 1-(self.total_delay-min_delay)/(max_delay-min_delay)
            else:
                score1 = 0
            max_delay = max(all_best_total_delay)
            min_delay = min(all_best_total_delay)
            if max_delay-min_delay>0:
                score2 = 1-(self.best_total_delay-min_delay)/(max_delay-min_delay)
            else:
                score2 = 0
            return (score1+score2)/2
        assert 0

    def backpropagate(self, score):
        self.visits += 1
        self.score = score
        if self.parent != None:
            self.parent.backpropagate(score)


class Scheduler():
    def __init__(self, conflict_zones: list, states: dict, safety_gap: float):
        ## states: (lane_id, fleet_id, veh_id) -> state
        self.conflict_zones = conflict_zones
        self.states = states
        self.safety_gap = safety_gap
        self.all_veh = []
        for veh in states.keys():
            self.all_veh.append((veh[0],self.states[veh]["des_lane_id"],veh[1],veh[2]))
        self.num_veh = len(states.keys())
        self.root = Node([])

    def get_state(self, veh):
        return self.states[(veh[0],veh[2],veh[3])]
    
    def get_dist_to_center(self, veh):
        return math_utils.euclidean_dist((0,0),self.get_state[veh]["location"])

    def simulate(self, node):
        states_list = [self.get_state(veh) for veh in node.passing_order]
        total_delay, _ = Simulator.simulate_passing_order(node.passing_order,self.conflict_zones,states_list,self.safety_gap)
        node.total_delay = total_delay

        remain_veh = list(filter(lambda veh: veh in node.passing_order, self.all_veh))
        remain_veh = sorted(remain_veh, key=lambda veh: self.get_dist_to_center(veh))
        complete_passing_order = node.passing_order + remain_veh
        states_list = [self.get_state(veh) for veh in complete_passing_order]
        total_delay_2, _ = Simulator.simulate_passing_order(complete_passing_order,self.conflict_zones,states_list,self.safety_gap)
        node.best_total_delay  = total_delay_2
        return node.normalize_delay()

    def search(self, num_iter):
        for _ in range(num_iter):
            node = self.select_node()
            if node:
                score = self.simulate(node)
                node.backpropagate(score)
        
        ## TODO: get best passing order

    def select_node(self):
        node = self.root
        while node.children:
            if not all(child.visits for child in node.children):
                return self.expand_node(node)
            
            node = node.select()
        return node

    def expand_node(self, node):
        raise NotImplementedError

 