import math
from typing import List, Dict
from copy import *
import random
import queue
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

    def get_all_siblings(self):
        if self.parent == None:
            return [self]
        # calculate its depth
        node = self.parent
        root_node = self
        depth = 0
        while (node != None):
            depth += 1
            root_node = node
            node = node.parent

        # return all nodes with the same depth
        node = root_node
        pool = queue.Queue()
        pool.put(node)
        cur_depth = 0
        while (cur_depth < depth): # not root node
            last_pool = []
            while not pool.empty():
                last_pool.append(pool.get())
            for node in last_pool:
                for child in node.children:
                    pool.put(child)
            cur_depth += 1
        
        all_siblings = []
        while not pool.empty():
            all_siblings.append(pool.get())
        return all_siblings

    def select(self):
        best_child = None
        best_ucb = -math.inf

        for child in self.children:
            assert child.visits > 0 and self.visits > 0
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
        '''
        if self.parent != None:
            all_siblings = self.get_all_siblings()
            all_total_delay = [child.total_delay for child in all_siblings]
            all_best_total_delay = [child.best_total_delay for child in all_siblings]
            all_total_delay = list(filter(lambda x: x != -1, all_total_delay))
            all_best_total_delay = list(filter(lambda x: x != -1, all_best_total_delay))
            max_delay = max(all_total_delay)
            min_delay = min(all_total_delay)
            if max_delay-min_delay>0:
                score1 = 1-(self.total_delay-min_delay)/(max_delay-min_delay)
            else:
                score1 = 1
            max_delay = max(all_best_total_delay)
            min_delay = min(all_best_total_delay)
            if max_delay-min_delay>0:
                score2 = 1-(self.best_total_delay-min_delay)/(max_delay-min_delay)
            else:
                score2 = 1
            return (score1+score2)/2
        '''
        if self.parent != None:
            return self.best_total_delay + 5*self.total_delay/len(self.passing_order)
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
        return math_utils.euclidean_dist((0,0),self.get_state(veh)["location"])
    
    def get_possible_next_veh(self, node):
        remain_veh = list(filter(lambda veh: veh not in node.passing_order, self.all_veh))
        if len(remain_veh) == 0:
            return []
        possible_next_veh = []
        for veh in remain_veh:
            if veh[3] == 0 or (veh[0], veh[1], veh[2], veh[3]-1) in node.passing_order:
                possible_next_veh.append(veh)
        return possible_next_veh
    
    def print_tree(self):
        def print_node(node, depth):
            print("-"*depth, end="")
            print(node.passing_order)
            for child in node.children:
                print_node(child, depth+1)

        print_node(self.root, 0)

    def simulate(self, node):
        states_list = [self.get_state(veh) for veh in node.passing_order]
        total_delay, _ = Simulator.simulate_passing_order(node.passing_order,self.conflict_zones,states_list,self.safety_gap)
        node.total_delay = total_delay

        remain_veh = list(filter(lambda veh: veh not in node.passing_order, self.all_veh))
        remain_veh = sorted(remain_veh, key=lambda veh: self.get_dist_to_center(veh))
        complete_passing_order = node.passing_order + remain_veh
        states_list = [self.get_state(veh) for veh in complete_passing_order]
        total_delay_2, _ = Simulator.simulate_passing_order(complete_passing_order,self.conflict_zones,states_list,self.safety_gap)
        node.best_total_delay  = total_delay_2
        return node.normalize_delay()

    def search(self, num_iter):
        for _ in range(num_iter):
            node = self.select_node()
            node = self.expand_node(node)
            if node:
                score = self.simulate(node)
                node.backpropagate(score)
            self.print_tree()
        
        ## get best passing order
        node = self.root
        while node.children:
            best_score = -1
            best_node = None
            for child in node.children:
                if child.score > best_score:
                    best_score = child.score
                    best_node = child
            node = best_node
        return node.passing_order


    def select_node(self):
        node = self.root
        while node.children:
            if not all(child.visits for child in node.children):
                return node
            
            node = node.select()
        return node

    def expand_node(self, node):
        if not node.children:
            possible_next_veh = self.get_possible_next_veh(node)
            if not possible_next_veh: # leaf node:
                return node
            for veh in possible_next_veh:
                next_order = deepcopy(node.passing_order)
                next_order.append(veh)
                node.expand(next_order)
        assert node.children

        unvisited_children = list(filter(lambda x: x.visits == 0, node.children))
        assert len(unvisited_children) > 0
        return random.choice(unvisited_children)

 