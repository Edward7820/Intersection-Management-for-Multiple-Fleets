import sys
import time
import argparse
import itertools
import math
from datetime import datetime
from typing import List, Dict
import copy
from . import math_utils

FLEET_LENGTH = 5
FLEET_DIRECTION = ['right', 'down'] 
# right: toward the direction of positive x-axis
# down: toward the direction of negative y-axis
MAX_ACCELERATE = 1 #max acceleration of each vehicle

def arrival_time(distance: float, speed: float, acceleration: float):
    t = math_utils.quadratic(acceleration/2, speed, -distance)
    return t

class Scheduler():
    def __init__(self, conflict_zone: list, states: list):
        self.conflict_zone = conflict_zone
        # type: a list of four two-tuples indicating the four position of four vertices of conflict zones 
        # Assume that the conflict zone is a square
        self.states = states # state of all vehicles
        '''
        Within each fleet, 
        states[k], states[k+1], ..., states[k+FLEET_LENGTH]
           ^                            ^
           |                            |
        (state of leading vehicle)   (state of last vehicle)
        '''
        self.first_fleet = None # the fleet id of first fleet to go (0 or 1)
        self.schedule = None # a list of time (type: float) list[float]

        self.x_min = min([conflict_zone[0][0], conflict_zone[1][0], conflict_zone[2][0], conflict_zone[3][0]])  
        self.x_max = max([conflict_zone[0][0], conflict_zone[1][0], conflict_zone[2][0], conflict_zone[3][0]])
        self.y_min = min([conflict_zone[0][1], conflict_zone[1][1], conflict_zone[2][1], conflict_zone[3][1]])
        self.y_max = max([conflict_zone[0][1], conflict_zone[1][1], conflict_zone[2][1], conflict_zone[3][1]])
        #range of the conflict zone
        assert(self.x_max-self.x_min==self.y_max-self.y_min)

    def loss_function(self, first_fleet, schedule):
        if (len(schedule)<2):
            return None
        states = self.states
        origin_arrival_time = [None]*(2*FLEET_LENGTH) #original expected time taken for each vehicle to leave the conflict zone
        remain_distance = [None]*(2*FLEET_LENGTH) #remaining distance to leave the conflict zone
        for vid in range(2*FLEET_LENGTH):
            direction = FLEET_DIRECTION[vid//FLEET_LENGTH]
            if direction=='left':
                remain_distance[vid]=abs(states[vid]['location'][0]-self.x_min)
            elif direction=='right':
                remain_distance[vid]=abs(states[vid]['location'][0]-self.x_max)
            elif direction=='top':
                remain_distance[vid]=abs(states[vid]['location'][1]-self.y_max)
            else:
                remain_distance[vid]=abs(states[vid]['location'][1]-self.y_min)
            origin_arrival_time[vid]=arrival_time(remain_distance[vid], states[vid]['speed'], states[vid]['acceleration'])
        
        exp_arrival_time = origin_arrival_time.copy()
        #expected arrival time of each vehicle under the schedule
        
        exp_speed = [states[vid]['speed'] for vid in range(2*FLEET_LENGTH)]
        #expected speed

        '''
        For the fleet which is not scheduled first, 
        vehicles within that fleet need to slow down to wait for another fleet.
        '''
        forward_distance = remain_distance[(1-first_fleet)*FLEET_LENGTH]-(self.x_max-self.x_min)
        #forward distance of the leading vehicle that did not cross the intersection
        
        slow_down_time = schedule[0]
        #slow down to wait for another fleet

        for vid in range((1-first_fleet)*FLEET_LENGTH,(2-first_fleet)*FLEET_LENGTH):
            remain_distance[vid] -= forward_distance
            exp_speed[vid] = 2*forward_distance/slow_down_time-exp_speed[vid]
            if (exp_speed[vid]<0):
                #the vehicle needs to stop
                exp_speed[vid] = 0
            exp_arrival_time[vid] = schedule[0]+arrival_time(remain_distance[vid],exp_speed[vid],MAX_ACCELERATE)
        #print(exp_arrival_time,exp_speed)

        scheduled = [False]*(2*FLEET_LENGTH) #whether a vehicle is scheduled
        fleet_scheduled = [False]*2 #whether all vehicles within a fleet are scheduled
        for i,deadline in enumerate(schedule):
            if i%2==0:
                turn = first_fleet
            else:
                turn = 1-first_fleet

            not_scheduled_list = []
            for vid in range(turn*FLEET_LENGTH, (turn+1)*FLEET_LENGTH):
                if scheduled[vid]:
                    continue
                elif exp_arrival_time[vid]<=deadline:
                    #these vehicles can cross the intersection within this time slot             
                    remain_distance[vid]=0
                    scheduled[vid]=True
                    #print(f"vehicle {vid} is supposed to cross the intersection in time slot {i}, and the expected arrival time is {exp_arrival_time[vid]}.")
                else:
                    #the vehicle can not be scheduled in this time slot
                    not_scheduled_list.append(vid)
            if i>=len(schedule)-2 and len(not_scheduled_list)>0:
                return None #unreasonable schedule
            if (len(not_scheduled_list)==0):
                fleet_scheduled[turn] = True
            if (fleet_scheduled[0] and fleet_scheduled[1]):
                ##calculate average delay and return
                total_delay = 0
                for i in range(2*FLEET_LENGTH):
                    total_delay += (exp_arrival_time[i]-origin_arrival_time[i])
                return total_delay/(2*FLEET_LENGTH)
            if fleet_scheduled[turn]:
                continue
            
            '''
            For the remaining vehicle, 
            assume that the gap between two vehicles remain the same
            '''
            forward_distance = remain_distance[not_scheduled_list[0]]-(self.x_max-self.x_min)
            #forward distance of the leading vehicle that did not cross the intersection
            
            if i==0:
                slow_down_time = schedule[i+1]
            else:
                slow_down_time = schedule[i+1]-schedule[i-1] 
            #slow down to wait for another fleet
            
            for vid in not_scheduled_list:
                remain_distance[vid] -= forward_distance
                exp_speed[vid] = 2*forward_distance/slow_down_time-exp_speed[vid]
                if (exp_speed[vid]<0):
                    #the vehicle needs to stop
                    exp_speed[vid] = 0
                exp_arrival_time[vid] = schedule[i+1]+arrival_time(remain_distance[vid],exp_speed[vid],MAX_ACCELERATE)
            #print(exp_arrival_time,exp_speed)

    def get_schedule(self):
        return self.first_fleet, self.schedule

    def propose_best_schedule(self, schedule_granularity):
        # Schedule grannularity should be greater than the time a vehicle needs to cross the conflict zone
        # Iterate through some possible schedules and find the best one
        schedule = [None]*(2*FLEET_LENGTH)
        first_fleet = None
        best_schedule = None
        min_loss = 10000

        def search_schedule(slot_id, quota):
            if slot_id==2*FLEET_LENGTH or quota<=0:
                for fleetid in range(2):
                    loss = self.loss_function(fleetid,schedule[:slot_id])
                    if (loss is not None) and (loss < min_loss):
                        nonlocal best_schedule
                        nonlocal first_fleet
                        best_schedule = schedule[:slot_id]
                        first_fleet = fleetid
                        #print(first_fleet,best_schedule)
                        #print(loss)
                return
            for t in range(1, max(FLEET_LENGTH+1, quota)):
                if slot_id==0:
                    schedule[slot_id] = t*schedule_granularity
                else:
                    schedule[slot_id] = schedule[slot_id-1]+t*schedule_granularity
                search_schedule(slot_id+1, quota-t)
        
        search_schedule(0,2*FLEET_LENGTH)
        if best_schedule==None:
            self.schedule = None
            self.first_fleet = None
        else:
            self.schedule = best_schedule.copy()
            self.first_fleet = first_fleet
 