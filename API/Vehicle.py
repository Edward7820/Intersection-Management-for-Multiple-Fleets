import sys
import time
import argparse
import itertools
import json
import zenoh
import math
from zenoh import config
from datetime import datetime
from zenoh import Reliability, Sample, session
from . import math_utils
from math_utils import euclidean_dist, get_conflict_zone_idx, get_min_arrival_time, vector_length
from . import Scheduler
from . import Simulator
from typing import List, Tuple
from Scheduler import Scheduler
CONFLICT_ZONES = [(0,0,4,4),(-4,0,0,4),(-4,-4,0,0),(0,-4,4,0)]


class MyVehicle():
    def __init__(self, session, velocity: Tuple, location: Tuple, 
    acceleration: Tuple, vehicle_id: int, fleet_id: int, lane_id: int, 
    des_lane_id: int, delta:float):
        self.velocity = velocity
        self.location = location
        self.session = session
        self.acceleration = acceleration
        self.vehicle_id = vehicle_id
        self.fleet_id = fleet_id
        self.lane_id = lane_id
        self.des_lane_id = des_lane_id
        self.delta = delta
        self.state_record = [None]*16
        self.finish = False # pass the intersection or not
        self.zone_idx_list = get_conflict_zone_idx(self.lane_id, self.des_lane_id)

        self.declare_pub_state()
        self.declare_sub_state()

        self.decalre_pub_zone_status()

    def declare_pub_state(self):
        key = f"state/{self.lane_id}/{self.fleet_id}/{self.vehicle_id}"
        self.pub_state = self.session.declare_publisher(key)

    def pub_state(self):
        # key = f"state/{self.lane_id}/{self.fleet_id}/{self.vehicle_id}"
        x = 1 if self.finish else 0
        state = f"{self.lane_id},{self.fleet_id},{self.vehicle_id}," + \
        f"{self.location[0]},{self.location[1]}," + \
        f"{self.velocity[0]},{self.velocity[1]}," + \
        f"{self.acceleration[0]},{self.acceleration[1]},{self.des_lane_id},{x}"
        # print(f"Putting Data ('{key}': '{state}')...")
        self.pub_state.put(state)

    def declare_sub_state(self):
        def listener(sample: Sample):
            # print(f">> [Subscriber] Received {sample.kind} ('{sample.key_expr}': '{sample.payload.decode('utf-8')}')")
            receive = sample.payload.decode('utf-8').split(',')
            state = {
                "location": (float(receive[3]),float(receive[4])),
                "velocity": (float(receive[5]),float(receive[6])),
                "acceleration": (float(receive[7]),float(receive[8])),
                "des_lane_id": int(receive[9])
            }
            rec_lane_id = int(receive[0])
            rec_fleet_id = int(receive[1])
            rec_vehicle_id = int(receive[2])
            rec_finish = int(receive[10])
            if rec_finish==0:
                self.state_record[rec_vehicle_id] = state

        key = f"state/{self.lane_id}/{self.fleet_id}/**"
        sub_state = self.session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())
        

    def decalre_pub_zone_status(self):
        key = "zone"
        self.pub_zone_status = self.session.declare_publisher(key)
    
    def pub_zone_status(self, occupied):
        if occupied:
            buf = "occupied"
        else:
            buf = "free"
        # print(f"Putting Data ('{key}': '{buf}')...")
        
        self.pub_zone_status.put(buf)

    def finish_cross(self) -> bool:
        ## judge whether the vehicle has crossed the intersection
        if self.des_lane_id == 0:
            if self.location[0] >= 4:
                return True
            else:
                return False
        elif self.des_lane_id == 1:
            if self.location[1] >= 4:
                return True
            else:
                return False
        elif self.des_lane_id == 2:
            if self.location[0] <= -4:
                return True
            else:
                return False
        else:
            if self.location[1] <= -4:
                return True
            else:
                return False

class Leader(MyVehicle):
    def __init__(self, session, velocity: Tuple[float], location: Tuple, 
    acceleration: Tuple, vehicle_id: int, fleet_id: int, lane_id: int, delta:float,
    des_lane_id: int, fleet_length: int):
        super().__init__(session, velocity, location, acceleration, vehicle_id, 
        fleet_id, lane_id, des_lane_id, delta)
        self.fleet_length = fleet_length
        self.schedule_map = set()
        self.schedule_map.add((self.lane_id, self.des_lane_id, self.fleet_id))
        self.agree = [False]*4
        self.agree[self.lane_id] = True
        self.fleets_state_record = dict()
        self.intersection_occupied = False
        #==================================#
        self.proposal = None

        self.declare_pub_schedule_map()
        self.declare_sub_schedule_map()

        self.declare_pub_state()
        self.declare_sub_state()

        self.declare_pub_propose()
        self.declare_sub_propose()

    def declare_pub_schedule_map(self):
        key = f"map/{self.lane_id}"
        self.pub_schedule_map = self.session.declare_publisher(key)

    def pub_schedule_map(self):
        key = f"map/{self.lane_id}"
        pub_map = f"{self.lane_id}:"
        for fleet_info in self.schedule_map:
            pub_map += str(self.lane_id)
            pub_map += ","
            pub_map += str(self.des_lane_id)
            pub_map += ","
            pub_map += str(self.fleet_id)
            pub_map += ";"
        # print(f"Putting Data ('{key}': '{state}')...")
        self.pub_schedule_map.put(pub_map)

    def declare_sub_schedule_map(self):
        def listener(sample: Sample):
            rcv_schedule_map = set()
            receive = sample.payload.decode('utf-8').split(":")
            rec_lane_id = int(receive[0])
            rec_fleet_info = receive[1].split(";")[:(-1)]
            for rec_fleet in rec_fleet_info:
                rec_fleet = rec_fleet.split(",")
                rcv_schedule_map.add((int(rec_fleet[0]),int(rec_fleet[1]),int(rec_fleet[2])))
            if rcv_schedule_map==self.schedule_map:
                self.agree[rec_lane_id] = True
            else:
                self.schedule_map = self.schedule_map.union(rcv_schedule_map)

        key = "map/**"
        sub = self.session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())

    def declare_sub_state(self):
        ## subscribe state from other fleets
        def listener(sample: Sample):
            # print(f">> [Subscriber] Received {sample.kind} ('{sample.key_expr}': '{sample.payload.decode('utf-8')}')")
            receive = sample.payload.decode('utf-8').split(',')
            state = {
                "location": (float(receive[3]),float(receive[4])),
                "velocity": (float(receive[5]),float(receive[6])),
                "acceleration": (float(receive[7]),float(receive[8])),
                "des_lane_id": int(receive[9])
            }
            rec_lane_id = int(receive[0])
            rec_fleet_id = int(receive[1])
            rec_vehicle_id = int(receive[2])
            rec_finish = int(receive[10])
            if rec_finish == 0:
                self.fleets_state_record[(rec_lane_id,rec_fleet_id,rec_vehicle_id)] = state

        key = f"state/**"
        sub = self.session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())

    def propose(self, num_iter, alpha):
        ## propose a schedule based on the states of other vehicles
        scheduler = Scheduler.Scheduler(CONFLICT_ZONES,self.fleets_state_record,1.5,self.lane_id,self.fleet_id,alpha)
        passing_order = scheduler.search(num_iter)
        time_slot = scheduler.passing_order_to_time_slot(passing_order)
        self.proposal = time_slot
        # self.proposal format:
        # (lane_id, fleet_id, veh_id) -> [deadline, deadline, deadline, deadline]

    def declare_pub_propose(self):
        key = f"proposal/{self.lane_id}/{self.fleet_id}"
        self.pub_propose = self.session.declare_publisher(key)

    def pub_propose(self):
        key = f"proposal/{self.lane_id}/{self.fleet_id}"
        pub_content = f"{self.lane_id},{self.fleet_id}:"
        for veh in self.proposal:
            deadlines = self.proposal[veh]
            assert len(veh) == 3 and len(deadlines) == 4
            pub_content += f"{veh[0]},{veh[1]},{veh[2]},{deadlines[0]},{deadlines[1]},{deadlines[2]},{deadlines[3]};"
        
        self.pub_propose.put(pub_content)

    def declare_sub_propose(self):
        self.other_proposal = dict()
        def listener(sample: Sample):
            receive = sample.payload.decode('utf-8').split(':')
            lane_id = int(receive[0].split(',')[0])
            fleet_id = int(receive[0].split(',')[1])
            self.other_proposal[(lane_id, fleet_id)] = dict()
            rec_proposal = receive[1].split(';')
            for s in rec_proposal[:(-1)]:
                s = s.split(',')
                veh = (int(s[0]),int(s[1]),int(s[2]))
                deadlines = [float(s[3]),float(s[4]),float(s[5]),float(s[6])]
                self.other_proposal[(lane_id,fleet_id)][veh] = deadlines
                
        key = "proposal/**"
        sub = self.session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())                

    def declare_pub_score(self):
        key = f"score/{self.lane_id}/{self.fleet_id}"
        self.pub_score = self.session.declare_publisher(key)
    
    def pub_score(self):
        key = f"score/{self.lane_id}/{self.fleet_id}"
        pub_content = f"{self.lane_id},{self.fleet_id}:"
        for (lane_id, fleet_id) in self.other_proposal:
            score = self.scoring(self.other_proposal[(lane_id, fleet_id)])
            pub_content += f"{lane_id},{fleet_id},{score};"
        score = self.scoring(self.proposal)
        pub_content += f"{self.lane_id},{self.fleet_id},{score};"
        self.pub_score.put(pub_content)

    def declare_sub_score(self):
        self.all_score = None
        def listener(sample: Sample):
            receive = sample.payload.decode('utf-8').split(':')
            rec_scores = receive[1].split(';')[:(-1)]
            for rec in rec_scores:
                rec = rec.split(',')
                lane_id = int(rec[0])
                fleet_id = int(rec[1])
                score = float(rec[2])
                if (lane_id, fleet_id) not in self.all_score:
                    self.all_score[(lane_id, fleet_id)] = score
                else:
                    self.all_socre[(lane_id, fleet_id)] += score
            
        key = "score/**"
        sub = self.session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())
    
    def schedule_group_consensus(self):
        return self.agree[0] and self.agree[1] and self.agree[2] and self.agree[3]
    
    def scoring(self, time_slot):
        total_delay = 0
        for (lane_id, fleet_id, veh_id) in time_slot:
            if lane_id == self.lane_id and fleet_id == self.fleet_id:
                velocity = self.fleets_state_record[(lane_id, fleet_id, veh_id)]['velocity']
                speed = vector_length(velocity[0], velocity[1])
                location = self.fleets_state_record[(lane_id, fleet_id, veh_id)]['location']
                des_lane_id = self.fleets_state_record[(lane_id, fleet_id, veh_id)]['des_lane_id']
                t_min = get_min_arrival_time(CONFLICT_ZONES,lane_id,des_lane_id,location,speed)
                total_delay += (time_slot[(lane_id, fleet_id, veh_id)] - t_min)
        return -total_delay/self.fleet_length
    
    def get_final_assignment(self):
        max_score = float("-inf")
        proposer = None
        for (lane_id, fleet_id) in self.all_score:
            if self.all_score[(lane_id,fleet_id)] > max_score:
                max_score = self.all_score[(lane_id,fleet_id)]
                proposer = (lane_id,fleet_id)
            elif self.all_score[(lane_id,fleet_id)] == max_score and lane_id < proposer[0]:
                proposer = (lane_id,fleet_id)
            elif self.all_score[(lane_id,fleet_id)] == max_score and lane_id == proposer[0] and fleet_id < proposer[1]:
                proposer = (lane_id,fleet_id)
        self.final_assignment = self.other_proposal[proposer]
                
                
    '''
    def propose(self, schedule_granularity=1.0): 
        # consider state of all vehicles, return: a, b
        # a: which fleet goes first
        # b: time slot order as an array (specify deadline)
        avg_delay = 1000000.0
        best_propose = [None]*2
        best_granularity = None
        self.scheduler = Scheduler.Scheduler(CONFLICT_ZONE, self.state_record)
        for x in range (9, 25):
            granularity = x * 0.1
            self.scheduler.propose_best_schedule(granularity)
            first_fleet, schedule= self.scheduler.get_schedule()
            if schedule is not None:
                if(self.scheduler.loss_function(first_fleet,schedule) <= avg_delay):
                    avg_delay = self.scheduler.loss_function(first_fleet,schedule)
                    best_propose[0], best_propose[1] = first_fleet+1, schedule
                    best_granularity = granularity
        self.proposal = f"{best_propose[0]}/"
        for slot in best_propose[1]:
            self.proposal += f"{slot},"

    def pub_result(self, scheduling_result):
        key = "scheduling/**"
        print(f"Putting Data ('{key}': '{scheduling_result}...'")
        
        self.pub = self.session.declare_publisher(key)
        self.pub.put(scheduling_result)

        receive = self.proposal.split('/')
        self.result[0] = float(receive[0])
        self.result[1] = []
        for slot in receive[1].split(','):
            if(slot == ''):
                break
            self.result[1].append(float(slot))

    def pub_propose(self):
        key = f"proposal/{self.fleet_id}"
        self.propose()
        buf = f"{self.fleet_id}:{self.proposal}"
        # print(f"Putting Data ('{key}': '{buf}')...")
        
        self.pub = self.session.declare_publisher(key)
        self.pub.put(buf)

        receive = self.proposal.split('/')
        self.result[0] = float(receive[0])
        self.result[1] = []
        for slot in receive[1].split(','):
            if(slot == ''):
                break
            self.result[1].append(float(slot))
    
    def declare_sub_propose(self, wait_time=3):
        def listener(sample: Sample):
            self.other_proposal = sample.payload.decode('utf-8')
            if(int(self.other_proposal[:1]) != self.fleet_id):
                if not self.consensus:
                    print(f">> [Subscriber] Received {sample.kind} ('{sample.key_expr}': '{self.other_proposal}')")
                if(self.other_proposal[2:] == self.proposal):
                    print("successful")
                    self.consensus = True # which means that the scehduling is usable
                else:
                    print("failed")
                    print(self.proposal)
        
        key = "proposal/**"
        self.sub_propose = self.session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())
        # time.sleep(wait_time)
        # sub.undeclare()
    '''

    def declare_sub_zone_status(self, wait_time=5):
        def listener(sample: Sample):
            receive = sample.payload.decode('utf-8')
            if receive == "occupied":
                self.intersection_occupied = True

        key = "zone"
        self.sub_result = self.session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())        