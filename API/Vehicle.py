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
from math_utils import euclidean_dist
from . import Scheduler
from typing import List, Tuple
#from Scheduler import Scheduler


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
        ## TODO: judge whether the vehicle has crossed the intersection
        if self.des_lane_id == 0:
            if euclidean_dist(self.location, (4,-2)) <= 2:
                return True
            else:
                return False
        elif self.des_lane_id == 1:
            if euclidean_dist(self.location, (2,4)) <= 2:
                return True
            else:
                return False
        elif self.des_lane_id == 2:
            if euclidean_dist(self.location, (-4,2)) <= 2:
                return True
            else:
                return False
        else:
            if euclidean_dist(self.location, (-2,-4)) <= 2:
                return True
            else:
                return False

class Leader(MyVehicle):
    def __init__(self, session, velocity: Tuple[float], location: Tuple, 
    acceleration: Tuple, vehicle_id: int, fleet_id: int, lane_id: int, delta:float,
    des_lane_id: int, fleet_length: int):
        super().__init__(session, velocity, location, acceleration, vehicle_id, 
        fleet_id, lane_id, delta)
        self.des_lane_id = des_lane_id
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
        ## TODO: subscribe state from other fleets
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
                self.fleets_state_record[(rec_lane_id,rec_fleet_id,rec_vehicle_id)] = state

        key = f"state/**"
        sub = self.session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())

    def propose(self):
        ## TODO: propose a schedule based on the states of other vehicles
        pass

    def declare_pub_propose(self):
        ## TODO
        pass

    def pub_propose(self):
        ## TODO
        pass

    def declare_sub_propose(self):
        ## TODO
        pass
    
    def schedule_group_consensus(self):
        return self.agree[0] and self.agree[1] and self.agree[2] and self.agree[3]
        

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