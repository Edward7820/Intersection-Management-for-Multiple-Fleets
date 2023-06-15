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
        self.assignment = dict()

        self.declare_pub_state()
        self.declare_sub_state()

        self.declare_sub_final_assignment

        # self.decalre_pub_zone_status()

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

    def declare_sub_final_assignment(self):
        def listener(sample: Sample):
            receive = sample.payload.decode('utf-8').split(';')[:(-1)]
            for r in receive:
                r = r.split(',')
                veh = (int(r[0]),int(r[1]),int(r[2]))
                deadlines = [float(r[3]),float(r[4]),float(r[5]),float(r[6])]
                self.assignment[veh] = deadlines
        
        key = f"final/{self.lane_id}/{self.fleet_id}"
        sub_final_assignment = self.session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())
        

    '''def decalre_pub_zone_status(self):
        key = "zone"
        self.pub_zone_status = self.session.declare_publisher(key)
    
    def pub_zone_status(self, occupied):
        if occupied:
            buf = "occupied"
        else:
            buf = "free"
        # print(f"Putting Data ('{key}': '{buf}')...")
        
        self.pub_zone_status.put(buf)'''

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
        self.schedule_map.add((self.lane_id, self.des_lane_id, self.fleet_id, self.fleet_length))
        self.agree = [False]*4
        self.agree[self.lane_id] = True
        self.fleets_state_record = dict()
        self.intersection_occupied = False
        #==================================#
        self.proposal = None
        self.other_proposal = dict()
        self.all_score = dict()
        self.final_assignment = None

        self.declare_pub_schedule_map()
        self.declare_sub_schedule_map()

        self.declare_pub_state()
        self.declare_sub_state()

        self.declare_pub_propose()
        self.declare_sub_propose()

        self.declare_pub_score()
        self.declare_sub_score()

        self.declare_pub_final_assignment()

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
            pub_map += ","
            pub_map += str(self.fleet_length)
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
                rcv_schedule_map.add((int(rec_fleet[0]),int(rec_fleet[1]),int(rec_fleet[2]),int(rec_fleet[3])))
            if rcv_schedule_map==self.schedule_map:
                self.agree[rec_lane_id] = True
            else:
                self.schedule_map = self.schedule_map.union(rcv_schedule_map)

        key = "map/**"
        sub = self.session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())

    def schedule_group_consensus(self):
        return self.agree[0] and self.agree[1] and self.agree[2] and self.agree[3]

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
            if rec_finish == 0 and (rec_lane_id,rec_fleet_id) in [(k[0],k[2]) for k in list(self.schedule_map.keys())]:
                self.fleets_state_record[(rec_lane_id,rec_fleet_id,rec_vehicle_id)] = state

        key = f"state/**"
        sub = self.session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())

    def all_states_received(self):
        for k in list(self.schedule_map.keys):
            fleet_size = k[3]
            lane_id = k[0]
            fleet_id = k[2]
            for i in range(fleet_size):
                if (lane_id,fleet_id,i) not in self.fleets_state_record:
                    return False 
        return True

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
        # self.other_proposal = dict()
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

    def all_proposal_received(self):
        for (lane_id,des_lane_id,fleet_id,fleet_len) in self.schedule_map:
            if lane_id == self.lane_id and fleet_id == self.fleet_id:
                continue
            if (lane_id,fleet_id) not in self.other_proposal:
                return False
        return True
    
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
        def listener(sample: Sample):
            receive = sample.payload.decode('utf-8').split(':')
            rec_scores = receive[1].split(';')[:(-1)]
            sender_info = receive[0].split(',')
            sender_lane_id = int(sender_info[0])
            sender_fleet_id = int(sender_info[1])
            for rec in rec_scores:
                rec = rec.split(',')
                lane_id = int(rec[0])
                fleet_id = int(rec[1])
                score = float(rec[2])
                self.all_score[(lane_id,fleet_id,sender_lane_id,sender_fleet_id)] = score
            
        key = "score/**"
        sub = self.session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())

    def all_score_received(self):
        for (lane_id,_,fleet_id,_) in self.schedule_map:
            for (sender_lane_id,_,sender_fleet_id,_) in self.schedule_map:
                if (lane_id,fleet_id,sender_lane_id,sender_fleet_id) not in self.all_score:
                    return False
        return True
    
    def get_final_assignment(self):
        final_score = dict()
        for (lane_id,fleet_id,sender_lane_id,sender_fleet_id) in self.all_score:
            if (lane_id,fleet_id) not in final_score:
                final_score[(lane_id,fleet_id)] = self.all_score[(lane_id,fleet_id,sender_lane_id,sender_fleet_id)]
            else:
                final_score[(lane_id,fleet_id)] += self.all_score[(lane_id,fleet_id,sender_lane_id,sender_fleet_id)]
        max_score = float("-inf")
        proposer = None
        for (lane_id, fleet_id) in final_score:
            if final_score[(lane_id,fleet_id)] > max_score:
                max_score = final_score[(lane_id,fleet_id)]
                proposer = (lane_id,fleet_id)
            elif final_score[(lane_id,fleet_id)] == max_score and lane_id < proposer[0]:
                proposer = (lane_id,fleet_id)
            elif final_score[(lane_id,fleet_id)] == max_score and lane_id == proposer[0] and fleet_id < proposer[1]:
                proposer = (lane_id,fleet_id)
        self.final_assignment = self.other_proposal[proposer]

    def declare_pub_final_assignment(self):
        key = f"final/{self.lane_id}/{self.fleet_id}"
        self.pub_final_assignment = self.session.declare_publisher(key)

    def pub_final_assignment(self):
        key = f"final/{self.lane_id}/{self.fleet_id}"
        pub_content = ""
        for veh in self.final_assignment:
            deadlines = self.final_assignment[veh]
            assert len(veh) == 3 and len(deadlines) == 4
            pub_content += f"{veh[0]},{veh[1]},{veh[2]},{deadlines[0]},{deadlines[1]},{deadlines[2]},{deadlines[3]};"    
        self.pub_propose.put(pub_content)
                
    '''def declare_sub_zone_status(self, wait_time=5):
        def listener(sample: Sample):
            receive = sample.payload.decode('utf-8')
            if receive == "occupied":
                self.intersection_occupied = True

        key = "zone"
        self.sub_result = self.session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())'''      