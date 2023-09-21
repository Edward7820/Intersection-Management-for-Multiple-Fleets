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
from .math_utils import *
from . import Simulator
from typing import List, Tuple
from .Scheduler import Scheduler
CONFLICT_ZONES = [(0,0,4,4),(-4,0,0,4),(-4,-4,0,0),(0,-4,4,0)]
X_MIN = 0
X_MAX = 2
Y_MIN = 1
Y_MAX = 3
MAX_SPEED = 12
MAX_ACCELERATION = 3
MIN_ACCELERATION = -5
SAFETY_GAP = 1.5
MAX_FLEET_SIZE = 16
TURNING_RADIUS = 2

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
        self.finish = False # pass the intersection or not
        self.tick = 0
        self.final_assignment = dict()
        self.state_record = [None]*MAX_FLEET_SIZE

    def declare_pub_state(self):
        key = f"state/{self.lane_id}/{self.fleet_id}/{self.vehicle_id}"
        self.publisher_state = self.session.declare_publisher(key)

    def pub_state(self):
        key = f"state/{self.lane_id}/{self.fleet_id}/{self.vehicle_id}"
        x = 1 if self.finish else 0
        state = f"{self.lane_id},{self.fleet_id},{self.vehicle_id}," + \
        f"{self.location[0]},{self.location[1]}," + \
        f"{self.velocity[0]},{self.velocity[1]}," + \
        f"{self.acceleration[0]},{self.acceleration[1]},{self.des_lane_id},{x}"
        # print(f"About to put Data ('{key}': '{state}')...")
        self.publisher_state.put(state)
        # print(f"Putting Data ('{key}': '{state}')...")

    def declare_sub_state(self):
        pass

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
            
    def step_vehicle(self):
        ## update its own state
        original_velocity = self.velocity
        delta_v = vector_mul_scalar(self.acceleration, self.delta)
        self.velocity = vector_add(original_velocity, delta_v)
        if inner_product(original_velocity, self.velocity) < 0:
            tangent_v = projection(self.velocity, original_velocity)
            self.velocity = vector_sub(self.velocity, tangent_v)
        speed = vector_length(self.velocity[0], self.velocity[1])
        if speed > MAX_SPEED:
            scalar = MAX_SPEED / speed
            self.velocity = vector_mul_scalar(self.velocity, scalar)
            speed = vector_length(self.velocity[0], self.velocity[1])
        average_v = vector_mul_scalar(vector_add(self.velocity, original_velocity), 0.5)
        displacement = vector_mul_scalar(average_v, self.delta)
        self.location = vector_add(self.location, displacement)
        self.tick += self.delta

    def get_waypoints(self, conflict_zones):
        # return: a list of dict
        waypoints = []
        deadlines = self.final_assignment[(self.lane_id, self.fleet_id, self.vehicle_id)]
        sorted_deadlines = sorted(deadlines)
        zone_idx_list = get_conflict_zone_idx(self.lane_id, self.des_lane_id)
        for _ in range(len(zone_idx_list)+1):
            waypoints.append(dict())
        waypoints[-1]["time"] = max(deadlines)
        if self.des_lane_id == 0:
            waypoints[-1]["location"] = conflict_zone_east_point(conflict_zones[3])
        elif self.des_lane_id == 1:
            waypoints[-1]["location"] = conflict_zone_north_point(conflict_zones[0])
        elif self.des_lane_id == 2:
            waypoints[-1]["location"] = conflict_zone_west_point(conflict_zones[1])
        elif self.des_lane_id == 3:
            waypoints[-1]["location"] = conflict_zone_south_point(conflict_zones[2])
        if len(zone_idx_list) >= 2:
            waypoints[-2]["time"] = sorted_deadlines[-2]
            if self.des_lane_id == 0:
                waypoints[-2]["location"] = conflict_zone_west_point(conflict_zones[3])
            elif self.des_lane_id == 1:
                waypoints[-2]["location"] = conflict_zone_south_point(conflict_zones[0])
            elif self.des_lane_id == 2:
                waypoints[-2]["location"] = conflict_zone_east_point(conflict_zones[1])
            elif self.des_lane_id == 3:
                waypoints[-2]["location"] = conflict_zone_north_point(conflict_zones[2])
        if len(zone_idx_list) >= 3:
            waypoints[-3]["time"] == sorted_deadlines[-3]
            if self.des_lane_id == 0:
                waypoints[-3]["location"] = conflict_zone_north_point(conflict_zones[2])
            elif self.des_lane_id == 1:
                waypoints[-3]["location"] = conflict_zone_west_point(conflict_zones[3])
            elif self.des_lane_id == 2:
                waypoints[-3]["location"] = conflict_zone_south_point(conflict_zones[0])
            elif self.des_lane_id == 3:
                waypoints[-3]["location"] = conflict_zone_east_point(conflict_zones[1])

        if self.lane_id == 0:
            waypoints[0]["location"] = conflict_zone_east_point(conflict_zones[0])
        elif self.lane_id == 1:
            waypoints[0]["location"] = conflict_zone_north_point(conflict_zones[1])
        elif self.lane_id == 2:
            waypoints[0]["location"] = conflict_zone_west_point(conflict_zones[2])
        else:
            waypoints[0]["location"] = conflict_zone_south_point(conflict_zones[3])
        first_zone_schedule = [t[zone_idx_list[0]] for t in self.final_assignment.values()]
        first_zone_schedule = filter(lambda t: t < deadlines[zone_idx_list[0]], first_zone_schedule)
        last_veh_leave_first_zone_time = max(first_zone_schedule)
        if last_veh_leave_first_zone_time < 0:
            last_veh_leave_first_zone_time = 0
        waypoints[0]["time"] = last_veh_leave_first_zone_time
        return waypoints
    
    def get_acceleration_linear_motion(self, waypt_loc, deadline):
        distance = euclidean_dist(self.location, waypt_loc)
        displacement = vector_sub(waypt_loc, self.location)
        speed = vector_length(self.velocity[0], self.velocity[1])
        if speed < 0.1:
            direction = get_unit_vector(displacement)
        else:
            direction = get_unit_vector(self.velocity)
        target_direction = get_unit_vector(displacement)
        if inner_product(direction, target_direction) > 0:
            waypt_speed = 2*distance/(deadline-self.tick) - speed
            if waypt_speed >= 0:
                a_tan = (waypt_speed - speed)/(deadline - self.tick)
                if a_tan > MAX_ACCELERATION:
                    a_tan = MAX_ACCELERATION    
            else:
                if speed < 1:
                    a_tan = 0
                else:
                    a_tan = MIN_ACCELERATION

            # remain safety distance to the front car
            if self.vehicle_id > 0 and self.state_record[self.vehicle_id-1] != None:
                front_car_loc = self.state_record[self.vehicle_id-1]["location"]
                front_car_dist = euclidean_dist(self.location, front_car_loc)
                if speed >= 0.1 and front_car_dist / speed < 1.5:
                    a_tan = MIN_ACCELERATION
                elif speed >= 0.1 and front_car_dist / speed < 2.5:
                    a_tan = MIN_ACCELERATION / 2
        else:
            a_tan = MIN_ACCELERATION
        return vector_mul_scalar(direction, a_tan)
    
    def update_acceleration(self):
        ## waypoints pursuing
        waypoints = self.get_waypoints(CONFLICT_ZONES)
        slot_id = 0
        for waypt in waypoints:
            if self.tick >= waypt["time"]:
                slot_id += 1
            else:
                break
        if slot_id >= len(waypoints):
            return
        waypt_loc = waypoints[slot_id]["location"]
        deadline = waypoints[slot_id]["time"]
        if (self.des_lane_id-self.lane_id) % 4 == 2:
            self.acceleration = self.get_acceleration_linear_motion(waypt_loc, deadline)
        elif (self.des_lane_id-self.lane_id) % 4 == 3 and slot_id != 2:
            self.acceleration = self.get_acceleration_linear_motion(waypt_loc, deadline)
        else:
            self.acceleration = self.get_acceleration_linear_motion(waypt_loc, deadline)
            target_direction = get_unit_vector(vector_sub(waypt_loc, self.location))
            speed = vector_length(self.velocity[0], self.velocity[1])
            if speed < 0.001:
                tan_direction = target_direction
            else:
                tan_direction = get_unit_vector(self.velocity)
            a_normal = speed * speed / TURNING_RADIUS
            if not right_or_left(tan_direction, target_direction):
                normal_direction = get_left_normal_vector(tan_direction)
            else:
                normal_direction = get_right_normal_vector(tan_direction)
            self.acceleration = vector_add(self.acceleration, vector_mul_scalar(normal_direction, a_normal))

class Member(MyVehicle):
    def __init__(self, session, velocity: Tuple, location: Tuple, 
    acceleration: Tuple, vehicle_id: int, fleet_id: int, lane_id: int, 
    des_lane_id: int, delta:float):
        super().__init__(session, velocity, location, acceleration, vehicle_id, 
        fleet_id, lane_id, des_lane_id, delta)
        self.zone_idx_list = get_conflict_zone_idx(self.lane_id, self.des_lane_id)
        # self.final_assignment = dict()
        # self.publisher_state = None
        # self.subscriber_state = None
        # self.subscriber_final_assignment = None

        self.declare_pub_state()
        self.declare_sub_state()

        self.declare_sub_final_assignment()

        # self.decalre_pub_zone_status()
        print(f"initialize vehicle {self.lane_id}-{self.fleet_id}-{self.vehicle_id}")

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
            else:
                self.state_record[rec_vehicle_id] = None

        key = f"state/{self.lane_id}/{self.fleet_id}/**"
        self.subscriber_state = self.session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())

    def declare_sub_final_assignment(self):
        def listener(sample: Sample):
            receive = sample.payload.decode('utf-8').split(';')[:(-1)]
            for r in receive:
                r = r.split(',')
                veh = (int(r[0]),int(r[1]),int(r[2]))
                deadlines = [float(r[3]),float(r[4]),float(r[5]),float(r[6])]
                self.final_assignment[veh] = deadlines
        
        key = f"final/{self.lane_id}/{self.fleet_id}"
        self.subscriber_final_assignment = self.session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())
        
    def get_front_veh_state(self):
        return self.state_record[self.vehicle_id - 1]
    
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
        self.all_proposal = dict()
        self.all_score = dict()
        # self.final_assignment = dict()
        '''self.publisher_schedule_map = None
        self.publisher_state = None
        self.publisher_propose = None
        self.publisher_score = None
        self.publisher_final_assignment = None
        self.subscriber_schedule_map = None
        self.subscriber_state = None
        self.subscriber_proposal = None
        self.subscriber_final_assignment = None
        self.subscriber_score = None'''

        self.declare_pub_state()
        self.declare_sub_state()

        self.declare_pub_schedule_map()
        self.declare_sub_schedule_map()

        self.declare_pub_propose()
        self.declare_sub_propose()

        self.declare_pub_score()
        self.declare_sub_score()

        self.declare_pub_final_assignment()
        print(f"initialize vehicle {self.lane_id}-{self.fleet_id}-{self.vehicle_id}")

    def declare_pub_schedule_map(self):
        key = f"map/{self.lane_id}"
        self.publisher_schedule_map = self.session.declare_publisher(key)

    def pub_schedule_map(self):
        key = f"map/{self.lane_id}"
        pub_map = f"{self.lane_id}:"
        for fleet_info in self.schedule_map:
            pub_map += str(fleet_info[0])
            pub_map += ","
            pub_map += str(fleet_info[1])
            pub_map += ","
            pub_map += str(fleet_info[2])
            pub_map += ","
            pub_map += str(fleet_info[3])
            pub_map += ";"
        # print(f"Putting Data ('{key}': '{pub_map}')...")
        self.publisher_schedule_map.put(pub_map)

    def declare_sub_schedule_map(self):
        def listener(sample: Sample):
            rcv_schedule_map = set()
            receive = sample.payload.decode('utf-8').split(":")
            # print(receive)
            rec_lane_id = int(receive[0])
            rec_fleet_info = receive[1].split(";")[:(-1)]
            for rec_fleet in rec_fleet_info:
                rec_fleet = rec_fleet.split(",")
                rcv_schedule_map.add((int(rec_fleet[0]),int(rec_fleet[1]),int(rec_fleet[2]),int(rec_fleet[3])))
            if rcv_schedule_map==self.schedule_map:
                self.agree[rec_lane_id] = True
            else:
                self.schedule_map = self.schedule_map.union(rcv_schedule_map)

        key = f"map/**"
        self.subscriber_schedule_map = self.session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())

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
            if rec_finish == 0 and (rec_lane_id,rec_fleet_id) in [(k[0],k[2]) for k in self.schedule_map]:
                self.fleets_state_record[(rec_lane_id,rec_fleet_id,rec_vehicle_id)] = state

        key = f"state/**"
        self.subscriber_state = self.session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())

    def all_states_received(self):
        for k in self.schedule_map:
            fleet_size = k[3]
            lane_id = k[0]
            fleet_id = k[2]
            for i in range(fleet_size):
                if (lane_id,fleet_id,i) not in self.fleets_state_record:
                    return False 
        return True

    def propose(self, num_iter, alpha):
        ## propose a schedule based on the states of other vehicles
        scheduler = Scheduler(CONFLICT_ZONES,self.fleets_state_record,SAFETY_GAP,self.lane_id,self.fleet_id,alpha)
        passing_order = scheduler.search(num_iter)
        time_slot = scheduler.passing_order_to_time_slot(passing_order)
        self.proposal = time_slot
        # self.proposal format:
        # (lane_id, fleet_id, veh_id) -> [deadline, deadline, deadline, deadline]

    def declare_pub_propose(self):
        key = f"proposal/{self.lane_id}/{self.fleet_id}"
        self.publisher_propose = self.session.declare_publisher(key)

    def pub_propose(self):
        key = f"proposal/{self.lane_id}/{self.fleet_id}"
        pub_content = f"{self.lane_id},{self.fleet_id}:"
        for veh in self.proposal:
            deadlines = self.proposal[veh]
            assert len(veh) == 3 and len(deadlines) == 4
            pub_content += f"{veh[0]},{veh[1]},{veh[2]},{deadlines[0]},{deadlines[1]},{deadlines[2]},{deadlines[3]};"
        
        self.publisher_propose.put(pub_content)

    def declare_sub_propose(self):
        # self.other_proposal = dict()
        def listener(sample: Sample):
            try:
                receive = sample.payload.decode('utf-8').split(':')
                lane_id = int(receive[0].split(',')[0])
                fleet_id = int(receive[0].split(',')[1])
                self.all_proposal[(lane_id, fleet_id)] = dict()
                rec_proposal = receive[1].split(';')
                for s in rec_proposal[:(-1)]:
                    s = s.split(',')
                    veh = (int(s[0]),int(s[1]),int(s[2]))
                    deadlines = [float(s[3]),float(s[4]),float(s[5]),float(s[6])]
                    self.all_proposal[(lane_id,fleet_id)][veh] = deadlines
            except:
                print(f"received {receive} at key proposal/**")
                raise NotImplementedError

        key = "proposal/**"
        self.subscriber_proposal = self.session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())

    def all_proposal_received(self):
        for (lane_id,des_lane_id,fleet_id,fleet_len) in self.schedule_map:
            if (lane_id,fleet_id) not in self.all_proposal:
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
                total_delay += (max(time_slot[(lane_id, fleet_id, veh_id)]) - t_min)
        return -total_delay/self.fleet_length

    def declare_pub_score(self):
        key = f"score/{self.lane_id}/{self.fleet_id}"
        self.publisher_score = self.session.declare_publisher(key)
    
    def pub_score(self):
        key = f"score/{self.lane_id}/{self.fleet_id}"
        pub_content = f"{self.lane_id},{self.fleet_id}:"
        for (lane_id, fleet_id) in self.all_proposal:
            score = self.scoring(self.all_proposal[(lane_id, fleet_id)])
            pub_content += f"{lane_id},{fleet_id},{score};"
        self.publisher_score.put(pub_content)

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
        self.subscriber_score = self.session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())

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
        self.final_assignment = self.all_proposal[proposer]

    def declare_pub_final_assignment(self):
        key = f"final/{self.lane_id}/{self.fleet_id}"
        self.publisher_final_assignment = self.session.declare_publisher(key)

    def pub_final_assignment(self):
        key = f"final/{self.lane_id}/{self.fleet_id}"
        pub_content = ""
        for veh in self.final_assignment:
            deadlines = self.final_assignment[veh]
            assert len(veh) == 3 and len(deadlines) == 4
            pub_content += f"{veh[0]},{veh[1]},{veh[2]},{deadlines[0]},{deadlines[1]},{deadlines[2]},{deadlines[3]};"    
        self.publisher_final_assignment.put(pub_content)
                
    '''def declare_sub_zone_status(self, wait_time=5):
        def listener(sample: Sample):
            receive = sample.payload.decode('utf-8')
            if receive == "occupied":
                self.intersection_occupied = True

        key = "zone"
        self.sub_result = self.session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())'''      