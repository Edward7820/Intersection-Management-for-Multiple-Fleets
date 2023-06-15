import sys
import time
from argparse import ArgumentParser
import itertools
import json
import zenoh
from datetime import datetime
from API.Vehicle import MyVehicle, Leader
import os, signal
from multiprocessing import Process, Array
from typing import List, Dict, Tuple
MAX_VEH_NUM = 100

SCHEDULE_GROUP_FORMING = 0
COLLECT_STATES = 1
COLLECT_PROPOSALS = 2
COLLECT_SCORES = 3
RUNNING = 4

def run_vehicle(veh_num: int, pid: int, lane_id: int, des_lane_id: int, fid: int, 
    fleet_len: int,  vid: int, location: tuple, velocity: tuple, 
    acceleration: tuple, rounds):
    # print(f"start running vehicle {lane_id}-{fid}-{vid}")
    delta_t = args.delta_t
    session = zenoh.open()
    if vid == 0:
        # Leader
        myvehicle = Leader(session, velocity, location, acceleration, vid, fid, lane_id,
                           delta_t, des_lane_id, fleet_len)
    else:
        myvehicle = MyVehicle(session, velocity, location, acceleration,
                               vid, fid, lane_id, des_lane_id, delta_t)
    cur_round = 1
    phase = SCHEDULE_GROUP_FORMING
    while(True):
        rounds[pid] = cur_round

        ## wait for other processes
        waiting = True
        while waiting:
            if all([r>=cur_round for r in rounds]):
                waiting = False

        if myvehicle.finish_cross():
            break

        myvehicle.pub_state()
        if phase != RUNNING:
            if vid == 0:
                if phase == SCHEDULE_GROUP_FORMING:
                    # print(myvehicle.schedule_map)
                    myvehicle.pub_schedule_map()
                    if myvehicle.schedule_group_consensus():
                        print(f"Fleet {lane_id}-{fid} got final schedule group: {myvehicle.schedule_map}")
                        assert myvehicle.all_states_received()
                        phase = COLLECT_PROPOSALS
                elif phase == COLLECT_PROPOSALS:
                    myvehicle.propose(1000, 1.2)
                    print(f"Fleet {lane_id}-{fid} proposed time slot assignment: {myvehicle.proposal}")
                    myvehicle.pub_propose()
                    if myvehicle.all_proposal_received():
                        print(f"Fleet {lane_id}-{fid} received all proposals!")
                        phase = COLLECT_SCORES
                elif phase == COLLECT_SCORES:
                    myvehicle.pub_score()
                    if myvehicle.all_score_received():
                        print(f"Fleet {lane_id}-{fid} received all score!")
                        myvehicle.get_final_assignment()
                        print(f"Final assignment for Fleet {lane_id}-{fid}: {myvehicle.final_assignment}")
                        phase = RUNNING
            else:
                if len(myvehicle.final_assignment) > 0:
                    phase = RUNNING
        else:
            ## TODO
            pass

        # time.sleep(0.5)
        cur_round += 1


def main():
    parser = ArgumentParser()
    parser.add_argument("--input_file",type=str, default="sample_input")
    parser.add_argument("--delta_t",type=float, default=0.1)
    parser.add_argument("--max_speed",type=float,default=16)
    parser.add_argument("--max_acceleration",type=float,default=3)
    parser.add_argument("--min_acceleration",type=float,default=-3)
    global args
    args = parser.parse_args()

    with open(args.input_file, 'r') as f:
        input_words = f.read()
    input_lines = input_words.split('\n')
    basic_info = input_lines[0].split(' ')
    fleets_num = int(basic_info[0])
    veh_num = int(basic_info[1])
    rounds = Array('i', [0]*veh_num)

    fleets = [None]*fleets_num
    line_index = 1
    veh_processes = []
    for i in range(fleets_num):
        l = input_lines[line_index]
        l = l.split(' ')
        fleets[i] = dict()
        fleets[i]['lane_id'] = int(l[0])
        fleets[i]['des_lane_id'] = int(l[1])
        fleets[i]['fid'] = int(l[2]) #fleet id
        fleets[i]['fleet_len'] = int(l[3])
        fleets[i]['vehicles'] = []
        fleet_len = fleets[i]['fleet_len']
        line_index += 1
        for j in range(fleet_len):
            l = input_lines[line_index]
            l = l.split(' ')
            vid = int(l[0]) #vehicle id
            loc_x = float(l[1])
            loc_y = float(l[2])
            vel_x = float(l[3])
            vel_y = float(l[4])
            acc_x = float(l[5])
            acc_y = float(l[6])
            fleets[i]['vehicles'].append({'vid':vid, 'location':(loc_x,loc_y),
                'velocity':(vel_x,vel_y), 'acceleration':(acc_x,acc_y)})
            line_index += 1
            veh_proc = Process(target=run_vehicle, args=(veh_num, len(veh_processes), 
                fleets[i]['lane_id'], fleets[i]['des_lane_id'], 
                fleets[i]['fid'], fleets[i]['fleet_len'],
                vid, (loc_x,loc_y), (vel_x,vel_y), (acc_x,acc_y), rounds))
            veh_processes.append(veh_proc)
            veh_proc.start()
    assert(len(veh_processes) == veh_num)

    while True:
        for proc in veh_processes:
            proc.join(timeout=2)

if __name__ == '__main__':
    main()
    