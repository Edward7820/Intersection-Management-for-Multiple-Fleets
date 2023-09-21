import sys
import time
from argparse import ArgumentParser
import itertools
import json
import zenoh
from datetime import datetime
from API.Vehicle import MyVehicle, Leader, Member
from API.math_utils import *
import os, signal
from multiprocessing import Process, Array
from typing import List, Dict, Tuple
import matplotlib.pyplot as plt
CONFLICT_ZONES = [(0,0,4,4),(-4,0,0,4),(-4,-4,0,0),(0,-4,4,0)]
MAX_VEH_NUM = 100

SCHEDULE_GROUP_FORMING = 0
COLLECT_STATES = 1
COLLECT_PROPOSALS = 2
COLLECT_SCORES = 3
RUNNING = 4

def run_vehicle(veh_num: int, pid: int, lane_id: int, des_lane_id: int, fid: int, 
    fleet_len: int,  vid: int, location: tuple, velocity: tuple, 
    acceleration: tuple, rounds, location_info, velocity_info, finished_list):
    print(f"start running vehicle {lane_id}-{fid}-{vid} (pid: {pid})")
    # print(veh_num, pid, des_lane_id, fleet_len, location, velocity, acceleration)
    delta_t = args.delta_t
    session = zenoh.open()
    if vid == 0:
        # Leader
        myvehicle = Leader(session, velocity, location, acceleration, vid, fid, lane_id,
                           delta_t, des_lane_id, fleet_len)
    else:
        myvehicle = Member(session, velocity, location, acceleration,
                               vid, fid, lane_id, des_lane_id, delta_t)
    cur_round = 1
    phase = SCHEDULE_GROUP_FORMING
    while(True):
        location_info[pid*2] = myvehicle.location[0]
        location_info[pid*2+1] = myvehicle.location[1]
        velocity_info[pid*2] = myvehicle.velocity[0]
        velocity_info[pid*2+1] = myvehicle.velocity[1]
        rounds[pid] = cur_round

        ## wait for other processes
        waiting = True
        while waiting:
            # print(f"lane_id: {lane_id}, fid: {fid}, vid: {vid} (pid: {pid}), rounds: {list(rounds)}")
            if all([r>=cur_round for r in rounds]):
                waiting = False
            else:
                time.sleep(1)
            
        if pid == 0:
            ## collision detection
            for i in range(veh_num):
                if finished_list[i] == 1:
                    continue
                for j in range(veh_num):
                    if j != i and finished_list[j] == 0:
                        location1 = (location_info[2*i], location_info[2*i+1])
                        location2 = (location_info[2*j], location_info[2*j+1])
                        if euclidean_dist(location1, location2) <= 1:
                            print(f"Collision detected between vehicle {i} (location: {location1}) and vehicle {j} (location: {location2}) at {myvehicle.tick} seconds!")
                            raise

            if (myvehicle.tick // delta_t) % 5 == 0 and phase == RUNNING:
                fig, ax = plt.subplots()
                ax.set_xlim(-10,10)
                ax.set_ylim(-10,10)
                ax.plot([-10,10],[4,4])
                ax.plot([4,4],[10,-10])
                ax.plot([10,-10],[-4,-4])
                ax.plot([-4,-4],[-10,10])
                ax.plot([-10,10],[0,0],linestyle='dashed')
                ax.plot([0,0],[-10,10],linestyle='dashed')
                for i in range(veh_num):
                    ax.scatter(location_info[2*i], location_info[2*i+1], label=f"{i}")
                plt.savefig("figure_{:.2f}.png".format(myvehicle.tick))


        if myvehicle.finish_cross():
            if finished_list[pid] == 0:
                print(f"vehicle {lane_id}-{fid}-{vid} (pid: {pid}) has crossed the intersection using {myvehicle.tick} seconds.")
                with open(args.output_file, "a") as f:
                    f.write(f"vehicle {lane_id}-{fid}-{vid} (pid: {pid}) has crossed the intersection using {myvehicle.tick} seconds.\n")
                    f.flush()
                finished_list[pid] = 1

            # print(f"Vehicle {lane_id}-{fid}-{vid} (pid: {pid}) finished round {cur_round}")
            cur_round += 1
            continue

        # print(f"lane_id: {lane_id}, fid: {fid}, vid: {vid} (pid: {pid}), current round: {cur_round}, phase: {phase}")

        # print(f"Vehicle {lane_id}-{fid}-{vid} (pid: {pid}) starts round {cur_round}")
        myvehicle.pub_state()
        if phase != RUNNING:
            if vid == 0:
                if phase == SCHEDULE_GROUP_FORMING:
                    # print(pid, myvehicle.schedule_map)
                    myvehicle.pub_schedule_map()
                    if myvehicle.schedule_group_consensus():
                        # print(f"Fleet {lane_id}-{fid} got final schedule group: {myvehicle.schedule_map}")
                        phase = COLLECT_STATES
                elif phase == COLLECT_STATES:
                    if myvehicle.all_states_received():
                        # print(f"Fleet {lane_id}-{fid} got all states!")
                        phase = COLLECT_PROPOSALS
                elif phase == COLLECT_PROPOSALS:
                    myvehicle.propose(1000, 1.2)
                    # print(f"Fleet {lane_id}-{fid} proposed time slot assignment: {myvehicle.proposal}")
                    myvehicle.pub_propose()
                    if myvehicle.all_proposal_received():
                        # print(f"Fleet {lane_id}-{fid} received all proposals!")
                        phase = COLLECT_SCORES
                elif phase == COLLECT_SCORES:
                    myvehicle.pub_score()
                    if myvehicle.all_score_received():
                        # print(f"Fleet {lane_id}-{fid} received all score!")
                        myvehicle.get_final_assignment()
                        print(f"Final assignment for vehicle {lane_id}-{fid}-{vid}: {myvehicle.final_assignment}")
                        if pid == 0:
                            with open(args.output_file, "a") as f:
                                f.write(f"Final assignment: {myvehicle.final_assignment}\n")
                                f.flush()
                        myvehicle.pub_final_assignment()
                        with open(args.output_file, "a") as f:
                            f.write(f"waypoints of vehicle {lane_id}-{fid}-{vid}: {myvehicle.get_waypoints(CONFLICT_ZONES)}\n")
                            f.flush()
                        phase = RUNNING
            else:
                if len(myvehicle.final_assignment) > 0:
                    print(f"Final assignment for vehicle {lane_id}-{fid}-{vid}: {myvehicle.final_assignment}")
                    with open(args.output_file, "a") as f:
                        f.write(f"waypoints of vehicle {lane_id}-{fid}-{vid}: {myvehicle.get_waypoints(CONFLICT_ZONES)}\n")
                        f.flush()
                    phase = RUNNING
        else:
            myvehicle.step_vehicle()
            myvehicle.update_acceleration()
            if finished_list[pid] == 0 and (myvehicle.tick // delta_t) % 5 == 0:
                print(f"State of the vehicle {lane_id}-{fid}-{vid} at time {round(myvehicle.tick,3)}: location {myvehicle.location}, velocity {myvehicle.velocity}, acceleration {myvehicle.acceleration}")

        # print(f"Vehicle {lane_id}-{fid}-{vid} (pid: {pid}) finished round {cur_round}")
        # if pid == 0:
        #     print(f"round {cur_round}")
        cur_round += 1


def main():
    parser = ArgumentParser()
    parser.add_argument("--input_file",type=str, default="sample_input")
    parser.add_argument("--delta_t",type=float, default=0.1)
    parser.add_argument("--max_speed",type=float,default=16)
    parser.add_argument("--max_acceleration",type=float,default=3)
    parser.add_argument("--min_acceleration",type=float,default=-3)
    parser.add_argument("--output_file", type=str, default="output.txt")
    global args
    args = parser.parse_args()

    with open(args.input_file, 'r') as f:
        input_words = f.read()
    input_lines = input_words.split('\n')
    basic_info = input_lines[0].split(' ')
    fleets_num = int(basic_info[0])
    veh_num = int(basic_info[1])
    rounds = Array('i', [0]*veh_num)

    # To detect collisions
    location_info = Array('d', [0]*(veh_num*2))
    velocity_info = Array('d', [0]*(veh_num*2))
    finished_list = Array('i',[0]*veh_num)

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
            pid = len(veh_processes)
            location_info[pid*2] = loc_x
            location_info[pid*2+1] = loc_y
            velocity_info[pid*2] = vel_x
            velocity_info[pid*2+1] = vel_y
            fleets[i]['vehicles'].append({'vid':vid, 'location':(loc_x,loc_y),
                'velocity':(vel_x,vel_y), 'acceleration':(acc_x,acc_y)})
            line_index += 1
            veh_proc = Process(target=run_vehicle, args=(veh_num, pid, 
                fleets[i]['lane_id'], fleets[i]['des_lane_id'], 
                fleets[i]['fid'], fleets[i]['fleet_len'],
                vid, (loc_x,loc_y), (vel_x,vel_y), (acc_x,acc_y), rounds, 
                location_info, velocity_info, finished_list))
            veh_processes.append(veh_proc)

    for veh_proc in veh_processes:
        veh_proc.start()
    assert(len(veh_processes) == veh_num)

    while True:
        try:
            for proc in veh_processes:
                proc.join(timeout=2)
        except KeyboardInterrupt:
            for proc in veh_processes:
                proc.terminate()
                proc.join()


if __name__ == '__main__':
    main()
    