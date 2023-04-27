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

def run_vehicle(veh_num: int, pid: int, lane_id: int, des_lane_id: int, fid: int, 
    fleet_len: int,  vid: int, location: tuple, velocity: tuple, 
    acceleration: tuple, rounds):
    ## TODO
    cur_round = 1
    while(True):
        rounds[pid] = cur_round

        ## wait for other processes
        waiting = True
        while waiting:
            for r in rounds:
                if r != cur_round:
                    continue
                waiting = False
        
        print(cur_round)
        cur_round += 1


def main():
    parser = ArgumentParser()
    parser.add_argument("--input_file",type=str, default="input.txt")
    parser.add_argument("--delta_t",type=float, default=0.1)
    parser.add_argument("--max_speed",type=float,default=16)
    parser.add_argument("--max_acceleration",type=float,default=3)
    parser.add_argument("--min_acceleration",type=float,default=-3)
    args = parser.parse_args()

    with open(args.input_file, 'r') as f:
        input_words = f.read()
    input_lines = input_words.split('\n')
    basic_info = input_lines.split(' ')
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
            fleets[i]['vehicle'].append({'vid':vid, 'location':(loc_x,loc_y),
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

if __init__ == '__main__':
    main()
    