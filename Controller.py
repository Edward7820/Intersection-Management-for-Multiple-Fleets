import sys
import time
from argparse import ArgumentParser
import itertools
import json
import zenoh
from datetime import datetime
from API.Vehicle import MyVehicle, Leader
from API.Scheduler import arrival_time
import os, signal
import math

FLEET_LENGTH = 5
FIRST = True
scheduling = None
TURN = None
SLOT = None
deadline = None

#conflict zone
MAX_X = 2
MIN_X = -2
MAX_Y = 2
MIN_Y = -2


def control(vehicle: MyVehicle,max_speed,max_acceleration,min_acceleration):
    global FIRST
    global scheduling
    global TURN
    global SLOT
    global deadline
    #global pass_through
    if FIRST:
        scheduling = vehicle.result[1]
        deadline1 = scheduling[-1] + 10
        deadline2 = scheduling[-1] + 20
        deadline3 = scheduling[-1] + 25
        deadline4 = scheduling[-1] + 30
        scheduling.append(deadline1)
        scheduling.append(deadline2)
        scheduling.append(deadline3)
        scheduling.append(deadline4)
        TURN = vehicle.result[0]
        SLOT = 0
        deadline = scheduling[0]
        FIRST = False
        pass_through = [False]*(2*FLEET_LENGTH)
    elif vehicle.tick > deadline and SLOT < len(scheduling)-1:
        SLOT += 1
        TURN = 2 if TURN==1 else 1
        deadline = scheduling[SLOT]
    elif vehicle.tick > deadline and SLOT >= len(scheduling)-1:
        assert(vehicle.finish) #beyond the schedule

    # Update its own state
    original_speed = vehicle.speed
    vehicle.speed += (vehicle.acceleration*vehicle.delta)
    if vehicle.speed < 0: # the vehicle has stopped
        vehicle.speed = 0
    elif vehicle.speed > max_speed:
        vehicle.speed = max_speed
    forward_distance = vehicle.delta*(vehicle.speed+original_speed)/2
    if vehicle.fleet_id == 1:
        vehicle.location[0] += forward_distance
    else:
        vehicle.location[1] -= forward_distance

    if vehicle.finish:
        vehicle.acceleration = max_acceleration
        return

    # Get its own vid
    vid = (vehicle.fleet_id-1)*FLEET_LENGTH+vehicle.vehicle_id-1

    # Get the state of front car
    front_veh_state = None
    front_vid = vid - 1
    if vehicle.vehicle_id > 1 and vehicle.state_record[front_vid] is not None:
        front_veh_state = vehicle.state_record[front_vid]
        front_veh_location = front_veh_state['location']
        front_veh_speed = front_veh_state['speed']
        front_veh_acceleration = front_veh_state['acceleration']
        front_veh_finish = front_veh_state['finish']
        if vehicle.fleet_id==1:
            front_veh_dist = abs(vehicle.location[0]-front_veh_location[0])
        else:
            front_veh_dist = abs(vehicle.location[1]-front_veh_location[1])

    # Estimate the arrival time
    remain_dist = None
    if vehicle.fleet_id == 1:
        remain_dist = abs(vehicle.location[0]-MAX_X)
    else:
        remain_dist = abs(vehicle.location[1]-MIN_Y)

    if front_veh_state is None or vehicle.speed==0:
        est_veh_arrival_time=vehicle.tick+arrival_time(remain_dist,vehicle.speed,max_acceleration)
    elif front_veh_finish==1:
        est_veh_arrival_time=vehicle.tick+arrival_time(remain_dist,vehicle.speed,max_acceleration)
    elif front_veh_dist/vehicle.speed<2:
        est_veh_arrival_time=None
    # elif front_veh_dist/vehicle.speed<3:
    #     est_veh_arrival_time=vehicle.tick+arrival_time(remain_dist,front_veh_speed,0)
    else:
        est_veh_arrival_time=vehicle.tick+arrival_time(remain_dist,front_veh_speed,max_acceleration)
    # print(f"fleet {vehicle.fleet_id}, vehicle {vehicle.vehicle_id}, estimated arrival time {est_veh_arrival_time} (current tick {vehicle.tick})")

    # Determine if it is possible to cross the intersection in this time slot
    cross = False
    if TURN == vehicle.fleet_id:
        if est_veh_arrival_time is not None and est_veh_arrival_time<deadline:
            cross = True

    if cross:
        vehicle.acceleration = max_acceleration
    else:
        if TURN == vehicle.fleet_id:
            vehicle.acceleration = 0
        else:
            vehicle.acceleration = min_acceleration/2

    if vehicle.fleet_id == 1 :
        if vehicle.location[0] > MAX_X  and not vehicle.finish:
            vehicle.finish = True
            vehicle.waiting = vehicle.tick
            print(f"fleet {vehicle.fleet_id}, vehicle {vehicle.vehicle_id} finished at tick {vehicle.tick}")
    elif vehicle.fleet_id == 2:
        if vehicle.location[1] < MIN_Y and not vehicle.finish:
            vehicle.finish = True
            vehicle.waiting = vehicle.tick
            print(f"fleet {vehicle.fleet_id}, vehicle {vehicle.vehicle_id} finished at tick {vehicle.tick}")


def collision_detect(vehicle: MyVehicle):
    if vehicle.finish:
        return
    time.sleep(0.5)
    vid = (vehicle.fleet_id-1)*FLEET_LENGTH + (vehicle.vehicle_id-1)
    for i in range (vid+1, 2*FLEET_LENGTH):
        if vehicle.state_record[i] is None or vehicle.state_record[i]["finish"]==1:
            continue
        car1, car2 = vehicle.location, vehicle.state_record[i]["location"]
        distance = math.sqrt((car1[0]-car2[0])**2 + (car1[1]-car2[1])**2)
        if distance <= 3:
            file = open("record.txt", "a")
            file.write(f"================== Collision Detected Orz... {i} & {vid} ==================\n")
            # file.write(f"{distance}\n")
            file.write(f"{vehicle.state_record[i]}\n")
            file.write(f"{vehicle.state_record[vid]}\n")
            file.close()