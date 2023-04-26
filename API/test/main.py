import sys
import time
import argparse
import itertools
import json
import zenoh
from datetime import datetime
from Vehicle import MyVehicle, Leader


def main():
    session = zenoh.open()
    while True:
        for i in range(5):
            vehicle = MyVehicle(session=session, speed=20-i, location=[i, 20], acceleration = 0.5, vehicle_id = i, fleet_id = 1)
            vehicle.pub_state()
            time.sleep(1)
    # vehicle = MyVehicle(session=session, speed=20-1, location=[1, 20], acceleration = 0.5, vehicle_id = 1, fleet_id = 1)
    # while True:
    #     vehicle.pub_state()
    #     time.sleep(1)


if __name__ == '__main__':
    main()