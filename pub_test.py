import sys
import time
from argparse import ArgumentParser
import itertools
import json
import zenoh
from zenoh import Config
from datetime import datetime
from API.Vehicle import MyVehicle, Leader
import os, signal
from multiprocessing import Process, Array
from typing import List, Dict, Tuple

session = zenoh.open(Config())
myvehicle = Leader(session, (10,0), (0,0), (0,0), 0, 0, 0, 0.1, 2, 5)
while (True):
    myvehicle.pub_schedule_map()
    time.sleep(1)