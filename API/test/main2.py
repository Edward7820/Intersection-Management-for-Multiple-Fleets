import sys
import time
import argparse
import itertools
import json
import zenoh
from zenoh import Reliability, Sample
from datetime import datetime
from Vehicle import MyVehicle, Leader


def main():
    session = zenoh.open()
    # leader1 = Leader(session=session, speed=20, location=[1, 20], acceleration = 0.5, vehicle_id = 1, fleet_id = 1)
    # while True:
    #     leader1.pub_state()
    leader2 = Leader(session=session, speed=20, location=[20, 1], acceleration = 0.5, vehicle_id = 1, fleet_id = 2)
    print("start")
    while True:
        leader2.sub_state(fleet_id=1)
        print("fuck")
        time.sleep(1)


if __name__ == '__main__':
    main()


'''
====================================================================
'''

# print("Opening session...")
# session = zenoh.open()
# key = f"state/**"
# print("Declaring Subscriber on '{}'...".format(key))


# def listener(sample: Sample):
#     print(f">> [Subscriber] Received {sample.kind} ('{sample.key_expr}': '{sample.payload.decode('utf-8')}')")
    

# # WARNING, you MUST store the return value in order for the subscription to work!!
# # This is because if you don't, the reference counter will reach 0 and the subscription
# # will be immediately undeclared.
# sub = session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())

# print("Enter 'q' to quit...")
# c = '\0'
# while c != 'q':
#     c = sys.stdin.read(1)
#     if c == '':
#         time.sleep(1)

# # Cleanup: note that even if you forget it, cleanup will happen automatically when 
# # the reference counter reaches 0
# sub.undeclare()
# session.close()
