from API.Vehicle import MyVehicle, Leader, Member
import zenoh
import time

session = zenoh.open()
vehicle = Member(session, (10, 0), (1, -2), (0,0), 1, 2, 2, 0, 0.1)
while True:
    vehicle.pub_state()
    print(vehicle.state_record)
    time.sleep(2)