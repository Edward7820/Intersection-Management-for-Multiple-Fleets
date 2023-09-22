from API.Vehicle import MyVehicle, Leader, Member
import zenoh
import time

session = zenoh.open()
vehicle = Member(session, (10, 0), (5, -2), (0,0), 0, 2, 2, 0, 0.1)
while True:
    vehicle.step_vehicle()
    vehicle.pub_state()
    time.sleep(2)