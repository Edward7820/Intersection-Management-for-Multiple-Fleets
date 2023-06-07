from argparse import ArgumentParser
from API.Scheduler import Scheduler
from API.Simulator import simulate_passing_order

def main():
    states = dict()

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
            states[(fleets[i]['lane_id'],fleets[i]['fid'],vid)] = {'location':(loc_x,loc_y),
                                                                   'velocity':(vel_x,vel_y),
                                                                   'acceleration':(acc_x,acc_y),
                                                                   'des_lane_id':fleets[i]['des_lane_id']}
            line_index += 1

    conflict_zones = [(0,0,4,4),(-4,0,0,4),(-4,-4,0,0),(0,-4,4,0)]
    scheduler = Scheduler(conflict_zones, states, 1.0, 0, 0, 1.2)
    passing_order = scheduler.search(200)
    print(passing_order)
    t_assign = scheduler.passing_order_to_time_slot(passing_order)
    print(t_assign)
    

if __name__ == '__main__':
    main()