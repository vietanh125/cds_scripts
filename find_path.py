#!/usr/bin/env python
import numpy as np
from time import time
import rospkg
rospack = rospkg.RosPack()
path = rospack.get_path('team107') + '/scripts/'

number_nodes = 13

rules = [[11, 1, 9], [9, 1, 11], [11, 2, 10], [8, 9, 7], [7, 9, 8]]
changes = []

matrix = np.ones((number_nodes, number_nodes))*10000

with open(path + 'path_direction_new.txt', 'r') as f:
    lines = f.readlines()
for line in lines:
    road = line.split()
    matrix[int(road[0])-1][int(road[1])-1] = 10

coordinate = []
with open(path + 'coordinate.txt', 'r') as f:
    lines = f.readlines()
for line in lines:
    x, y = [int(p) for p in line.split()]
    coordinate.append([x, y])

directions = []
commands = []
with open(path + 'direction.txt', 'r') as f:
    lines = f.readlines()
for line in lines:
    prev, now, future, command = [int(p) for p in line.split()]
    directions.append([prev, now, future])
    commands.append(command)

roads = []
status_s = []
with open(path + 'status_road.txt', 'r') as f:
    lines = f.readlines()
for line in lines:
    array = line.split()
    now, future = [int(p) for p in array[:2]]
    status = array[2:]
    roads.append([now, future])
    status_s.append(status)

def get_min_heuristic(present, end, candidate, path):
    coordinate_tmp = coordinate[:]
    # print(candidate, matrix[2-1][10-1], matrix[2-1][1-1])

    if present == 13 or present == 12:
        if end == 8 or end == 9 or end == 7:
            coordinate_tmp[7 - 1] = [2.1, 7]
            coordinate_tmp[8 - 1] = [1.9, 7]
            coordinate_tmp[9 - 1] = [1.85, 7]

    if present == 1:
        if end == 3 or end == 4:
            coordinate_tmp[3 - 1] = [1, 2]
            coordinate_tmp[4 - 1] = [3, 2]

    if present == 6:
        if end == 3 or end == 4:
            coordinate_tmp[3 - 1] = [1.9, 7]
            coordinate_tmp[4 - 1] = [2.1, 7]

    if present == 11:
        if end == 10:
            coordinate_tmp[10 - 1] = [0, 1]

    min_value = 10000
    min_index = -1
    for c in candidate:
        ratio = 1
        # if c+1 not in path:
        for rule in rules:
            if present == rule[0] and end == rule[2] and c+1 == rule[1]:
                ratio = 10000

        heuristic = ratio*np.linalg.norm(np.array(coordinate_tmp[end-1])-np.array(coordinate_tmp[c]))
        if heuristic < min_value:
            min_index = c
            min_value = heuristic
    return min_index + 1

# matrix = np.array([[10000, 10000, 1, 4, 10000, 10000, 10000],
#                     [10000, 10000, 1, 10000, 10000, 4, 10000],
#                     [1, 1, 10000, 2, 3, 10000, 10000],
#                     [4, 10000, 2, 10000, 10000, 10000, 1],
#                     [10000, 10000, 3, 10000, 10000, 2, 10000],
#                     [10000, 4, 10000, 10000, 2, 10000, 1],
#                     [10000, 10000, 10000, 1, 10000, 1, 10000]])

def find_path_ha(start, end):
    global changes
    node_present = start
    node_previous = 0
    distance = 0
    path = [start]
    start_time = time()
    while node_present != end:
        if time() - start_time > 1:
            return "Time out!"
        # print(node_present)
        min_tmp_distance = matrix[node_present-1][np.argmin(matrix[node_present-1])]
        candidate = np.where(matrix[node_present-1] == min_tmp_distance)[0]
        # print(matrix[10][4])
        #Return last changes
        for change in changes:
            matrix[change[0]][change[1]] = change[2]

        changes = []
        node_previous = node_present
        node_present = get_min_heuristic(node_present, end, candidate, path)
        path.append(node_present)
        distance += matrix[node_previous-1][node_present-1]

        # Adding some changes tmp for matrix
        changes.append([node_present-1, node_previous-1, matrix[node_present-1][node_previous-1]]) #no return back

        for rule in rules:
            if node_present == rule[1] and node_previous == rule[0]:
                changes.append([rule[1] - 1, rule[2] - 1, matrix[rule[1] - 1][rule[2] - 1]])

        # change command
        for change in changes:
            matrix[change[0]][change[1]] = 10000
        # print(matrix[2-1][10-1])
        # print("Changes: ", changes)

    return path

def request(s):
    global changes
    s = [5, 11] + [int(x) for x in s.split("-")]
    print("Request: ", s)
    final_path = []
    for i in range(len(s)-1):
        if i == len(s)-2:
            changes.append([1, 9, 10])
        # print(s[i], s[i + 1])
        this_path = find_path_ha(s[i], s[i+1])

        if this_path == "Time out!":
            return "Time out!"
        else:
            final_path = final_path[:-1]
            final_path += this_path

    return final_path

def command(request):
    final = []
    status = []
    off = []
    for i in range(len(request)-2):
        if [request[i], request[i+1], request[i+2]] in directions:
            final.append(commands[directions.index([request[i], request[i+1], request[i+2]])])
            if [request[i + 1], request[i + 2]] in roads:
                status_car = status_s[roads.index([request[i + 1], request[i + 2]])]
                status.append(off + status_car)
                off = []
                if len(status_car) > 0:
                    if "obstacle_on" in status_car:
                        off += ["obstacle_off"]
                    if "bridge_on" in status_car:
                        off += ["bridge_off"]
                    if "left_stick" in status_car:
                        off += ["stick_off"]
                    if "right_stick" in status_car:
                        off += ["stick_off"]
            else:
                status.append(off + [])
                off = []
    status[-1] += ["parking_on"]
    return final, status

# Test random
# while True:
#     points = np.random.choice(9, 4, replace=False) + 1
#     r = ""
#     for p in points:
#         r += str(p) + "-"
#     if request(r[:-1]) == "Time out!":
#         print("TIME OUT", r[:-1])
#         break


def get_command(request_string, round):
    request_string = request_string.replace("3", "100")
    request_string = request_string.replace("4", "3")
    request_string = request_string.replace("100", "4")
    
    if round == 1:
        status = [["obstacle_on, circle"], ["obstacle_off", "right_stick"], [], [], ["parking_on", "right_stick"]]
        if request_string[0] == "1":
            print("Request", [1, 2, 4, 3, 5])
            path_found = [5, 11, 1, 2, 11, 5, 12, 4, 13, 3 , 12, 5]
            print("Path found: ", path_found)
            command_out = [-1, -1, -1, 1, -1]
            print("Command out: ", command_out)
            print("Status: ", status)
        else:
            print("Request", [2, 1, 4, 3, 5])
            path_found = [5, 11, 2, 1, 11, 5, 12, 4, 13, 3, 12, 5]
            print("Path found: ", path_found)
            command_out = [1, 1, -1, 1, -1]
            print("Command out: ", command_out)
            print("Status: ", status)
    else:
        path_found = request(request_string)
        print("Path found: ", path_found)

        command_out, status = command(path_found)
        status[-1] += ["left_stick"]
        print("Command out: ", command_out)
        print("Status: ", status)

    return np.array(command_out)*(-1), status
