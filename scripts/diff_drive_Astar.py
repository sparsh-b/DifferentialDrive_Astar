import copy
import matplotlib.pyplot as plt
import os
from tqdm.autonotebook import tqdm
import cv2 
from math import sqrt, radians, cos, sin, degrees, pow
import numpy as np
import argparse
import heapq as hq
from config_space import main


class Node():
    def __init__(self, x, y, th) -> None:
        self.x = x
        self.y = y
        self.th = th

    def print(self):
        print(self.x, self.y, self.th)

    def __eq__(self, __o) -> bool:
        if (self.x == __o.x) and (self.y == __o.y) and (self.th == __o.th):
            return True
        else:
            return False

def euclidean_dist(node1, node2, node1_y= -1):
    try:
        return sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
    except:
        return sqrt((node1 - node2.x)**2 + (node1_y - node2.y)**2)

def close_to_goal(curr_node, goal_node, threshold):
    if euclidean_dist(curr_node, goal_node) <= threshold:
        return True
    else:
        return False

def backtrack(final_node):
    path = []
    curr_node = (final_node.x, final_node.y, final_node.th)
    while (nodes[curr_node]['parentx'] != -1 or nodes[curr_node]['parenty'] != -1):
        path.append(curr_node)
        curr_node = (nodes[curr_node]['parentx'], nodes[curr_node]['parenty'], nodes[curr_node]['parentth'])
    path.append(curr_node)
    return path

def round_off(child_x, child_y, child_th):
    while child_th < 0:
        child_th += 360
    while child_th >= 360:
        child_th -= 360
    if child_th%30 > 15: # [0,15] degrees -> child_th=0, (15,45]->1, (45,75]->2 & so on
        child_th += 30
    child_th = (child_th // 30)*30
    child_x = round(child_x)
    child_y = round(child_y)
    return child_x, child_y, child_th

def check_if_duplicate(child_x, child_y, child_th, nodes):
    if (child_x, child_y, child_th) in nodes:
        return True
    else:
        return False

def generate_children(parent, goal_node, rpms, num_generated, nodes, nav_space, open_q):
    rpm0 = [   0   , rpms[0]]
    rpm1 = [rpms[0],    0   ]
    rpm2 = [rpms[0], rpms[0]]
    rpm3 = [   0   , rpms[1]]
    rpm4 = [rpms[1],    0   ]
    rpm5 = [rpms[1], rpms[1]]
    rpm6 = [rpms[1], rpms[0]]
    rpm7 = [rpms[0], rpms[1]]
    eight_rpms = [rpm0,rpm1,rpm2,rpm3,rpm4,rpm5,rpm6,rpm7]
    # eight_rpms = [rpm7]
    r = 38 #in mm
    L = 354 #in mm
    dt = 0.1 #in seconds
    x = parent.x
    y = parent.y
    th = radians(parent.th)
    parent_cost = nodes[(parent.x, parent.y, parent.th)]['cost']
    parent_euclidean = euclidean_dist(parent, goal_node)
    child_common_cost = parent_cost - parent_euclidean
    for child_rpm in eight_rpms:
        t = 0
        child_x = parent.x
        child_y = parent.y
        child_th= th
        additional_cost = 0
        common_multiple = 0.5*r * (child_rpm[0] + child_rpm[1]) * dt
        dth = (r/L) * (child_rpm[0] - child_rpm[1]) * dt
        while t<1:
            t += dt
            dx = common_multiple * cos(child_th)
            dy = common_multiple * sin(child_th)
            xs = copy.copy(child_x)
            ys = copy.copy(child_y)
            child_x += dx
            child_y += dy
            child_th+= dth
            additional_cost += sqrt(pow((common_multiple * cos(child_th)),2) + pow((common_multiple * sin(child_th)),2))
            plt.plot([xs, child_x], [ys, child_y], color="blue")
        child_x, child_y, child_th = round_off(child_x, child_y, degrees(child_th))
        # child_x, child_y, child_th, additional_cost = eight_moves(rpms, parent)
        new_cost = child_common_cost + additional_cost + euclidean_dist(child_x, goal_node, child_y)

        if (child_x < 0) or (child_y < 0):
            continue
        if (child_x >= 10000) or (child_y >= 10000):
            continue
        if nav_space[10000-1-child_y, child_x] == 0:
            print('row, col:', 10000-1-child_y, child_x)
            continue
        print(child_x, child_y, child_th, '<-', parent.x, parent.y, parent.th)

        if check_if_duplicate(child_x, child_y, child_th, nodes):
            prev_cost = nodes[(child_x, child_y, child_th)]['cost']
            if new_cost < prev_cost:
                # num_generated += 1
                nodes[(child_x, child_y, child_th)] = {'parentx': parent.x, 'parenty': parent.y, 'parentth': parent.th, 'cost': new_cost}
                for idx in range(len(open_q)):
                    if open_q[idx][2] == Node(child_x, child_y, child_th):
                        open_q[idx] = (new_cost, open_q[idx][1], open_q[idx][2])
                        hq.heapify(open_q)
        else:
            num_generated += 1
            nodes[(child_x, child_y, child_th)] =  {'parentx': parent.x, 'parenty': parent.y, 'parentth': parent.th, 'cost': new_cost}
            hq.heappush(open_q, (new_cost, num_generated, Node(child_x, child_y, child_th)))
    return num_generated

def valid_start_and_goal(start_node, goal_node, nav_space):
    if (nav_space[start_node.x, start_node.y] == 1) and (nav_space[goal_node.x, goal_node.y] == 1):
        return True
    return False

def visualize(grid_size, grid, path):
    prev_node = path[-1]
    req_rpms = []
    for path_node in path[::-1]:
        plt.plot([prev_node[0], path_node[0]], [prev_node[1], path_node[1]], color="red")
        
        rpm0 = [   0   , rpms[0]]
        rpm1 = [rpms[0],    0   ]
        rpm2 = [rpms[0], rpms[0]]
        rpm3 = [   0   , rpms[1]]
        rpm4 = [rpms[1],    0   ]
        rpm5 = [rpms[1], rpms[1]]
        rpm6 = [rpms[1], rpms[0]]
        rpm7 = [rpms[0], rpms[1]]
        eight_rpms = [rpm0,rpm1,rpm2,rpm3,rpm4,rpm5,rpm6,rpm7]
        r = 38 #in mm
        L = 354 #in mm
        dt = 0.1 #in seconds
        th = radians(prev_node[2])
        for child_rpm in eight_rpms:
            t = 0
            child_x = prev_node[0]
            child_y = prev_node[1]
            child_th= th
            additional_cost = 0
            common_multiple = 0.5*r * (child_rpm[0] + child_rpm[1]) * dt
            dth = (r/L) * (child_rpm[0] - child_rpm[1]) * dt
            while t<1:
                t += dt
                dx = common_multiple * cos(child_th)
                dy = common_multiple * sin(child_th)
                child_x += dx
                child_y += dy
                child_th+= dth
                additional_cost += sqrt(pow((common_multiple * cos(child_th)),2) + pow((common_multiple * sin(child_th)),2))
            child_x, child_y, child_th = round_off(child_x, child_y, degrees(child_th))
            if (child_x, child_y, child_th) == path_node:
                req_rpms.append(child_rpm)
                break
        prev_node = path_node
        
    return req_rpms

    # grid_disp = grid.astype(np.uint8)
    # grid_disp = cv2.resize(grid_disp, (int(grid_size/10), int(grid_size/10)))
    # cv2.imshow('grid_disp', grid_disp)
    # cv2.waitKey(0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--start', nargs='+', type=int, help='enter as:x y theta')
    parser.add_argument('--goal', nargs='+', type=int, help='enter as:x y')
    parser.add_argument('--rpm', nargs='+', type=int, help='enter as:rpm1 rpm2')
    parser.add_argument('--clearance', default=5, type=int, help='clearance around the robot')
    args = parser.parse_args()

    rpms = args.rpm
    start = args.start
    goal = args.goal
    clearance = args.clearance
    threshold = 100 # in mm - distance b/w bot & goal to consider that bot arrived at goal
    reached_goal = False
    fig, ax = plt.subplots()

    start_node = Node(*round_off(*start))
    goal_node = Node(*goal, -1)
    open_q = []
    closed_q = []
    hq.heappush(open_q, (0, 0, start_node)) #cost, added_index, node
    nodes = {} # maintains track of all generated nodes
    # key:tuple having x,y&th coordinates of a node, value:dictionary containing parent node & cost
    nodes[tuple(round_off(start_node.x, start_node.y, start_node.th))] = {'parentx': -1, 'parenty': -1, 'parentth':-1, 'cost': euclidean_dist(start_node, goal_node)}
    num_generated = 1

    nav_space, grid = main(clearance, False) # navigable space
    if not valid_start_and_goal(start_node, goal_node, nav_space):
        print("Start & goal nodes aren't in valid locations. Please try again.")
        exit()
    while (not reached_goal) and (len(open_q) != 0):
        _, _, curr_node = hq.heappop(open_q)
        closed_q.append(curr_node)
        if close_to_goal(curr_node, goal_node, threshold):
            path = backtrack(curr_node)
            reached_goal = True
            print('\n\nGenerated Path:', path[::-1])
            req_rpms = visualize(10000, grid, path)
            print('\n\nreq_rpms', req_rpms)
            break
        else:
            num_generated = generate_children(curr_node, goal_node, rpms, num_generated, nodes, nav_space, open_q)
            # break
            # if num_generated > 20:
            #     break

    plt.grid()
    ax.set_aspect('equal')
    plt.xlim(0,10000)
    plt.ylim(0,10000)
    plt.title('How to plot a vector in matplotlib ?',fontsize=10)
    plt.show()
    plt.savefig('path_{}-{}-{}_to_{}-{}.png'.format(*round_off(*start), *goal))
    plt.close()