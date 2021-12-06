import numpy as np
from utils import get_collision_fn_PR2, load_env, execute_trajectory, draw_sphere_marker
from pybullet_tools.utils import connect, disconnect, get_joint_positions, wait_if_gui, set_joint_positions, joint_from_name, get_link_pose, link_from_name
from pybullet_tools.pr2_utils import PR2_GROUPS
import time

### YOUR IMPORTS HERE ###
from queue import PriorityQueue
#defines a basic node class
class Node:
    def __init__(self,x_in,y_in,theta_in, g_in):
        self.x = x_in
        self.y = y_in
        self.theta = theta_in
        self.g = g_in
        # self.id = id_in
        # self.parentid = parentid_in

    def printme(self):
        print("\tx =", self.x, "y =",self.y, "theta =", self.theta, "g cost:", self.g)



#########################

def main(screenshot=False):
    # initialize PyBullet
    connect(use_gui=True)
    # load robot and obstacle resources
    robots, obstacles = load_env('pr2doorway.json')

    # define active DoFs
    base_joints = [joint_from_name(robots['pr2'], name) for name in PR2_GROUPS['base']]

    collision_fn = get_collision_fn_PR2(robots['pr2'], base_joints, list(obstacles.values()))
    # Example use of collision checking
    # print("Robot colliding? ", collision_fn((0.5, -1.3, -np.pi/2)))

    # Example use of setting body poses
    # set_pose(obstacles['ikeatable6'], ((0, 0, 0), (1, 0, 0, 0)))

    # Example of draw 
    # draw_sphere_marker((0, 0, 1), 0.1, (1, 0, 0, 1))
    
    start_config = tuple(get_joint_positions(robots['pr2'], base_joints))
    goal_config = (2.6, -1.3, -np.pi/2)
    path = []
    start_time = time.time()
    ### YOUR CODE HERE ###
    plot_height = 0.5
    def calH(current_config):
        return np.sqrt( (current_config[0] - goal_config[0])**2 + (current_config[1] - goal_config[1])**2 \
        + min(abs(current_config[2] - goal_config[2]), 2*np.pi - abs(current_config[2] - goal_config[2]))**2)

    def calG(parent_config, current_config): # accumulate
        return np.sqrt( (parent_config[0] - current_config[0])**2 + (parent_config[1] - current_config[1])**2 \
        + min(abs(parent_config[2] - current_config[2]), 2*np.pi - abs(parent_config[2] - current_config[2]))**2)
        
    def calF(parent_config, current_config):
        return calG(parent_config, current_config) + calH(current_config)

    def warp_to_pi(theta):
        while theta > np.pi:
            theta -= 2 * np.pi
        while theta < -np.pi:
            theta += 2 * np.pi
        return theta
    
    def reach_goal(current_config):
        TOLERANCE = 1
        if calH(current_config) <= TOLERANCE:
            return True
        else:
            return False

    def trace_path(id_cnt, parents):
        path = []
        draw_path = []
        goal_idx = id_cnt
        while parents[goal_idx][0] != -1:
            parent_idx = parents[goal_idx][0]
            draw_path.append((parents[goal_idx][1][0], parents[goal_idx][1][1], plot_height))
            path.append(parents[goal_idx][1])
            goal_idx = parent_idx

        path = path[::-1]
        draw_path = draw_path[::-1]
        return path, draw_path
     
    #initialize the priority queue
    closed_list = []
    open_list = [] # add g_cost, config
    id_cnt = 0
    ###### 4-connect ######
    # x_step = [0.0, 0.0, 1.0, -1.0]
    # y_step = [1.0, -1.0, 0.0, 0.0]

    ###### 8-connect ######
    x_step = [0.0, 0.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0]
    y_step = [1.0, -1.0, 0.0, 0.0, 1.0, -1.0, -1.0, 1.0]

    theta_step = []
    STEP_SIZE = 0.15
    THETA_DIVIDE_SIZE = 4
    parents = {}

    obstacle_list = []
    free_list = []
    draw_path = []
    
    TOLERANCE = 0.1
    for i in range(THETA_DIVIDE_SIZE):
        theta_step.append(2 * np.pi/THETA_DIVIDE_SIZE * (i+1))

    goal_flag = False

    q = PriorityQueue()
    G = calG(start_config, start_config)
    q.put((calH(start_config) + G, id_cnt, Node(start_config[0], start_config[1], start_config[2], G)))
    open_list.append((calG(start_config, start_config), start_config))
    parents[id_cnt] = (-1, start_config)


    while (not q.empty()) and (goal_flag == False):
        current_item = q.get()

        current_config = (current_item[2].x, current_item[2].y, current_item[2].theta)

        closed_list.append(current_config) 
        
        for i in range(len(x_step)):
            if goal_flag == False:
                x = current_item[2].x + x_step[i] * STEP_SIZE
                y = current_item[2].y + y_step[i] * STEP_SIZE
                
                for j in range(len(theta_step)):
                    id_cnt += 1
                    theta = warp_to_pi(current_item[2].theta + theta_step[j])
                    new_config = (x, y, theta)
                    if collision_fn(new_config):
                        obstacle_list.append((new_config[0], new_config[1], plot_height))
                        continue
                    if new_config in closed_list:
                        continue




                    new_G = calG((current_item[2].x, current_item[2].y, current_item[2].theta), new_config) + current_item[2].g
                    new_Node = Node(x, y, theta, new_G)
                    H = calH(new_config)
                    if(H <= TOLERANCE):
                        print("Goal Reached")
                        print("Goal: ", new_config)
                        goal_flag = True
                        parents[id_cnt] = (current_item[1], current_config)
                        parents[id_cnt + 1] = (id_cnt, new_config)

                        path, draw_path = trace_path(id_cnt + 1, parents)
                        break
                    free_list.append((new_config[0], new_config[1], plot_height))
                    q.put((H + new_G, id_cnt, new_Node))
                    parents[id_cnt] = (current_item[1], current_config)
                    closed_list.append(new_config)
    
    # plot sphere
    obstacle_list = list(set(obstacle_list))
    free_list = list(set(free_list))
    print("obstacle length: ", len(obstacle_list))
    print("free length: ", len(free_list))

    print("Planner run time: ", time.time() - start_time)
    for obstacle_config in obstacle_list:
        draw_sphere_marker(obstacle_config, 0.05, (1, 0, 0, 1))
    for free_config in free_list:
        draw_sphere_marker(free_config, 0.05, (0, 0, 1, 1))
    for config in draw_path:
        draw_sphere_marker(config, 0.1, (0, 0, 0, 1))

    
    ######################
    # Execute planned path
    execute_trajectory(robots['pr2'], base_joints, path, sleep=0.2)
    # Keep graphics window opened
    wait_if_gui()
    disconnect()

if __name__ == '__main__':
    main()