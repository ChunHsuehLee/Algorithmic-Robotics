import numpy as np
from utils import get_collision_fn_PR2, load_env, execute_trajectory, draw_sphere_marker
from pybullet_tools.utils import connect, disconnect, get_joint_positions, wait_if_gui, set_joint_positions, joint_from_name, get_link_pose, link_from_name
from pybullet_tools.pr2_utils import PR2_GROUPS
import time
import random

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
    
    initial_pose = tuple(get_joint_positions(robots['pr2'], base_joints))

    # odometry
    def Odometry(input, previous_pose):
        c1 = 0.05
        c2 = 0.05
        c3 = 0.05
        c4 = 0.05
        delta_t = 1
        v = input[0]
        w = input[1]
        R = np.array([[(c1* abs(v)+c2*abs(w))**2, 0], [0, (c3* abs(v)+c4*abs(w))**2]])
        rand = np.random.multivariate_normal(input, R)
        v_actual = rand[0]
        w_actual = rand[1]
        print("v_actual", v_actual)
        print("w_actual", w_actual)
        delta_x = v_actual*delta_t*np.cos(previous_pose[2] + w_actual*delta_t)
        delta_y = v_actual*delta_t*np.sin(previous_pose[2] + w_actual*delta_t)
        delta_theta = w_actual * delta_t
        delta_pose = np.array([delta_x, delta_y, delta_theta])
        new_pose = previous_pose + delta_pose
        return new_pose
    initial_pose = np.array(initial_pose)
    print(initial_pose)
    input = [1, 0.04]
    new_pose = Odometry(input, initial_pose)
    print(new_pose)

    wait_if_gui()
    disconnect()

if __name__ == '__main__':
    main()