from __future__ import division
import pybullet as p
import pybullet_data
import numpy as np
import time
import argparse


UR5_JOINT_INDICES = [0, 1, 2]


def set_joint_positions(body, joints, values):
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        p.resetJointState(body, joint, value)


def draw_sphere_marker(position, radius, color):
   vs_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
   marker_id = p.createMultiBody(basePosition=position, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vs_id)
   return marker_id


def remove_marker(marker_id):
   p.removeBody(marker_id)


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--birrt', action='store_true', default=False)
    parser.add_argument('--smoothing', action='store_true', default=False)
    args = parser.parse_args()
    return args


def rrt(start_conf, goal_conf, step_size, collision_fn):
    ###############################################
    # TODO your code to implement the rrt algorithm
    ###############################################
    T = [[start_conf], {}]
    step_check_depth = 3
    while True:
        # Check if goal is in the vicinity
        nearest2goal, dist2goal = get_closest_neighbor(T, goal_conf)
        if dist2goal < step_size and is_collision_free(nearest2goal, goal_conf, step_check_depth, collision_fn):
            draw_line(nearest2goal, goal_conf, [0,1,0], 3)
            break

        if np.random.uniform(size=1) < 0.05:
            q_rand = goal_conf
        else:
            q_rand = generate_random_point()

        q_near, _ = get_closest_neighbor(T, q_rand)
        q_new = progress_along(q_near, q_rand, step_size)
        if is_collision_free(q_near, q_new, step_check_depth, collision_fn):
            T[0].append(q_new)
            T = add_edge(T, [q_near, q_new])
            draw_line(q_near, q_new, [0,1,0], 3)

    path_conf = [nearest2goal, goal_conf]

    while True:
        if path_conf[0] == start_conf:
            break
        path_conf.insert(0, T[1][tuple(path_conf[0])][0])
    return path_conf


def birrt(start_conf, goal_conf, step_size, collision_fn):
    #################################################
    # TODO your code to implement the birrt algorithm
    #################################################
    T1_T2 = [ [[start_conf], {}], [[goal_conf], {}] ]
    T_colors = [ [0,1,0], [0,0,1] ]
    step_check_depth = 3
    iter_count = 0
    while True:
        min_vt_pair, min_dist = tree_dist(T1_T2)
        if min_dist < step_size and is_collision_free(min_vt_pair[0], min_vt_pair[1], step_check_depth, collision_fn):
            draw_line(min_vt_pair[0], min_vt_pair[1], T_colors[0], 3)
            break

        T1_idx = iter_count % 2
        T2_idx = (iter_count + 1) % 2

        q_rand = generate_random_point()
        q_near_1, _ = get_closest_neighbor(T1_T2[T1_idx], q_rand)
        q_new_1 = progress_along(q_near_1, q_rand, step_size)
        if is_collision_free(q_near_1, q_new_1, step_check_depth, collision_fn):
            T1_T2[T1_idx][0].append(q_new_1)
            T1_T2[T1_idx] = add_edge(T1_T2[T1_idx], [q_near_1, q_new_1])
            draw_line(q_near_1, q_new_1, T_colors[T1_idx], 3)

            q_near_2, dist2T2 = get_closest_neighbor(T1_T2[T2_idx], q_new_1)
            q_new_2 = progress_along(q_near_2, q_new_1, step_size)
            if is_collision_free(q_near_2, q_new_2, step_check_depth, collision_fn):
                T1_T2[T2_idx][0].append(q_new_2)
                T1_T2[T2_idx] = add_edge(T1_T2[T2_idx], [q_near_2, q_new_2])
                draw_line(q_near_2, q_new_2, T_colors[T2_idx], 3)

        iter_count += 1

    path_from_start = [tuple(min_vt_pair[0])]
    path_from_goal = [tuple(min_vt_pair[1])]
    while True:
        if path_from_start[0] == start_conf:
            break
        path_from_start.insert(0, T1_T2[0][1][tuple(path_from_start[0])][0])

    while True:
        if path_from_goal[-1] == goal_conf:
            break
        path_from_goal.append(T1_T2[1][1][tuple(path_from_goal[-1])][0])
    path_conf = path_from_start + path_from_goal

    return path_conf


def birrt_smoothing(start_conf, goal_conf, step_size, collision_fn, N):
    ################################################################
    # TODO your code to implement the birrt algorithm with smoothing
    ################################################################
    path_conf = birrt(start_conf, goal_conf, step_size, collision_fn)
    for idx in range(N):
        if len(path_conf) <= 2:
            break

        left_idx = int(np.random.uniform(low = 0, high = len(path_conf), size=1))
        if left_idx + 1 == len(path_conf):
            continue
        right_idx = int(np.random.uniform(low = left_idx + 1, high = len(path_conf), size=1))

        if is_collision_free(path_conf[left_idx], path_conf[right_idx], 10, collision_fn):
            path_conf_next = path_conf[:left_idx + 1]
            left_vec = np.array(path_conf[left_idx])
            right_vec = np.array(path_conf[right_idx])
            itr_vec = (right_vec - left_vec) / (right_idx - left_idx)
            for itr in range(right_idx - left_idx - 1):
                path_conf_next.append(tuple(left_vec + (itr + 1) * itr_vec))
            path_conf_next += path_conf[right_idx:]
            path_conf = path_conf_next
    return path_conf

def generate_random_point():
    # Three joint limits: [-2pi. 2pi], [-2pi, 2pi] and [-pi. pi]
    q_rand = np.random.uniform(low = -2*np.pi, high = 2*np.pi, size=3)
    q_rand[2] = q_rand[2] / 2.
    return q_rand

def get_closest_neighbor(T, q):
    V = T[0]
    min_dist = np.inf
    for vtx in V:
        cur_dist = np.linalg.norm(np.array(vtx) - np.array(q))
        if cur_dist < min_dist:
            q_near = vtx
            min_dist = cur_dist
    return q_near, min_dist

def is_collision_free(q1, q2, n, collision_fn):
    mid_point = ((np.array(q1) + np.array(q2)) / 2.)
    if n == 0:
        return not collision_fn(mid_point)
    else:
        q1_check = is_collision_free(q1, mid_point, n-1, collision_fn)
        if not q1_check:
            return q1_check
        q2_check = is_collision_free(mid_point, q2, n-1, collision_fn)
        return q1_check and q2_check

def add_edge(T, edge):
    E = T[1]
    v1 = tuple(edge[0])
    v2 = tuple(edge[1])
    if v1 in E:
        E[v1].append(v2)
    else:
        E[v1] = [v2]
    if v2 in E:
        E[v2].append(v1)
    else:
        E[v2] = [v1]
    T[1] = E
    return T

def tree_dist(T1_T2):
    vts_T1 = T1_T2[0][0]
    vts_T2 = T1_T2[1][0]
    min_dist = np.inf
    for v_T1 in vts_T1:
        for v_T2 in vts_T2:
            cur_dist = np.linalg.norm(np.array(v_T1) - np.array(v_T2))
            if cur_dist < min_dist:
                min_vt_pair = (v_T1, v_T2)
                min_dist = cur_dist
    return min_vt_pair, min_dist

def progress_along(q_near, q_rand, step_size):
    move_vec = q_rand - np.array(q_near)
    move_vec = move_vec / np.linalg.norm(move_vec) * step_size
    q_new = (np.array(q_near) + move_vec).tolist()
    return q_new

def draw_line(q_near, q_rand, color, width):
    set_joint_positions(ur5, UR5_JOINT_INDICES, q_near)
    start = p.getLinkState(ur5,3)[0]
    set_joint_positions(ur5, UR5_JOINT_INDICES, q_rand)
    end = p.getLinkState(ur5,3)[0]
    p.addUserDebugLine(start, end, color, width)

def draw_solution_path(path_conf, color, width):
    for i in range(len(path_conf)-1):
        set_joint_positions(ur5, UR5_JOINT_INDICES, path_conf[i])
        start = p.getLinkState(ur5,3)[0]
        set_joint_positions(ur5, UR5_JOINT_INDICES, path_conf[i+1])
        end = p.getLinkState(ur5,3)[0]
        p.addUserDebugLine(start, end, color, width)

if __name__ == "__main__":
    args = get_args()

    # set up simulator
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.setGravity(0, 0, -9.8)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, True)
    p.resetDebugVisualizerCamera(cameraDistance=1.400, cameraYaw=58.000, cameraPitch=-42.200, cameraTargetPosition=(0.0, 0.0, 0.0))

    # load objects
    plane = p.loadURDF("plane.urdf")
    ur5 = p.loadURDF('assets/ur5/ur5.urdf', basePosition=[0, 0, 0.02], useFixedBase=True)
    obstacle1 = p.loadURDF('assets/block.urdf',
                           basePosition=[1/4, 0, 1/2],
                           useFixedBase=True)
    obstacle2 = p.loadURDF('assets/block.urdf',
                           basePosition=[2/4, 0, 2/3],
                           useFixedBase=True)
    obstacles = [plane, obstacle1, obstacle2]

    # start and goal
    start_conf = (-0.813358794499552, -0.37120422397572495, -0.754454729356351)
    start_position = (0.3998897969722748, -0.3993956744670868, 0.6173484325408936)
    goal_conf = (0.7527214782907734, -0.6521867735052328, -0.4949270744967443)
    goal_position = (0.35317009687423706, 0.35294029116630554, 0.7246701717376709)
    goal_marker = draw_sphere_marker(position=goal_position, radius=0.02, color=[1, 0, 0, 1])
    set_joint_positions(ur5, UR5_JOINT_INDICES, start_conf)

    # place holder to save the solution path
    path_conf = None

    # get the collision checking function
    from collision_utils import get_collision_fn
    collision_fn = get_collision_fn(ur5, UR5_JOINT_INDICES, obstacles=obstacles,
                                       attachments=[], self_collisions=True,
                                       disabled_collisions=set())

    if args.birrt:
        if args.smoothing:
            # using birrt with smoothing
            path_conf = birrt_smoothing(start_conf, goal_conf, 0.1, collision_fn, 100)
        else:
            # using birrt without smoothing
            path_conf = birrt(start_conf, goal_conf, 0.1, collision_fn)
    else:
        # using rrt
        path_conf = rrt(start_conf, goal_conf, 0.1, collision_fn)

    if path_conf is None:
        # pause here
        raw_input("no collision-free path is found within the time budget, finish?")
    else:
        ###############################################
        # TODO your code to highlight the solution path
        ###############################################
        draw_solution_path(path_conf, [1,0,0], 10)

        # execute the path
        while True:
            for q in path_conf:
                set_joint_positions(ur5, UR5_JOINT_INDICES, q)
                time.sleep(0.5)
