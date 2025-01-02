import robotic as ry
from utility import *
from bottleneck_analysis import bottleneck_vertices_from_config
from config_to_graph import graph_from_config
import time
import numpy as np
import csv

puzzles = [
    "p1-two-blocks",
    "p2-maze-easy",
    "p3-maze",
    "p4-four-blocks",
    "p5-wall-easy",
    "p6-wall",
    "p7-o-room",
    "p8-corner",
    "p9-cube-free"
]

FPR = "fpr"
BOTTLENECK = "bottleneck"

strategies = [FPR, BOTTLENECK]

EGO_NAME = "ego"
GOAL_OBJ_NAME = "obj"
SUB_GOAL_NAME = "sub-goal1"
GOAL_NAME = "goal"
CAMERA_NAME = "camera_top"
FLOOR_NAME = "floor"
GAP_COEFFICIENT = 1.1

def solve_touch_with_komo(komo_config, obj_name):
    komo = ry.KOMO(komo_config, phases=1, slicesPerPhase=1, kOrder=0, enableCollisions=True)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, scale=[1e2])
    komo.addObjective([], ry.FS.negDistance, [EGO_NAME, obj_name], ry.OT.eq, [1e1])
    ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()
    if ret:
        path = komo.getPath()
        return path
    else:
        return None

ry.params_add({
    "rrt/stepsize": 0.05
})
def solve(cur_config : ry.Config, object_name: str, goal_name : str):
    q_begin = cur_config.getJointState()
    q_ego = cur_config.getFrame(EGO_NAME).getPosition()[:2]
    q_object = cur_config.getFrame(object_name).getPosition()[:2]
    q_target = cur_config.getFrame(goal_name).getPosition()[:2]

    ITERATIONS = 16
    delta_theta = 2 * np.pi / ITERATIONS
    theta_offset = np.random.uniform(0, 2 * np.pi)
    touch_positions = []
    gap = cur_config.getFrame(EGO_NAME).getSize()[1] + max(cur_config.getFrame(object_name).getSize()) * GAP_COEFFICIENT
    for i in range(ITERATIONS):
        transpose_polar1 = np.array([gap, i * delta_theta + theta_offset])
        transpose_cartesian1 = np.array(polar_to_cartesian(transpose_polar1))
        q_home1 = q_object + transpose_cartesian1 - q_ego + q_begin
        cur_config.setJointState(q_home1)

        q_sub = solve_touch_with_komo(cur_config, object_name)
        if q_sub is not None:
            touch_positions.append(q_sub)

    np.random.shuffle(touch_positions)

    for q_sub in touch_positions:
        cur_config.setJointState(q_begin)
        copy_config = ry.Config()
        copy_config.addConfigurationCopy(cur_config)
        copy_config.setJointState(q_sub)
        if copy_config.getCollisionsTotalPenetration() > 0:
            continue

        rrt = ry.PathFinder()
        rrt.setProblem(cur_config, [q_begin], [q_sub])
        solution1 = rrt.solve()
        del rrt
        if not solution1.feasible:
            continue
        
        cur_config.setJointState(q_sub)
        cur_config.attach(EGO_NAME, object_name)
        q_real_goal = q_target + q_sub - q_object - q_ego + q_begin

        rrt = ry.PathFinder()
        rrt.setProblem(cur_config, [q_sub], [q_real_goal])
        solution2 = rrt.solve()
        del rrt
        cur_config.attach(FLOOR_NAME, object_name)
        cur_config.setJointState(q_begin)
        if solution1.feasible and solution2.feasible:
            return solution1, solution2
        
    return None, None

def solve_with_fpr(puzzle_name):
    config = ry.Config()
    config.addFile(f"puzzles/{puzzle_name}.g")
    object_names = []
    for frame_name in config.getFrameNames():
        attributes = config.getFrame(frame_name).getAttributes()
        if "logical" in attributes and "object" in attributes["logical"]:
            object_names.append(frame_name)

    object_names.sort()

    solution_path = None
    start_node = SolutionNode(config, GOAL_OBJ_NAME)
    L = SolutionTree(start_node)
    while L.get_len() > 0:
        cur_node = L.get_next_node()
        x = cur_node.get_config()

        sol1, sol2 = solve(x, GOAL_OBJ_NAME, GOAL_NAME)
        if sol1 is not None:
            solution_path = cur_node.get_path()
            break
        
        for object_name in object_names:
            if not reachable(x, object_name):
                continue
            subgoals = propose_subgoals(x, object_name)
            for subgoal in subgoals:
                new_config = ry.Config()
                new_config.addConfigurationCopy(x)
                new_config.getFrame(SUB_GOAL_NAME).setPosition(subgoal.tolist() + [0.2])
                if reject(new_config, L):
                    del new_config
                    continue

                path1, path2 = solve(new_config, object_name, SUB_GOAL_NAME)
                if path1 is not None:
                    new_config.setJointState(path1.x[-1])
                    new_config.attach(EGO_NAME, object_name)
                    new_config.setJointState(path2.x[-1])
                    new_config.attach(FLOOR_NAME, object_name)
                    new_node = cur_node.add_child(new_config, object_name)
                    L.add_node(new_node)
                else:
                    del new_config

    if solution_path is None:
        return -1
    
    step_count = 0
    for i in range(1, len(solution_path)):
        next_config = solution_path[i].get_config()
        target_obj_name = solution_path[i].target_obj
        config.getFrame(SUB_GOAL_NAME).setPosition(next_config.getFrame(target_obj_name).getPosition())
        solution1, solution2 = None, None
        while solution1 is None:
            solution1, solution2 = solve(config, target_obj_name, SUB_GOAL_NAME)
                
        step_count += len(solution1.x)
        config.setJointState(solution1.x[-1])
        config.attach(EGO_NAME, target_obj_name)

        step_count += len(solution2.x)
        config.setJointState(solution2.x[-1])
        config.attach(FLOOR_NAME, target_obj_name)

    solution1, solution2 = None, None
    while solution1 is None:
        solution1, solution2 = solve(config, GOAL_OBJ_NAME, GOAL_NAME)
            
    step_count += len(solution1.x)
    config.setJointState(solution1.x[-1])
    config.attach(EGO_NAME, target_obj_name)
    
    step_count += len(solution2.x)
    config.setJointState(solution2.x[-1])
    config.attach(FLOOR_NAME, target_obj_name)

    del config

    return step_count

def solve_with_bottleneck(puzzle_name):
    config = ry.Config()
    config.addFile(f"puzzles/{puzzle_name}.g")
    object_names = []
    for frame_name in config.getFrameNames():
        attributes = config.getFrame(frame_name).getAttributes()
        if "logical" in attributes and "object" in attributes["logical"]:
            object_names.append(frame_name)

    object_names.sort()

    solution_path = None
    start_node = SolutionNode(config, GOAL_OBJ_NAME)
    L = SolutionTree(start_node)
    inner_radius = 0.75
    outer_radius = 2.0
    step_size = 0.05
    bottleneck_threshold = 0.25
    graph, obj_vertex = graph_from_config(config, step_size)

    while L.get_len() > 0:
        cur_node = L.get_next_node()
        x = cur_node.get_config()

        sol1, sol2 = solve(x, GOAL_OBJ_NAME, GOAL_NAME)
        if sol1 is not None:
            solution_path = cur_node.get_path()
            break
        
        for object_name in object_names:
            if not reachable(x, object_name):
                continue
            bottleneck_vertices = bottleneck_vertices_from_config(x, inner_radius, outer_radius, step_size, bottleneck_threshold=bottleneck_threshold, graph=graph)
            subgoals = vertex_to_position(bottleneck_vertices, step_size)
            for subgoal in subgoals:
                new_config = ry.Config()
                new_config.addConfigurationCopy(x)
                new_config.getFrame(SUB_GOAL_NAME).setPosition(subgoal.tolist() + [0.2])
                if reject(new_config, L):
                    del new_config
                    continue

                path1, path2 = solve(new_config, object_name, SUB_GOAL_NAME)
                if path1 is not None:
                    new_config.setJointState(path1.x[-1])
                    new_config.attach(EGO_NAME, object_name)
                    new_config.setJointState(path2.x[-1])
                    new_config.attach(FLOOR_NAME, object_name)
                    new_node = cur_node.add_child(new_config, object_name)
                    L.add_node(new_node)
                else:
                    del new_config

    if solution_path is None:
        return -1
    
    step_count = 0
    for i in range(1, len(solution_path)):
        next_config = solution_path[i].get_config()
        target_obj_name = solution_path[i].target_obj
        config.getFrame(SUB_GOAL_NAME).setPosition(next_config.getFrame(target_obj_name).getPosition())
        solution1, solution2 = None, None
        while solution1 is None:
            solution1, solution2 = solve(config, target_obj_name, SUB_GOAL_NAME)
                
        step_count += len(solution1.x)
        config.setJointState(solution1.x[-1])
        config.attach(EGO_NAME, target_obj_name)

        step_count += len(solution2.x)
        config.setJointState(solution2.x[-1])
        config.attach(FLOOR_NAME, target_obj_name)

    solution1, solution2 = None, None
    while solution1 is None:
        solution1, solution2 = solve(config, GOAL_OBJ_NAME, GOAL_NAME)
            
    step_count += len(solution1.x)
    config.setJointState(solution1.x[-1])
    config.attach(EGO_NAME, target_obj_name)
    
    step_count += len(solution2.x)
    config.setJointState(solution2.x[-1])
    config.attach(FLOOR_NAME, target_obj_name)

    del config

    return step_count
    

results = {}

REPEAT_COUNT = 1
for puzzle_name in puzzles:
    results[puzzle_name] = {}
    for strategy in strategies:
        times = []
        steps = []
        for i in range(REPEAT_COUNT):
            if strategy == FPR:
                start_time = time.time()
                number_of_steps = solve_with_fpr(puzzle_name)
                solution_time = (time.time() - start_time) * 1000 # in ms
            elif strategy == BOTTLENECK:
                start_time = time.time()
                number_of_steps = solve_with_bottleneck(puzzle_name)
                solution_time = (time.time() - start_time) * 1000 # in ms

            if number_of_steps == -1:
                print(f"Failed to solve {puzzle_name} with {strategy}")
            else:
                times.append(solution_time)
                steps.append(number_of_steps)
                print(f"Solved {puzzle_name} with {strategy} in {number_of_steps} steps in {solution_time} ms")

        if times and steps:
            avg_time = np.mean(times)
            var_time = np.var(times)
            avg_steps = np.mean(steps)
            var_steps = np.var(steps)
            results[puzzle_name][strategy] = {
                "avg_time": avg_time,
                "var_time": var_time,
                "avg_steps": avg_steps,
                "var_steps": var_steps
            }

with open("/results.csv", "w", newline='') as csvfile:
    fieldnames = ['Puzzle', 'Strategy', 'Average Time (ms)', 'Variance Time (ms)', 'Average Steps', 'Variance Steps']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    writer.writeheader()
    for puzzle_name, strategies in results.items():
        for strategy, metrics in strategies.items():
            writer.writerow({
                'Puzzle': puzzle_name,
                'Strategy': strategy,
                'Average Time (ms)': metrics['avg_time'],
                'Variance Time (ms)': metrics['var_time'],
                'Average Steps': metrics['avg_steps'],
                'Variance Steps': metrics['var_steps']
            })

        
        
        