import robotic as ry
import numpy as np
from bisect import bisect_left
import time

EGO_NAME = "ego"
OBJ_NAME = "obj"
SUB_GOAL_NAME = "sub-goal1"
GOAL_NAME = "goal"
CAMERA_NAME = "camera_top"
FLOOR_NAME = "floor"

def naive_is_in_line_of_sight(object_frame : ry.Frame, target_frame : ry.Frame, config : ry.Config) -> bool:
    frame_position = target_frame.getPosition()
    object_position = object_frame.getPosition()
    object_size = object_frame.getSize()
    frame_hypotenuse = np.sqrt(object_size[0]**2 + object_size[1]**2)

    sight_ray = (frame_position - object_position)[:2]
    alpha = np.arctan2(sight_ray[1], sight_ray[0])

    offset = frame_hypotenuse / 2 # can be made more precise

    box_length = np.sqrt(sight_ray[0]**2 + sight_ray[1]**2) - 2 * offset
    box_width = 0.05
    box_height = .2

    box_x = object_position[0] + sight_ray[0]/2
    box_y = object_position[1] + sight_ray[1]/2
    box_z = 0.3
    box_yaw = alpha

    copy_config = ry.Config()
    copy_config.addConfigurationCopy(config)
    copy_config.addFrame("line_of_sight") \
        .setShape(ry.ST.box, [box_length, box_width, box_height]) \
        .setContact(1) \
        .setColor([1, 1, 1]) \
        .setPose(f"t({box_x} {box_y} {box_z}) E(0 0 {box_yaw})")
    copy_config.addFrame("point_A") \
        .setShape(ry.ST.marker, [.2]) \
        .setPosition(frame_position)
    copy_config.addFrame("point_B") \
        .setShape(ry.ST.marker, [.2]) \
        .setPosition(object_position)

    is_in_sight = copy_config.getCollisionFree()
    del copy_config
    return is_in_sight


def is_in_line_of_sight(object_frame : ry.Frame, target_frame : ry.Frame, config : ry.Config) -> bool:
    frame_position = target_frame.getPosition()
    object_position = object_frame.getPosition()
    object_size = object_frame.getSize()
    frame_hypotenuse = np.sqrt(object_size[0]**2 + object_size[1]**2)

    sight_ray = (frame_position - object_position)[:2]
    alpha = np.arctan2(sight_ray[1], sight_ray[0])
    theta = abs(alpha)

    offset = frame_hypotenuse / 2 # can be made more precise

    box_length = np.sqrt(sight_ray[0]**2 + sight_ray[1]**2) - 2 * offset
    box_width = frame_hypotenuse * max(
        np.cos(np.pi/4 - theta),
        np.sin(theta - np.pi/4)
    )
    box_height = .2

    box_x = object_position[0] + sight_ray[0]/2
    box_y = object_position[1] + sight_ray[1]/2
    box_z = 0.3
    box_yaw = alpha

    copy_config = ry.Config()
    copy_config.addConfigurationCopy(config)
    copy_config.addFrame("line_of_sight") \
        .setShape(ry.ST.box, [box_length, box_width, box_height]) \
        .setContact(1) \
        .setColor([1, 1, 1]) \
        .setPose(f"t({box_x} {box_y} {box_z}) E(0 0 {box_yaw})")
    copy_config.addFrame("point_A") \
        .setShape(ry.ST.marker, [.2]) \
        .setPosition(frame_position)
    copy_config.addFrame("point_B") \
        .setShape(ry.ST.marker, [.2]) \
        .setPosition(object_position)

    is_in_sight = copy_config.getCollisionFree()
    del copy_config
    return is_in_sight


def cartesian_to_polar(coordinates):
    x = coordinates[0]
    y = coordinates[1]
    r = np.hypot(x, y)
    theta = np.arctan2(y, x)
    return r, theta


def polar_to_cartesian(coordinates):
    r = coordinates[0]
    theta = coordinates[1]
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y


def get_proximity_score(object_frame : ry.Frame, goal_frame : ry.Frame) -> int:
    distance = np.linalg.norm(object_frame.getPosition()[:2] - goal_frame.getPosition()[:2])
    if distance < 0.2:
        return 5
    elif distance < 0.4:
        return 2
    else:
        return 0


def get_goal_score(
        object_frame : ry.Frame,
        agent_frame : ry.Frame,
        goal_frame : ry.Frame,
        config : ry.Config) -> int:
    return 10 * int(naive_is_in_line_of_sight(object_frame, goal_frame, config)) \
        + 5 * int(naive_is_in_line_of_sight(object_frame, agent_frame, config)) \
        + get_proximity_score(object_frame, goal_frame)


def reachable(reach_config : ry.Config, object_frame : ry.Frame) -> bool:
    copy_config = ry.Config()
    copy_config.addConfigurationCopy(reach_config)
    copy_config.getFrame(OBJ_NAME).setContact(0)
    rrt = ry.PathFinder()
    rrt.setProblem(copy_config,
                   [copy_config.getJointState()],
                   [object_frame.getPosition()[:2] - copy_config.getFrame(EGO_NAME).getPosition()[:2]] + copy_config.getJointState())
    ret = rrt.solve()
    del rrt
    del copy_config
    return ret.feasible


def reject(rej_config : ry.Config, solution_tree : 'SolutionTree') -> bool:
    # calculate a similarity value for configurations, based on the discretized positions of movable objects, and reject similar configurations
    SIMILARITY_THRESHOLD = 0.05
    count   = 0
    for node in solution_tree.list1:
        if np.linalg.norm(rej_config.getFrame(OBJ_NAME).getPosition()[:2] - node.get_config().getFrame(OBJ_NAME).getPosition()[:2]) < SIMILARITY_THRESHOLD:
            count += 1
        
    for node in solution_tree.list2:
        if np.linalg.norm(rej_config.getFrame(OBJ_NAME).getPosition()[:2] - node.get_config().getFrame(OBJ_NAME).getPosition()[:2]) < SIMILARITY_THRESHOLD:
            count += 1
    
    for node in solution_tree.list3:
        if np.linalg.norm(rej_config.getFrame(OBJ_NAME).getPosition()[:2] - node.get_config().getFrame(OBJ_NAME).getPosition()[:2]) < SIMILARITY_THRESHOLD:
            count += 1
        
    return count > 1


POINT_COUNT = 200
THRESHOLD = 4
SUBSET_SIZE = 25
def propose_subgoals(config : ry.Config, object_frame : ry.Frame) -> list:
    generated_points = np.random.uniform(low=-2, high=2, size=(POINT_COUNT, 2)) # random point sampling

    filtered_points = []
    for point in generated_points:
        copy_config = ry.Config()
        copy_config.addConfigurationCopy(config)

        obj_pos = copy_config.getFrame(OBJ_NAME).getPosition()
        copy_config.getFrame(OBJ_NAME).setPosition([point[0], point[1], obj_pos[2]])

        if not copy_config.getCollisionFree():
            del copy_config
            continue
        score = get_goal_score(copy_config.getFrame(OBJ_NAME),
                               copy_config.getFrame(EGO_NAME),
                               copy_config.getFrame(GOAL_NAME),
                               copy_config)
        del copy_config
        if score >= THRESHOLD:
            filtered_points.append(point)

    filtered_points.append(object_frame.getPosition()[:2])
    points_array = np.array(filtered_points)
    subset_size = min(len(filtered_points), SUBSET_SIZE)
    point_subset = points_array[np.random.choice(points_array.shape[0], size=subset_size, replace=False)]

    return point_subset


def animate_solution(anim_config, solution):
    for state in solution.x:
        anim_config.setJointState(state)
        anim_config.view()
        time.sleep(0.025)


class SolutionNode:
    def __init__(self, goal_config : ry.Config, score : float = None, parent : 'SolutionNode' = None):
        self.goal_config = goal_config
        self.parent = parent
        self.children = []
        if score is None:
            copy_config = ry.Config()
            copy_config.addConfigurationCopy(goal_config)
            copy_config.getFrame(OBJ_NAME).setPosition(copy_config.getFrame(SUB_GOAL_NAME).getPosition())
            self.score = get_goal_score(copy_config.getFrame(OBJ_NAME),
                                        copy_config.getFrame(EGO_NAME),
                                        copy_config.getFrame(GOAL_NAME),
                                        copy_config)
            del copy_config
        else :
            self.score = score
        self.depth = 0 if parent is None else parent.depth + 1

    def __lt__(self, other):
        return self.score < other.score
    
    def __eq__(self, other):
        return self.score == other.score

    def add_child(self, new_goal : ry.Config, score : float = None) -> 'SolutionNode':
        new_node = SolutionNode(new_goal, score, parent=self)
        self.children.append(new_node)
        return new_node
    
    def get_config(self) -> ry.Config:
        return self.goal_config
    
    def get_path(self) -> list['SolutionNode']:
        path = []
        node = self
        while node is not None:
            path.append(node)
            node = node.parent
        return path[::-1]


class SolutionTree:
    def __init__(self, root : SolutionNode):
        self.root = root
        self.list1 = [root]
        self.list2 = []
        self.list3 = []
        self.mode = 0
        self.tree_depth = 1


    def add_node(self, node : SolutionNode, list_num : int = 1) -> None:
        if list_num == 1:
            index = bisect_left(self.list1, node)
            self.list1.insert(index, node)
        elif list_num == 2:
            index = bisect_left(self.list2, node)
            self.list2.insert(index, node)
        else:
            index = bisect_left(self.list3, node)
            self.list3.insert(index, node)
        self.update_mode()
        self.tree_depth = max(self.tree_depth, node.depth + 1)

    
    def update_mode(self) -> None:
        if len(self.list1) == 0:
            if len(self.list2) == 0:
                self.mode = 2
            else:
                self.mode = 1
        else:
            self.mode = 0


    def get_next_node(self) -> SolutionNode:
        if self.mode == 0:
            if len(self.list1) == 0:
                return None
            else:
                node = self.list1.pop(0)
                self.add_node(node, 2)
                return node
        elif self.mode == 1:
            if len(self.list2) == 0:
                return None
            else:
                node = self.list2.pop(0)
                self.add_node(node, 3)
                return node
        else:
            if len(self.list3) == 0:
                return None
            else:
                return self.list3.pop(0)


    def get_len(self) -> int:
        return len(self.list1) + len(self.list2) + len(self.list3)


    def get_depth(self) -> int:
        return self.tree_depth
    