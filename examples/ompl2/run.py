from __future__ import print_function

import math
import os
import sys
import numpy as np

from pddlstream.algorithms.meta import create_parser
from pddlstream.algorithms.meta import solve
from pddlstream.language.generator import from_gen_fn, from_test
from pddlstream.utils import read, user_input, str_from_object, INF, Profiler
from pddlstream.language.constants import PDDLProblem, print_solution
from pddlstream.algorithms.constraints import PlanConstraints

import pybullet as p
import pybullet_data

current_directory = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_directory)
import pb_ompl
import utils

ARRAY = np.array # No hashing
#ARRAY = list # No hashing
#ARRAY = tuple # Hashing

class OMPLDemo():
    def __init__(self):
        self.obstacles = []

        p.connect(p.GUI)

        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(1./240.)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")

        # load robotCSC2521HS
        robot_id = p.loadURDF(os.path.join(current_directory,
            "models/franka_description/robots/panda_arm.urdf"), (0,0,0), useFixedBase = 1)
        robot = pb_ompl.PbOMPLRobot(robot_id)
        self.robot = robot

        # Calculate limits
        self.ll = []
        self.ul = []
        self.jr = []
        self.robot.get_joint_bounds()
        for bound in self.robot.joint_bounds:
            self.ll.append(bound[0])
            self.ul.append(bound[1])
            self.jr.append(bound[1]-bound[0])

        # setup pb_ompl
        self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles)
        self.pb_ompl_interface.set_planner("PRM")

        # add obstacles
        self.add_obstacles()

    def clear_obstacles(self):
        for obstacle in self.obstacles:
            p.removeBody(obstacle)

    def add_obstacles(self):
        pass
        # add box
        #self.add_box([1, 0, 0.7], [0.5, 0.5, 0.05])

        # store obstacles
        #self.pb_ompl_interface.set_obstacles(self.obstacles)

    def add_box(self, box_pos, half_box_size):
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        self.obstacles.append(box_id)
        return box_id

    def inverse_kinematics(self, pos, orientation, hand_link=7):
        state = p.calculateInverseKinematics(self.robot.id, hand_link, pos, orientation, lowerLimits=self.ll, upperLimits=self.ul, jointRanges=self.jr)
        return list(state)

    def calc_path(self, start, goal):
        pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles)
        pb_ompl_interface.set_planner("BITstar")

        #print("robot state:", self.robot.state)
        #for i in self.robot.joint_idx:
        #    print(f"getJointState({i}):", p.getJointState(self.robot.id, i))

        if start is None:
            start = self.robot.get_cur_state()
        res, path = pb_ompl_interface.plan_start_goal(start, goal)
        #if res:
        #    pb_ompl_interface.execute(path)
        return res, path

    def run_path(self, path):
        pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles)
        pb_ompl_interface.execute(path)

def create_problem(objects, env):
    directory = os.path.dirname(os.path.abspath(__file__))
    domain_pddl = read(os.path.join(directory, 'domain.pddl'))
    stream_pddl = read(os.path.join(directory, 'stream.pddl'))
    constant_map = {}
    #goal_pos = [0.6, -1.5, 0.3]

    init = [('Conf', (0.6, -1.5, 0.3)), ('Arm', 'myarm'), ('ArmEmpty', 'myarm'),
            ('ObjectPose', (0.6, 1.0, 0.3)), ('ObjectPose', (0.6, -1.0, 0.3)), ('ArmPos', 'myarm', (0.1, 0.0, 0.9))]

    # Set object positions
    for name, pos in objects.items():
        init += [('Object', name), ('AtConf', name, pos), ('ObjectPose', pos)]
        env.add_box(pos, [0.05, 0.05, 0.3])
        #env.pb_ompl_interface.set_obstacles(env.obstacles)

    
    start = [0, 0, 0, -1, 0, 1.0 ,0]  # should be replaced by current pose
    initial_pos = env.inverse_kinematics((0.1, 0.0, 0.9), (1.0, 0.0, 0.0, 0))
    res, path = env.calc_path(start, initial_pos)
    env.run_path(path)

    # Bring water to 0
    #goal = ('ArmPos', 'myarm', (0.6, 1.0, 0.3))
    goal = ('Picked', 'water')

    samples = []
    roadmap = []

    def test_movable(obj, s, e, fluents=[]):
        for fluent in fluents:
            print(fluent)
            obj_pos = None
            if fluent[1] == obj:
                obj_pos = fluent[2]
            else:
                pos = fluent[2]
                #env.add_obstacles()  # dummy
        start = [0, 0, 0, -1, 0, 1.0 ,0]  # should be replaced by current pose
        goal_orn = p.getQuaternionFromEuler([math.pi, 0, 0])
        goal = env.inverse_kinematics(e, goal_orn)
        res, path = env.calc_path(start, goal)
        if path is not None:
            yield (path,)
        else:
            return

    stream_map = {
        'find-motion':  from_gen_fn(test_movable),
    }

    problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

    return problem, samples, roadmap


##################################################

# TODO: algorithms that take advantage of metric space (RRT)

def main(max_time=600):
    parser = create_parser()
    args = parser.parse_args()
    print('Arguments:', args)

    env = OMPLDemo()

    # No obstacle
    objects = {
        'water': (0.6, -1.0, 0.3),
        'vodka': (0.5, 0.5, 0.1),
    }

    problem, samples, roadmap = create_problem(objects, env)
    constraints = PlanConstraints(max_cost=1.25) # max_cost=INF)

    with Profiler(field='tottime', num=10):
        solution = solve(problem, algorithm="adaptive",
                         constraints=constraints, unit_costs=args.unit, success_cost=0,
                                     max_time=max_time, verbose=False)

    for i, action in enumerate(solution.plan):
        print(i, action.name, action.args[:-1])

    input('Run?')
    for action in solution.plan:
            path = action.args[4]
            env.run_path(path)
    input()


if __name__ == '__main__':
    main()
