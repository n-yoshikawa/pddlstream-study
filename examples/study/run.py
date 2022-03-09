from __future__ import print_function

import os
import numpy as np

from pddlstream.algorithms.meta import create_parser
from examples.study.viewer import sample_box, get_distance, is_collision_free, \
    create_box, draw_solution, draw_roadmap, draw_environment
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.algorithms.meta import solve
from pddlstream.language.generator import from_gen_fn, from_test
from pddlstream.utils import read, user_input, str_from_object, INF, Profiler
from pddlstream.language.constants import PDDLProblem, print_solution
from pddlstream.algorithms.constraints import PlanConstraints


ARRAY = np.array # No hashing
#ARRAY = list # No hashing
#ARRAY = tuple # Hashing

def create_problem(objects):
    directory = os.path.dirname(os.path.abspath(__file__))
    domain_pddl = read(os.path.join(directory, 'domain.pddl'))
    stream_pddl = read(os.path.join(directory, 'stream.pddl'))
    constant_map = {}

    init = [('Conf', 0), ('Arm', 'myarm'), ('ArmEmpty', 'myarm'), ('ObjectPose', 0), ('ObjectPose', -1)]

    # Set object positions
    for name, pos in objects.items():
        init += [('Object', name), ('AtConf', name, pos), ('ObjectPose', pos)]

    # Bring water to 0
    goal = ('AtConf', 'water', 0)

    samples = []
    roadmap = []

    def test_movable(obj, s, e, fluents=[]):
        for fluent in fluents:
            if fluent[1] != obj:
                pos = fluent[2]
                if pos < 0:
                    continue
                if (s <= pos <= e) or (s >= pos >= e):
                    return False
        return True

    stream_map = {
        'find-motion':  from_test(test_movable),
    }

    problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

    return problem, samples, roadmap


##################################################

# TODO: algorithms that take advantage of metric space (RRT)

def main(max_time=600):
    parser = create_parser()
    args = parser.parse_args()
    print('Arguments:', args)

    # 3 obstacles case worked, 4 obstacles didn't
    objects = {
        'water': 4,
        #'coffee': 3,
        'wine': 2,
        'vodka': 1,
    }

    problem, samples, roadmap = create_problem(objects)
    constraints = PlanConstraints(max_cost=1.25) # max_cost=INF)

    with Profiler(field='tottime', num=10):
        solution = solve(problem, algorithm="adaptive",
                         constraints=constraints, unit_costs=args.unit, success_cost=0,
                                     max_time=max_time, verbose=False)

    print_solution(solution)

if __name__ == '__main__':
    main()
