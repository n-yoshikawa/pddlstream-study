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

    init = [('Conf', 0), ('Arm', 'myarm'), ('ArmEmpty', 'myarm')]

    # Set object positions
    for name, pos in objects.items():
        init += [('Object', name), ('AtConf', name, pos)]

    # Bring water to 0
    goal = ('AtConf', 'water', 0)

    samples = []
    roadmap = []

    # Check whether ethanol is an obstacle
    def test_movable(obj, fluents=[]):
        print("fluents:", fluents)
        water_pos = None
        ethanol_pos = None
        for i, fluent in enumerate(fluents):
            if fluent[1] == 'water':
                water_pos = fluent[2]
            elif fluent[1] == 'ethanol':
                ethanol_pos = fluent[2]
        if water_pos > ethanol_pos: # ethanol is an obstacle
            return False
        else:  # ethanol is not an obstacle
            return True

    stream_map = {
        'find-motion':  from_test(test_movable),
    }

    problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

    return problem, samples, roadmap


##################################################

# TODO: algorithms that take advantage of metric space (RRT)

def main(max_time=20):
    parser = create_parser()
    args = parser.parse_args()
    print('Arguments:', args)

    objects = {
        'water': 1,
        'ethanol': 2,
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
