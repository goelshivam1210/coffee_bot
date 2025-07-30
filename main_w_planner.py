from scripts.planner.planning_functions import plan_and_parse, parse_pddl_objects
from envs.sim.env import PickPlaceEnv
from envs.wrappers.coffee_env import ActionExecutor
from utils.trajectory import Trajectory, save_to_json
import numpy as np
import argparse

parser = argparse.ArgumentParser(prog='CoffeeBot Planner',
                                 description='CoffeeBot Planner for executing PDDL plans in a simulated environment.')
parser.add_argument('-p', '--problem_file', type=str, default="pddl/problem.pddl")
parser.add_argument('-t', '--trajectories', action='store_true',
                    help='Flag to save trajectories to JSON file.')
parser.add_argument('-v', '--video', action='store_true',
                    help='Flag to save a video of the simulation.')
args = parser.parse_args()

high_resolution = True
high_frame_rate = False

# Get objects and robot location from PDDL problem
CUPS, LOCATIONS, BUTTONS, ROBOT_LOCATION = parse_pddl_objects(problem_file=args.problem_file)

# For debugging
print("Generated CUPS:", CUPS)
print("Generated LOCATIONS:", LOCATIONS) 
print("Generated BUTTONS:", BUTTONS)
print("Generated ROBOT_LOCATION:", ROBOT_LOCATION)


env = ActionExecutor(CUPS, LOCATIONS, BUTTONS, ROBOT_LOCATION, hands_free=True, render=True, high_res=high_resolution, high_frame_rate=high_frame_rate)


parsed_plan = plan_and_parse(problem_file=args.problem_file)
print("parsed plan:: ", parsed_plan)

trajectories = []
for action in parsed_plan:
  env.execute(action)
  if args.trajectories:
    # Need to save state (pddl groundings), image, and action at each timestep
    trajectories.append(Trajectory(
        groundings=env.symbolic_state.get_groundings(),
        image=env.sim_actions.base_env.get_camera_image(),
        action=action
    ))

if args.trajectories:
  save_to_json(trajectories)
  print("Trajectories saved to trajectories.json")      

# Save video of simulation
if args.video:
  env.sim_actions.base_env.save_video("my_simulation.mp4")