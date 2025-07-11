from scripts.planner.planning_functions import plan_and_parse, parse_pddl_objects
from envs.sim.env import PickPlaceEnv
from envs.wrappers.coffee_env import ActionExecutor
import numpy as np

high_resolution = True
high_frame_rate = False

problem_file = "pddl/problem_all_clean.pddl"

CUPS, LOCATIONS, BUTTONS, ROBOT_LOCATION = parse_pddl_objects(problem_file=problem_file)

print("Generated CUPS:", CUPS)
print("Generated LOCATIONS:", LOCATIONS) 
print("Generated BUTTONS:", BUTTONS)
print("Generated ROBOT_LOCATION:", ROBOT_LOCATION)


env = ActionExecutor(CUPS, LOCATIONS, BUTTONS, ROBOT_LOCATION, hands_free=True, render=True, high_res=high_resolution, high_frame_rate=high_frame_rate)



parsed_plan = plan_and_parse(problem_file=problem_file)
print("parsed plan:: ", parsed_plan)


for individual_operator in parsed_plan:
  env.execute(individual_operator)

# Save video of simulation
baseenv.save_video("my_simulation.mp4")