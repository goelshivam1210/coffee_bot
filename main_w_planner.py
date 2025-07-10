from scripts.planner.planning_functions import plan_and_parse, parse_pddl_objects
from envs.simulation.env import PickPlaceEnv
from envs.wrappers.coffee_env import CoffeeEnv
import numpy as np

high_resolution = True
high_frame_rate = False

problem_file = "pddl/problem_all_clean.pddl"

CUPS, LOCATIONS, BUTTONS, ROBOT_LOCATION = parse_pddl_objects(problem_file=problem_file)

print("Generated CUPS:", CUPS)
print("Generated LOCATIONS:", LOCATIONS) 
print("Generated BUTTONS:", BUTTONS)
print("Generated ROBOT_LOCATION:", ROBOT_LOCATION)


baseenv = PickPlaceEnv(render=True, high_res=high_resolution, high_frame_rate=high_frame_rate)
env = CoffeeEnv(baseenv, CUPS, LOCATIONS, BUTTONS, ROBOT_LOCATION, True)



parsed_plan = plan_and_parse(problem_file=problem_file)
print("parsed plan:: ", parsed_plan)


for individual_operator in parsed_plan:
  action = individual_operator.split(' ')[0]
  if action == 'PICK':
    cup_to_pick = individual_operator.split(' ')[1].lower()
    cup_location = individual_operator.split(' ')[2].lower()
    env.pick(cup_to_pick, cup_location)
  if action == 'MOVE':
    move_from = individual_operator.split(' ')[1].lower()
    move_to = individual_operator.split(' ')[2].lower()
    env.move(move_from, move_to)
  if action == 'PLACE':
    cup_to_place = individual_operator.split(' ')[1].lower()
    cup_location = individual_operator.split(' ')[2].lower()
    env.place(cup_to_place, cup_location)
  if action == 'PRESS-BUTTON-CAPPUCCINO':
    button_name = individual_operator.split(' ')[1].lower()
    cup_name = individual_operator.split(' ')[2].lower()
    cm_location = individual_operator.split(' ')[3].lower()
    env.press_cappuccino_button(button_name, cup_name, cm_location)
  if action == 'PRESS-BUTTON-ESPRESSO':
    button_name = individual_operator.split(' ')[1].lower()
    cup_name = individual_operator.split(' ')[2].lower()
    cm_location = individual_operator.split(' ')[3].lower()
    env.press_espresso_button(button_name, cup_name, cm_location)
  if action == 'PRESS-BUTTON-AMERICANO':
    button_name = individual_operator.split(' ')[1].lower()
    cup_name = individual_operator.split(' ')[2].lower()
    cm_location = individual_operator.split(' ')[3].lower()
    env.press_americano_button(button_name, cup_name, cm_location)
  if action == 'PRESS-CLEANING-BUTTON':
    button_name = individual_operator.split(' ')[1].lower()
    cup_name = individual_operator.split(' ')[2].lower()
    cm_location = individual_operator.split(' ')[3].lower()
    env.press_cleaning_button(button_name, cup_name, cm_location)

# Save video of simulation
baseenv.save_video("my_simulation.mp4")