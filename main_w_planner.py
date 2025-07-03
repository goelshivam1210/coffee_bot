from planning_functions import plan_and_parse
from environment.simulation.env import PickPlaceEnv
from environment.coffee_env import CoffeeEnv
import numpy as np

high_resolution = False
high_frame_rate = False

# Create espresso-cup cappuccino-cup americano-cup
CUPS = ['espresso-cup esp-cup1 loc1 False True False', 'cappuccino-cup cap-cup1 loc2 True True False', 'americano-cup amer-cup1 loc2 True True False']
# Create locations: loc1 loc2 loc3 loc4
LOCATIONS = ['dirty-area loc1', 'clean-area loc2', 'coffee-machine loc3', 'serving-counter loc4']
# Create buttons on coffee machine: esp-btn cap-btn amer-btn
BUTTONS = ['esp-btn', 'cap-btn', 'amer-btn']


baseenv = PickPlaceEnv(render=True, high_res=high_resolution, high_frame_rate=high_frame_rate)
env = CoffeeEnv(baseenv, CUPS, LOCATIONS, "loc2", True)



parsed_plan = plan_and_parse()
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