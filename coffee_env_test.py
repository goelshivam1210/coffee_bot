from coffee_env import CoffeeEnv
from env import PickPlaceEnv
import numpy as np

high_resolution = False
high_frame_rate = False
# Create espresso-cup cappuccino-cup americano-cup
CUPS = ['espresso-cup esp-cup1 loc1 False True False', 'cappuccino-cup cap-cup1 loc2 True True False', 'americano-cup amer-cup1 loc2 True True False']
# Create locations: loc1 loc2 loc3 loc4
LOCATIONS = ['loc1', 'loc2', 'loc3', 'loc4']

env = PickPlaceEnv(render=True, high_res=high_resolution, high_frame_rate=high_frame_rate)
coffee_env = CoffeeEnv(env, CUPS, LOCATIONS, "loc2", True)

coffee_env.move("loc2", "loc1")
coffee_env.pick("esp-cup1", "loc1")
coffee_env.move("loc1", "loc3")
coffee_env.place("esp-cup1", "loc3")