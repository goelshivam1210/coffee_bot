import os
import pybullet
import pybullet_data
import cv2
import numpy as np
from utils.transform_utils import correct_quaternion_ignore_roll
from envs.sim.gripper import Robotiq2F85

COLORS = {
    'red':    (255/255,  87/255,  89/255, 255/255),
    'blue1':  (65/255,  110/255, 180/255, 255/255),
    'purple1': (180/255, 115/255, 155/255, 255/255),
    'yellow1': (240/255, 190/255,  65/255, 255/255),
    'green1':  (89/255,  169/255,  79/255, 255/255),
    'blue2':   (78/255,  121/255, 167/255, 255/255),
    'yellow2': (237/255, 201/255,  72/255, 255/255),
    'purple2': (176/255, 122/255, 161/255, 255/255),
    'green2':  (80/255,  180/255,  70/255, 255/255),
    # Coffee-specific colors
    'brown':  (156/255, 117/255,  95/255, 255/255),
    'white':  (0.9, 0.9, 0.9, 1.0),
    'black':  (0.1, 0.1, 0.1, 1.0),
    'silver': (0.7, 0.7, 0.7, 1.0),
    'coffee_machine': (0.3, 0.3, 0.3, 1.0),
}

# Coffee-specific object configurations
CUP_CONFIGS = {
    'espresso-cup': {'radius': 0.025, 'height': 0.06, 'mass': 0.05, 'color': 'brown'},
    'cappuccino-cup': {'radius': 0.02, 'height': 0.05, 'mass': 0.04, 'color': 'white'},
    'americano-cup': {'radius': 0.03, 'height': 0.08, 'mass': 0.06, 'color': 'black'},
}

BUTTON_CONFIGS = {
    'esp-btn': {'type': 'espresso-button', 'color': 'brown'},
    'cap-btn': {'type': 'cappuccino-button', 'color': 'white'},
    'amer-btn': {'type': 'americano-button', 'color': 'black'},
}

LOCATION_CONFIGS = {
    'loc3': {'type': 'coffee-machine', 'position': (0, -0.7, 0), 'color': 'coffee_machine'},
    'loc1': {'type': 'dirty-area', 'position': (-0.2, -0.35, 0), 'color': 'red'},      # Left third
    'loc2': {'type': 'clean-area', 'position': (0, -0.35, 0), 'color': 'green1'},      # Middle third  
    'loc4': {'type': 'serving-counter', 'position': (0.2, -0.35, 0), 'color': 'silver'}, # Right third
}

CORNER_POS = {
  'top left corner':     (-0.3 + 0.05, -0.2 - 0.05, 0),
  'top side':            (0,           -0.2 - 0.05, 0),
  'top right corner':    (0.3 - 0.05,  -0.2 - 0.05, 0),
  'left side':           (-0.3 + 0.05, -0.5,        0),
  'middle':              (0,           -0.5,        0),
  'right side':          (0.3 - 0.05,  -0.5,        0),
  'bottom left corner':  (-0.3 + 0.05, -0.8 + 0.05, 0),
  'bottom side':         (0,           -0.8 + 0.05, 0),
  'bottom right corner': (0.3 - 0.05,  -0.8 + 0.05, 0),
}

PIXEL_SIZE = 0.00267857
BOUNDS = np.float32([[-0.3, 0.3], [-0.8, -0.2], [0, 0.15]])  # X Y Z


# Gym-style environment code

class PickPlaceEnv():

  def __init__(self, render=False, high_res=False, high_frame_rate=False, max_steps=5000):
    self.render = render
    self.dt = 1/480
    self.sim_step = 0

    # Configure and start PyBullet.
    # python3 -m pybullet_utils.runServer
    # pybullet.connect(pybullet.SHARED_MEMORY)  # pybullet.GUI for local GUI.
    # pybullet.connect(pybullet.DIRECT)  # pybullet.GUI for local GUI.
    if self.render:
      pybullet.connect(pybullet.GUI)
    else:
      pybullet.connect(pybullet.DIRECT)  # pybullet.GUI for local GUI.
    
    # pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
    pybullet.setPhysicsEngineParameter(enableFileCaching=0)
    assets_path = os.path.dirname(os.path.abspath(""))
    pybullet.setAdditionalSearchPath(assets_path)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    pybullet.setTimeStep(self.dt)

    self.home_joints = (np.pi / 2, -np.pi / 2, np.pi / 2, -np.pi / 2, 3 * np.pi / 2, 0)  # Joint angles: (J0, J1, J2, J3, J4, J5).
    self.home_ee_euler = (np.pi, 0, np.pi)  # (RX, RY, RZ) rotation in Euler angles.
    self.ee_link_id = 9  # Link ID of UR5 end effector.
    self.tip_link_id = 10  # Link ID of gripper finger tips.
    self.gripper = None

    
    self.high_res = high_res
    self.high_frame_rate = high_frame_rate

    self.max_steps = max_steps # max number of steps for one action

  def reset(self, object_list):
    pybullet.resetSimulation(pybullet.RESET_USE_DEFORMABLE_WORLD)
    pybullet.setGravity(0, 0, -9.8)
    self.cache_video = []

    # Temporarily disable rendering to load URDFs faster.
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)

    # Add robot.
    pybullet.loadURDF("plane.urdf", [0, 0, -0.001])
    self.robot_id = pybullet.loadURDF("envs/simulation/ur5e/ur5e.urdf", [0, 0, 0], flags=pybullet.URDF_USE_MATERIAL_COLORS_FROM_MTL)
    self.ghost_id = pybullet.loadURDF("envs/simulation/ur5e/ur5e.urdf", [0, 0, -10])  # For forward kinematics.
    self.joint_ids = [pybullet.getJointInfo(self.robot_id, i) for i in range(pybullet.getNumJoints(self.robot_id))]
    self.joint_ids = [j[0] for j in self.joint_ids if j[2] == pybullet.JOINT_REVOLUTE]

    # Move robot to home configuration.
    for i in range(len(self.joint_ids)):
      pybullet.resetJointState(self.robot_id, self.joint_ids[i], self.home_joints[i])

    # Add gripper.
    if self.gripper is not None:
      while self.gripper.constraints_thread.is_alive():
        self.constraints_thread_active = False
    self.gripper = Robotiq2F85(self.robot_id, self.ee_link_id)
    self.gripper.release()

    # Add workspace.
    plane_shape = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents=[0.3, 0.3, 0.001])
    plane_visual = pybullet.createVisualShape(pybullet.GEOM_BOX, halfExtents=[0.3, 0.3, 0.001])
    plane_id = pybullet.createMultiBody(0, plane_shape, plane_visual, basePosition=[0, -0.5, 0])
    pybullet.changeVisualShape(plane_id, -1, rgbaColor=[0.2, 0.2, 0.2, 1.0])

    # Load objects according to config.
    self.object_list = object_list
    self.obj_name_to_id = {}
    self.location_ids = {}
    
    # First, create location areas
    obj_xyz = np.zeros((0, 3))  # To track object positions and avoid overlaps.
    for obj_name in object_list:
      if obj_name in LOCATION_CONFIGS:
        loc_config = LOCATION_CONFIGS[obj_name]
        position = loc_config['position']

        # Create location area as a visual marker
        if loc_config['type'] == 'coffee-machine':
          # Coffee machine - larger box with buttons
          area_shape = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents=[0.08, 0.08, 0.05])
          area_visual = pybullet.createVisualShape(pybullet.GEOM_BOX, halfExtents=[0.08, 0.08, 0.05])
          area_id = pybullet.createMultiBody(0, area_shape, area_visual, basePosition=[position[0], position[1], 0.05])
          pybullet.changeVisualShape(area_id, -1, rgbaColor=COLORS[loc_config['color']])
          obj_xyz = np.concatenate((obj_xyz, np.array(position).reshape(1, 3)), axis=0)
          # Add buttons to coffee machine
          button_offset = 0.075  # Offset from coffee machine center
          for button_name in ['esp-btn', 'cap-btn', 'amer-btn']:
            if button_name in object_list:
              btn_config = BUTTON_CONFIGS[button_name]
              btn_x = position[0] + (list(BUTTON_CONFIGS.keys()).index(button_name) - 1) * 0.03
              btn_y = position[1] + button_offset
              btn_z = 0.105
              
              btn_shape = pybullet.createCollisionShape(pybullet.GEOM_CYLINDER, radius=0.01, height=0.005)
              btn_visual = pybullet.createVisualShape(pybullet.GEOM_CYLINDER, radius=0.01, length=0.005)
              btn_id = pybullet.createMultiBody(0, btn_shape, btn_visual, basePosition=[btn_x, btn_y, btn_z])
              pybullet.changeVisualShape(btn_id, -1, rgbaColor=COLORS[btn_config['color']])
              self.obj_name_to_id[button_name] = btn_id
              obj_xyz = np.concatenate((obj_xyz, np.array([btn_x, btn_y, btn_z]).reshape(1, 3)), axis=0)
        else:
          # Regular location areas - flat rectangles
          area_shape = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents=[0.09, 0.12, 0.001])
          area_visual = pybullet.createVisualShape(pybullet.GEOM_BOX, halfExtents=[0.09, 0.12, 0.001])
          area_id = pybullet.createMultiBody(0, area_shape, area_visual, basePosition=[position[0], position[1], 0.001])
          pybullet.changeVisualShape(area_id, -1, rgbaColor=COLORS[loc_config['color']])
          if loc_config['type'] == 'dirty-area':
            # Create cleaning button in corner of dirty area
            if 'clean-btn' in object_list:
              btn_shape = pybullet.createCollisionShape(pybullet.GEOM_CYLINDER, radius=0.01, height=0.005)
              btn_visual = pybullet.createVisualShape(pybullet.GEOM_CYLINDER, radius=0.01, length=0.005)
              btn_id = pybullet.createMultiBody(0, btn_shape, btn_visual, 
                                               basePosition=[position[0], position[1] - 0.1, 0.005])
              pybullet.changeVisualShape(btn_id, -1, rgbaColor=COLORS['white'])
              self.obj_name_to_id['clean-btn'] = btn_id
              obj_xyz = np.concatenate((obj_xyz, np.array([position[0], position[1] + 0.05, 0.005]).reshape(1, 3)), axis=0)
        
        self.location_ids[obj_name] = area_id
        self.obj_name_to_id[obj_name] = area_id
    
    # Then create cups at appropriate locations  
    
    cup_list = []
    for obj in object_list:
      # format for cups: "CUP_TYPE CUP_NAME CUP_LOC"
      obj_details = obj.split(' ')
      if obj_details[0] in CUP_CONFIGS:
        cup_config = CUP_CONFIGS[obj_details[0]]
        cup_name = obj_details[1]
        cup_location = obj_details[2]


        base_pos = LOCATION_CONFIGS[cup_location]['position']
        

        while True:
          rand_x = base_pos[0] + np.random.uniform(-0.08, 0.08)  # Within location width
          rand_y = base_pos[1] + np.random.uniform(-0.10, 0.10)  # Within location depth
          rand_x = np.clip(rand_x, BOUNDS[0][0] + cup_config['radius'], BOUNDS[0][1] - cup_config['radius'])
          rand_y = np.clip(rand_y, BOUNDS[1][0] + cup_config['radius'], BOUNDS[1][1] - cup_config['radius'])
          # Ensure the cup
          rand_xyz = np.float32([rand_x, rand_y, 0.03]).reshape(1, 3)
          if len(obj_xyz) == 0:
            obj_xyz = np.concatenate((obj_xyz, rand_xyz), axis=0)
            break
          else:
            nn_dist = np.min(np.linalg.norm(obj_xyz - rand_xyz, axis=1)).squeeze()
            if nn_dist > 0.1:
              obj_xyz = np.concatenate((obj_xyz, rand_xyz), axis=0)
              break
        
        rand_z = cup_config['height'] / 2  # Place on surface
        object_position = [rand_x, rand_y, rand_z]
        
        # Create cup geometry
        object_shape = pybullet.createCollisionShape(pybullet.GEOM_CYLINDER, 
                                                   radius=cup_config['radius'], 
                                                   height=cup_config['height'])
        object_visual = pybullet.createVisualShape(pybullet.GEOM_CYLINDER, 
                                                 radius=cup_config['radius'], 
                                                 length=cup_config['height'])
        object_id = pybullet.createMultiBody(cup_config['mass'], object_shape, object_visual, 
                                           basePosition=object_position)
        pybullet.changeVisualShape(object_id, -1, rgbaColor=COLORS[cup_config['color']])
        cup_list.append(cup_name)
        self.obj_name_to_id[cup_name] = object_id
        obj_xyz = np.concatenate((obj_xyz, np.array(object_position).reshape(1, 3)), axis=0)
    
    # remove cups from object_list and add just the cup names
    object_list = [obj for obj in object_list if "cup" not in obj]
    object_list += cup_list
    self.object_list = object_list
    # Re-enable rendering.
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)

    for _ in range(200):
      pybullet.stepSimulation()

    # record object positions at reset
    self.init_pos = {name: self.get_obj_pos(name) for name in object_list}

    return self.get_observation()

  def servoj(self, joints):
    """Move to target joint positions with position control."""
    pybullet.setJointMotorControlArray(
      bodyIndex=self.robot_id,
      jointIndices=self.joint_ids,
      controlMode=pybullet.POSITION_CONTROL,
      targetPositions=joints,
      positionGains=[0.01]*6)
  
  def movep(self, position):
    """Move to target end effector position."""
    joints = pybullet.calculateInverseKinematics(
        bodyUniqueId=self.robot_id,
        endEffectorLinkIndex=self.tip_link_id,
        targetPosition=position,
        targetOrientation=pybullet.getQuaternionFromEuler(self.home_ee_euler),
        maxNumIterations=100)
    self.servoj(joints)

  def move_with_ee(self, position, orientation):
    joints = pybullet.calculateInverseKinematics(
        bodyUniqueId=self.robot_id,
        endEffectorLinkIndex=self.tip_link_id,
        targetPosition=position,
        targetOrientation=orientation,
        maxNumIterations=100)
    self.servoj(joints)

  def get_ee_pos(self):
    ee_xyz = np.float32(pybullet.getLinkState(self.robot_id, self.tip_link_id)[0])
    return ee_xyz

  def pick(self, obj_to_pick):
    """Do pick and place motion primitive."""

    self.gripper.release()
    steps = 0

    if not self.locate(obj_to_pick):
      print(f"cannot pick as {obj_to_pick} cannot be located")
      return False
    if not self.clear(obj_to_pick) :
      print(f"cannot pick as {obj_to_pick} as its not clear")
      return False

    pick_pos = self.get_obj_pos(obj_to_pick).copy()
    obj_orn = self.get_obj_orn(obj_to_pick).copy()
    # we only care about the z , since the cube might be rotated
    obj_orn = correct_quaternion_ignore_roll(obj_orn)

    # add home_ee_euler to the orientation

    home_quat = pybullet.getQuaternionFromEuler(self.home_ee_euler)
    pick_orn = pybullet.multiplyTransforms([0, 0, 0], home_quat, [0, 0, 0], obj_orn)[1]


    pick_z = pick_pos[2] + 0.005
    # Set fixed primitive z-heights.
    hover_xyz = np.float32([pick_pos[0], pick_pos[1], 0.2])
    if pick_pos.shape[-1] == 2:
      pick_xyz = np.append(pick_pos, pick_z)
    else:
      pick_xyz = pick_pos
      pick_xyz[2] = pick_z

    # Move to object.
    ee_xyz = self.get_ee_pos()
    while np.linalg.norm(hover_xyz - ee_xyz) > 0.01:
      # self.movep(hover_xyz)
      self.move_with_ee(hover_xyz, pick_orn)
      self.step_sim_and_render()
      ee_xyz = self.get_ee_pos()
      steps += 1
      if steps > self.max_steps:
        print("max steps reached while moving to hover position")
        return False

    while np.linalg.norm(pick_xyz - ee_xyz) > 0.01:
      # self.movep(pick_xyz)
      self.move_with_ee(pick_xyz, pick_orn)
      self.step_sim_and_render()
      ee_xyz = self.get_ee_pos()
      steps += 1
      if steps > self.max_steps:
        print("max steps reached while moving to pick position")
        return False

    # Pick up object.
    self.gripper.activate()
    for _ in range(240):
      self.step_sim_and_render()
    while np.linalg.norm(hover_xyz - ee_xyz) > 0.01:
      self.movep(hover_xyz)
      self.step_sim_and_render()
      ee_xyz = self.get_ee_pos()
      steps += 1
      if steps > self.max_steps:
        print("max steps reached while moving to hover position after pick")
        # release the object
        self.gripper.release()
        return False
    
    for _ in range(50):
      self.step_sim_and_render()

    if self.hand_empty():
      return False
    

    if obj_to_pick == 'blue block' and not self.hand_empty():
      # check the object that is closest to the gripper
      obj_pos = []
      for objs in self.object_list:
        obj_pos.append(self.get_obj_pos(objs).copy())
      obj_pos = np.array(obj_pos)
      ee_pos = self.get_ee_pos()
      dist = np.linalg.norm(obj_pos - ee_pos, axis=1)
      closest_obj = self.object_list[np.argmin(dist)]
      # print(f"closest object to gripper is {closest_obj}")
      if closest_obj == obj_to_pick:
        print("picked up the correct object!")


    return True

  def place(self, which_object, obj_to_place, find_empty_pos=False):
    """Do place motion primitive."""

    if 'table' in obj_to_place:
      self.putdown()
      return True

    if not self.locate(obj_to_place):
      print(f"cannot place as {obj_to_place} cannot be located")
      return False
    # if not self.clear(obj_to_place) :
    #   print(f"cannot place as {obj_to_place} as its not clear")
    #   return False
        
    place_pos = self.get_obj_pos(obj_to_place).copy()

    if place_pos.shape[-1] == 2:
      place_xyz = np.append(place_pos, 0.15)
    else:
      place_xyz = place_pos
      place_xyz[2] = 0.15

    # find empty position on top of the obj_to_place area if find_empty_pos is True
    if find_empty_pos:
      obj_pos = []
      for objs in self.object_list:
        if objs == obj_to_place:
          continue
        obj_pos.append(self.get_obj_pos(objs).copy())
      
      # Check if initial place_xyz is already good
      total_objects_far = 0
      for pos in obj_pos:
        if np.linalg.norm(place_xyz[:2] - pos[:2]) > 0.07:
          total_objects_far += 1
      
      # Only search for new position if initial one is not suitable
      if total_objects_far != len(obj_pos):
        num_choose_times = 0
        while True:
          random_empty_pos_candidate = [np.random.uniform(LOCATION_CONFIGS[obj_to_place]["position"][0] - .08, 
                                                          LOCATION_CONFIGS[obj_to_place]["position"][0] + .08), 
                                        np.random.uniform(LOCATION_CONFIGS[obj_to_place]["position"][1] - .10, 
                                                          LOCATION_CONFIGS[obj_to_place]["position"][1] + .10), 
                                        0.15]
          total_objects_far = 0
          for pos in obj_pos:
            if np.linalg.norm(random_empty_pos_candidate[:2] - pos[:2]) > 0.07:
              total_objects_far += 1
          if total_objects_far == len(obj_pos):
            place_xyz = np.array(random_empty_pos_candidate)
            break 
          num_choose_times += 1
          if num_choose_times > 300:
            print("cannot find empty position to place object")
            return False
    

    ee_xyz = self.get_ee_pos()
    # Move to place location.
    while np.linalg.norm(place_xyz - ee_xyz) > 0.01:
      self.movep(place_xyz)
      self.step_sim_and_render()
      ee_xyz = self.get_ee_pos()

    # Place down object.
    while (not self.gripper.detect_contact()) and (place_xyz[2] > 0.03):
      place_xyz[2] -= 0.001
      self.movep(place_xyz)
      for _ in range(3):
        self.step_sim_and_render()
    self.gripper.release()
    for _ in range(240):
      self.step_sim_and_render()
    place_xyz[2] = 0.2
    ee_xyz = self.get_ee_pos()
    while np.linalg.norm(place_xyz - ee_xyz) > 0.01:
      self.movep(place_xyz)
      self.step_sim_and_render()
      ee_xyz = self.get_ee_pos()
    place_xyz = np.float32([0, -0.5, 0.2])
    while np.linalg.norm(place_xyz - ee_xyz) > 0.01:
      self.movep(place_xyz)
      self.step_sim_and_render()
      ee_xyz = self.get_ee_pos()

    if not self.hand_empty():
      return False
    return True
    # observation = self.get_observation()
    # reward = self.get_reward()
    # done = False
    # info = {}
    # return observation, reward, done, info

  def locate(self, obj_to_locate):
    if obj_to_locate in self.object_list:
      return True
    else:
      return False

  def putdown(self, obj_to_place = None):
    steps = 0
    
    obj_pos = []
    for objs in self.object_list:
      obj_pos.append(self.get_obj_pos(objs).copy())

    num_choose_times = 0
    while True:
      # random_empty_pos_candidate = [np.random.uniform(-0.28, 0.28), np.random.uniform(-0.75, -0.25), 0.15]
      random_empty_pos_candidate = [np.random.uniform(-0.28, 0.28), np.random.uniform(-0.75, -0.25), 0.15]
      total_objects_far = 0
      for pos in obj_pos:
        # if abs(random_empty_pos_candidate[0] - pos[0]) > 0.07 and abs(random_empty_pos_candidate[1] - pos[1]) > 0.07:
        if np.linalg.norm(random_empty_pos_candidate[:2] - pos[:2]) > 0.07:
          total_objects_far += 1
      if total_objects_far == len(obj_pos):
        empty_position = random_empty_pos_candidate.copy()
        break 
      num_choose_times += 1
      if num_choose_times > 300:
        print("cannot find empty position to place object")
        self.gripper.release()
        observation = self.get_observation()
        reward = self.get_reward()
        done = False
        info = {}
        return observation, reward, done, info
    place_xyz = empty_position
    place_xyz[2] = 0.15
    ee_xyz = self.get_ee_pos()
    # Move to place location.
    while np.linalg.norm(place_xyz - ee_xyz) > 0.01:
      self.movep(place_xyz)
      self.step_sim_and_render()
      ee_xyz = self.get_ee_pos()
      steps += 1
      if steps > self.max_steps:
        print("max steps reached while moving to place position")
        self.gripper.release()
        observation = self.get_observation()
        reward = self.get_reward()
        done = False
        info = {}
        return observation, reward, done, info

    # Place down object.
    while (not self.gripper.detect_contact()) and (place_xyz[2] > 0.03):
      place_xyz[2] -= 0.001
      self.movep(place_xyz)
      steps += 1
      if steps > self.max_steps:
        print("max steps reached while placing object")
        self.gripper.release()
        observation = self.get_observation()
        reward = self.get_reward()
        done = False
        info = {}
        return observation, reward, done, info
      for _ in range(3):
        self.step_sim_and_render()
    self.gripper.release()
    for _ in range(240):
      self.step_sim_and_render()
    place_xyz[2] = 0.2
    ee_xyz = self.get_ee_pos()
    while np.linalg.norm(place_xyz - ee_xyz) > 0.01:
      self.movep(place_xyz)
      self.step_sim_and_render()
      ee_xyz = self.get_ee_pos()
      steps += 1
      if steps > self.max_steps:
        print("max steps reached while moving to hover position after place")
        observation = self.get_observation()
        reward = self.get_reward()
        done = False
        info = {}
        return observation, reward, done, info

    place_xyz = np.float32([0, -0.5, 0.2])
    while np.linalg.norm(place_xyz - ee_xyz) > 0.01:
      self.movep(place_xyz)
      self.step_sim_and_render()
      steps += 1
      if steps > self.max_steps:
        print("max steps reached while moving to default position after place")
        observation = self.get_observation()
        reward = self.get_reward()
        done = False
        info = {}
        return observation, reward, done, info

    observation = self.get_observation()
    reward = self.get_reward()
    done = False
    info = {}
    return observation, reward, done, info

    
        

  def set_alpha_transparency(self, alpha: float) -> None:
    for id in range(20):
      visual_shape_data = pybullet.getVisualShapeData(id)
      for i in range(len(visual_shape_data)):
        object_id, link_index, _, _, _, _, _, rgba_color = visual_shape_data[i]
        rgba_color = list(rgba_color[0:3]) +  [alpha]
        pybullet.changeVisualShape(
            self.robot_id, linkIndex=i, rgbaColor=rgba_color)      
        pybullet.changeVisualShape(
            self.gripper.body, linkIndex=i, rgbaColor=rgba_color)

  def step_sim_and_render(self):
    pybullet.stepSimulation()
    self.sim_step += 1
    #TODO: add way to pause simulation
    interval = 40 if self.high_frame_rate else 60
    # Render current image at 8 FPS.
    if self.sim_step % interval == 0 and self.render:
      self.cache_video.append(self.get_camera_image())

  def get_camera_image(self):
    if not self.high_res:
      image_size = (240, 240)
      intrinsics = (120., 0, 120., 0, 120., 120., 0, 0, 1)
      color, _, _, _, _ = self.render_image(image_size, intrinsics)
    else:
      color, _, _, _, _ = self.render_image()
    
    return color

  def get_reward(self):
    return None

  def get_observation(self):
    observation = {}


    if self.render:
      # Render current image.
      color, depth, position, orientation, intrinsics = self.render_image()

      # Get heightmaps and colormaps.
      points = self.get_pointcloud(depth, intrinsics)
      position = np.float32(position).reshape(3, 1)
      rotation = pybullet.getMatrixFromQuaternion(orientation)
      rotation = np.float32(rotation).reshape(3, 3)
      transform = np.eye(4)
      transform[:3, :] = np.hstack((rotation, position))
      points = self.transform_pointcloud(points, transform)
      heightmap, colormap, xyzmap = self.get_heightmap(points, color, BOUNDS, PIXEL_SIZE)

      observation["image"] = colormap
      observation["xyzmap"] = xyzmap
    
    else:
      observation["image"] = None
      observation["xyzmap"] = None

    return observation

  def render_image(self, image_size=(720, 720), intrinsics=(360., 0, 360., 0, 360., 360., 0, 0, 1)):
    # print("rendering image")
    # Camera parameters.
    position = (0, -0.85, 0.4)
    orientation = (np.pi / 4 + np.pi / 48, np.pi, np.pi)
    orientation = pybullet.getQuaternionFromEuler(orientation)
    zrange = (0.01, 10.)
    noise=True

    # OpenGL camera settings.
    lookdir = np.float32([0, 0, 1]).reshape(3, 1)
    updir = np.float32([0, -1, 0]).reshape(3, 1)
    rotation = pybullet.getMatrixFromQuaternion(orientation)
    rotm = np.float32(rotation).reshape(3, 3)
    lookdir = (rotm @ lookdir).reshape(-1)
    updir = (rotm @ updir).reshape(-1)
    lookat = position + lookdir
    focal_len = intrinsics[0]
    znear, zfar = (0.01, 10.)
    viewm = pybullet.computeViewMatrix(position, lookat, updir)
    fovh = (image_size[0] / 2) / focal_len
    fovh = 180 * np.arctan(fovh) * 2 / np.pi

    # Notes: 1) FOV is vertical FOV 2) aspect must be float
    aspect_ratio = image_size[1] / image_size[0]
    projm = pybullet.computeProjectionMatrixFOV(fovh, aspect_ratio, znear, zfar)

    # Render with OpenGL camera settings.
    _, _, color, depth, segm = pybullet.getCameraImage(
        width=image_size[1],
        height=image_size[0],
        viewMatrix=viewm,
        projectionMatrix=projm,
        shadow=1,
        flags=pybullet.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
        renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)

    # Get color image.
    color_image_size = (image_size[0], image_size[1], 4)
    color = np.array(color, dtype=np.uint8).reshape(color_image_size)
    color = color[:, :, :3]  # remove alpha channel
    if noise:
      color = np.int32(color)
      color += np.int32(np.random.normal(0, 3, color.shape))
      color = np.uint8(np.clip(color, 0, 255))

    # Get depth image.
    depth_image_size = (image_size[0], image_size[1])
    zbuffer = np.float32(depth).reshape(depth_image_size)
    depth = (zfar + znear - (2 * zbuffer - 1) * (zfar - znear))
    depth = (2 * znear * zfar) / depth
    if noise:
      depth += np.random.normal(0, 0.003, depth.shape)

    intrinsics = np.float32(intrinsics).reshape(3, 3)
    return color, depth, position, orientation, intrinsics

  def get_pointcloud(self, depth, intrinsics):
    """Get 3D pointcloud from perspective depth image.
    Args:
      depth: HxW float array of perspective depth in meters.
      intrinsics: 3x3 float array of camera intrinsics matrix.
    Returns:
      points: HxWx3 float array of 3D points in camera coordinates.
    """
    height, width = depth.shape
    xlin = np.linspace(0, width - 1, width)
    ylin = np.linspace(0, height - 1, height)
    px, py = np.meshgrid(xlin, ylin)
    px = (px - intrinsics[0, 2]) * (depth / intrinsics[0, 0])
    py = (py - intrinsics[1, 2]) * (depth / intrinsics[1, 1])
    points = np.float32([px, py, depth]).transpose(1, 2, 0)
    return points

  def transform_pointcloud(self, points, transform):
    """Apply rigid transformation to 3D pointcloud.
    Args:
      points: HxWx3 float array of 3D points in camera coordinates.
      transform: 4x4 float array representing a rigid transformation matrix.
    Returns:
      points: HxWx3 float array of transformed 3D points.
    """
    padding = ((0, 0), (0, 0), (0, 1))
    homogen_points = np.pad(points.copy(), padding,
                            'constant', constant_values=1)
    for i in range(3):
      points[Ellipsis, i] = np.sum(transform[i, :] * homogen_points, axis=-1)
    return points

  def get_heightmap(self, points, colors, bounds, pixel_size):
    """Get top-down (z-axis) orthographic heightmap image from 3D pointcloud.
    Args:
      points: HxWx3 float array of 3D points in world coordinates.
      colors: HxWx3 uint8 array of values in range 0-255 aligned with points.
      bounds: 3x2 float array of values (rows: X,Y,Z; columns: min,max) defining
        region in 3D space to generate heightmap in world coordinates.
      pixel_size: float defining size of each pixel in meters.
    Returns:
      heightmap: HxW float array of height (from lower z-bound) in meters.
      colormap: HxWx3 uint8 array of backprojected color aligned with heightmap.
      xyzmap: HxWx3 float array of XYZ points in world coordinates.
    """
    width = int(np.round((bounds[0, 1] - bounds[0, 0]) / pixel_size))
    height = int(np.round((bounds[1, 1] - bounds[1, 0]) / pixel_size))
    heightmap = np.zeros((height, width), dtype=np.float32)
    colormap = np.zeros((height, width, colors.shape[-1]), dtype=np.uint8)
    xyzmap = np.zeros((height, width, 3), dtype=np.float32)

    # Filter out 3D points that are outside of the predefined bounds.
    ix = (points[Ellipsis, 0] >= bounds[0, 0]) & (points[Ellipsis, 0] < bounds[0, 1])
    iy = (points[Ellipsis, 1] >= bounds[1, 0]) & (points[Ellipsis, 1] < bounds[1, 1])
    iz = (points[Ellipsis, 2] >= bounds[2, 0]) & (points[Ellipsis, 2] < bounds[2, 1])
    valid = ix & iy & iz
    points = points[valid]
    colors = colors[valid]

    # Sort 3D points by z-value, which works with array assignment to simulate
    # z-buffering for rendering the heightmap image.
    iz = np.argsort(points[:, -1])
    points, colors = points[iz], colors[iz]
    px = np.int32(np.floor((points[:, 0] - bounds[0, 0]) / pixel_size))
    py = np.int32(np.floor((points[:, 1] - bounds[1, 0]) / pixel_size))
    px = np.clip(px, 0, width - 1)
    py = np.clip(py, 0, height - 1)
    heightmap[py, px] = points[:, 2] - bounds[2, 0]
    for c in range(colors.shape[-1]):
      colormap[py, px, c] = colors[:, c]
      xyzmap[py, px, c] = points[:, c]
    colormap = colormap[::-1, :, :]  # Flip up-down.
    xv, yv = np.meshgrid(np.linspace(BOUNDS[0, 0], BOUNDS[0, 1], height),
                         np.linspace(BOUNDS[1, 0], BOUNDS[1, 1], width))
    xyzmap[:, :, 0] = xv
    xyzmap[:, :, 1] = yv
    xyzmap = xyzmap[::-1, :, :]  # Flip up-down.
    heightmap = heightmap[::-1, :]  # Flip up-down.
    return heightmap, colormap, xyzmap
  
  def on_top_of(self, obj_a, obj_b):
    """
    check if obj_a is on top of obj_b
    condition 1: l2 distance on xy plane is less than a threshold
    condition 2: obj_a is higher than obj_b
    """
    obj_a_pos = self.get_obj_pos(obj_a)
    obj_b_pos = self.get_obj_pos(obj_b)
    xy_dist = np.linalg.norm(obj_a_pos[:2] - obj_b_pos[:2])
    if obj_b in CORNER_POS:
      is_near = xy_dist < 0.06
      return is_near
    elif 'bowl' in obj_b:
      is_near = xy_dist < 0.06
      is_higher = obj_a_pos[2] > obj_b_pos[2]
      return is_near and is_higher
    else:
      is_near = xy_dist < 0.04
      is_higher = obj_a_pos[2] > obj_b_pos[2]
      return is_near and is_higher

  def hand_empty(self):
    """
    True if gripper is empty
    """
    return self.gripper.check_if_gripper_empty()

  def on_table(self, obj_a):
    """
    True if obj_a is on table
    """
    obj_a_pos = self.get_obj_pos(obj_a)
    if obj_a_pos[2] < 0.03:
      return True
    return False

  def clear(self, obj_a):
    obj_a_pos = self.get_obj_pos(obj_a)
    for obj_b in self.object_list:
      is_obj_b_on_top_of_a = self.on_top_of(obj_a = obj_b, obj_b= obj_a)      
      if is_obj_b_on_top_of_a:
        return False
    return True

  def get_obj_id(self, obj_name):
    try:
      if obj_name in self.obj_name_to_id:
        obj_id = self.obj_name_to_id[obj_name]
      else:
        obj_name = obj_name.replace('circle', 'bowl').replace('square', 'block').replace('small', '').strip()
        obj_id = self.obj_name_to_id[obj_name]
    except:
      print(f'requested_name="{obj_name}"')
      print(f'available_objects_and_id="{self.obj_name_to_id}')
    return obj_id
  
  def get_obj_pos(self, obj_name):
    obj_name = obj_name.replace('the', '').replace('_', ' ').strip()
    if obj_name in CORNER_POS:
      position = np.float32(np.array(CORNER_POS[obj_name]))
    else:
      pick_id = self.get_obj_id(obj_name)
      pose = pybullet.getBasePositionAndOrientation(pick_id)
      position = np.float32(pose[0])
    return position
  
  def get_obj_orn(self, obj_name):
    obj_name = obj_name.replace('the', '').replace('_', ' ').strip()
    pick_id = self.get_obj_id(obj_name)
    pose = pybullet.getBasePositionAndOrientation(pick_id)
    orn = np.float32(pose[1])
    return orn
  
  def get_bounding_box(self, obj_name):
    obj_id = self.get_obj_id(obj_name)
    return pybullet.getAABB(obj_id)

  def get_object_positions(self):
    obj_pos = []
    for objs in self.object_list:
      obj_pos.append(self.get_obj_pos(objs).copy())
    return obj_pos    
  
  def move_to_location(self, obj_name):
    """
    Move the robot to location and hover above it
    """
    if obj_name in self.location_ids:
      loc_id = self.location_ids[obj_name]
      loc_pos = pybullet.getBasePositionAndOrientation(loc_id)[0]
      
      # Hover above the location
      hover_pos = np.float32([loc_pos[0], loc_pos[1], 0.2])
      
      steps = 0
      ee_xyz = self.get_ee_pos()
      
      # Move to hover position above location
      while np.linalg.norm(hover_pos - ee_xyz) > 0.01:
        self.movep(hover_pos)
        self.step_sim_and_render()
        ee_xyz = self.get_ee_pos()
        steps += 1
        if steps > self.max_steps:
          print(f"Max steps reached while moving to location {obj_name}")
          return False
      
      # Stabilize at hover position
      for _ in range(150):
        self.step_sim_and_render()
      
      return True
    else:
      print(f"Location {obj_name} not found")
      return False
    
  def press_button(self, button_name):
    """
    Press a button on the coffee machine by moving to it and tapping with gripper
    """
    if button_name not in self.obj_name_to_id:
      print(f"Button {button_name} not found")
      return False
    
    button_id = self.obj_name_to_id[button_name]
    button_pos = pybullet.getBasePositionAndOrientation(button_id)[0]
    
    # Positions for button pressing sequence
    hover_pos = np.float32([button_pos[0], button_pos[1], button_pos[2] + 0.05])  # Hover above button
    press_pos = np.float32([button_pos[0], button_pos[1], button_pos[2]])  # Press down on button
    
    steps = 0
    ee_xyz = self.get_ee_pos()
    
    # Move to hover position above button
    while np.linalg.norm(hover_pos - ee_xyz) > 0.01:
      self.movep(hover_pos)
      self.step_sim_and_render()
      ee_xyz = self.get_ee_pos()
      steps += 1
      if steps > self.max_steps:
        print(f"Max steps reached while moving to button {button_name}")
        return False
    
    # Stabilize above button
    for _ in range(30):
      self.step_sim_and_render()
    
    # Close gripper before pressing
    self.gripper.activate()
    for _ in range(240):
      self.step_sim_and_render()
    
    # Move down to press the button
    ee_xyz = self.get_ee_pos()
    while np.linalg.norm(press_pos - ee_xyz) > 0.005:
      self.movep(press_pos)
      self.step_sim_and_render()
      ee_xyz = self.get_ee_pos()
      steps += 1
      if steps > self.max_steps:
        print(f"Max steps reached while pressing button {button_name}")
        self.gripper.release()
        return False
    
    # Hold press for a moment
    for _ in range(60):
      self.step_sim_and_render()
    
    # Move back up to hover position
    ee_xyz = self.get_ee_pos()
    while np.linalg.norm(hover_pos - ee_xyz) > 0.01:
      self.movep(hover_pos)
      self.step_sim_and_render()
      ee_xyz = self.get_ee_pos()
      steps += 1
      if steps > self.max_steps:
        print(f"Max steps reached while retracting from button {button_name}")
        break
    
    # Release gripper
    self.gripper.release()
    for _ in range(240):
      self.step_sim_and_render()
    
    return True
  
  def change_color(self, obj_name, color):
    """
    Change the color of an object in the simulation
    
    Args:
        obj_name: Name of the object to change color
        color: New color as a tuple (R, G, B) with values in [0, 255]
    """
    if obj_name not in self.obj_name_to_id:
      print(f"Object {obj_name} not found")
      return False
    
    obj_id = self.obj_name_to_id[obj_name]
    if color not in COLORS:
      print(f"Color {color} not recognized. Available colors: {list(COLORS.keys())}")
      return False
    pybullet.changeVisualShape(obj_id, -1, rgbaColor=COLORS[color])
    return True
  
  def save_video(self, filename="simulation_video.mp4", fps=32):
    """
    Save the cached video frames to a video file
    
    Args:
        filename: Output video filename (should end with .mp4, .avi, etc.)
        fps: Frames per second for the output video
    """
    if not self.cache_video:
        print("No video frames cached. Make sure rendering is enabled.")
        return False
    
    # Get video dimensions from first frame
    height, width, channels = self.cache_video[0].shape
    
    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # You can also use 'XVID'
    out = cv2.VideoWriter(filename, fourcc, fps, (width, height))
    
    # Write each frame to the video file
    for frame in self.cache_video:
        # Convert RGB to BGR for OpenCV
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        out.write(frame_bgr)
    
    # Release the VideoWriter
    out.release()
    
    print(f"Video saved as {filename} with {len(self.cache_video)} frames")
    return True