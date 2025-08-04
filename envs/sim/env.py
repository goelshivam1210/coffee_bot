from envs.sim.configs import COLORS, CUP_CONFIGS, BUTTON_CONFIGS, LOCATION_CONFIGS, CORNER_POS, PIXEL_SIZE, BOUNDS
import os
import pybullet
import pybullet_data
import cv2
import numpy as np
from utils.transform_utils import correct_quaternion_ignore_roll
from envs.sim.gripper import Robotiq2F85
from envs.wrappers.cup import Cup
from envs.wrappers.location import Location

class SimActions:
  def __init__(self, cups, locations, buttons, robot_location, render=False, high_res=False, high_frame_rate=False, max_steps=5000):
    self.base_env = PickPlaceEnv(render=render, high_res=high_res, high_frame_rate=high_frame_rate, max_steps=max_steps)
    self.base_env.reset(cups, locations, buttons)
    self.move_to_location(robot_location)

  def reset(self, cups, locations, buttons, robot_location):
    self.base_env.reset(cups, locations, buttons)
    self.move_to_location(robot_location)

  def pick(self, obj_to_pick):
    """Do pick and place motion primitive."""

    self.base_env.gripper.release()
    steps = 0

    if not self.base_env.locate(obj_to_pick):
      print(f"cannot pick as {obj_to_pick} cannot be located")
      return False
    # if not self.base_env.clear(obj_to_pick) :
    #   print(f"cannot pick as {obj_to_pick} as its not clear")
    #   return False

    pick_pos = self.base_env.get_obj_pos(obj_to_pick).copy()
    obj_orn = self.base_env.get_obj_orn(obj_to_pick).copy()
    # we only care about the z , since the cube might be rotated
    obj_orn = correct_quaternion_ignore_roll(obj_orn)

    # add home_ee_euler to the orientation

    home_quat = pybullet.getQuaternionFromEuler(self.base_env.home_ee_euler)
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
    ee_xyz = self.base_env.get_ee_pos()
    while np.linalg.norm(hover_xyz - ee_xyz) > 0.01:
      # self.base_env.movep(hover_xyz)
      self.base_env.move_with_ee(hover_xyz, pick_orn)
      self.base_env.step_sim_and_render()
      ee_xyz = self.base_env.get_ee_pos()
      steps += 1
      if steps > self.base_env.max_steps:
        print("max steps reached while moving to hover position")
        return False

    while np.linalg.norm(pick_xyz - ee_xyz) > 0.01:
      # self.base_env.movep(pick_xyz)
      self.base_env.move_with_ee(pick_xyz, pick_orn)
      self.base_env.step_sim_and_render()
      ee_xyz = self.base_env.get_ee_pos()
      steps += 1
      if steps > self.base_env.max_steps:
        print("max steps reached while moving to pick position")
        return False

    # Pick up object.
    self.base_env.gripper.activate()
    for _ in range(240):
      self.base_env.step_sim_and_render()
    while np.linalg.norm(hover_xyz - ee_xyz) > 0.01:
      self.base_env.movep(hover_xyz)
      self.base_env.step_sim_and_render()
      ee_xyz = self.base_env.get_ee_pos()
      steps += 1
      if steps > self.base_env.max_steps:
        print("max steps reached while moving to hover position after pick")
        # release the object
        self.base_env.gripper.release()
        return False
    
    for _ in range(50):
      self.base_env.step_sim_and_render()

    if self.hand_empty():
      return False
    

    if obj_to_pick == 'blue block' and not self.hand_empty():
      # check the object that is closest to the gripper
      obj_pos = []
      for objs in self.base_env.object_list:
        obj_pos.append(self.base_env.get_obj_pos(objs).copy())
      obj_pos = np.array(obj_pos)
      ee_pos = self.base_env.get_ee_pos()
      dist = np.linalg.norm(obj_pos - ee_pos, axis=1)
      closest_obj = self.base_env.object_list[np.argmin(dist)]
      # print(f"closest object to gripper is {closest_obj}")
      if closest_obj == obj_to_pick:
        print("picked up the correct object!")

    return True
  

  def place(self, which_object, obj_to_place, find_empty_pos=False):
    """Do place motion primitive."""

    if 'table' in obj_to_place:
      self.base_env.putdown()
      return True

    if not self.base_env.locate(obj_to_place):
      print(f"cannot place as {obj_to_place} cannot be located")
      return False
    # if not self.base_env.clear(obj_to_place) :
    #   print(f"cannot place as {obj_to_place} as its not clear")
    #   return False
        
    place_pos = self.base_env.get_obj_pos(obj_to_place).copy()

    if place_pos.shape[-1] == 2:
      place_xyz = np.append(place_pos, 0.15)
    else:
      place_xyz = place_pos
      place_xyz[2] = 0.15

    # find empty position on top of the obj_to_place area if find_empty_pos is True
    if find_empty_pos:
      obj_pos = []
      for objs in self.base_env.object_list:
        if objs == obj_to_place:
          continue
        obj_pos.append(self.base_env.get_obj_pos(objs).copy())
      
      # Check if initial place_xyz is already good
      total_objects_far = 0
      for pos in obj_pos:
        if np.linalg.norm(place_xyz[:2] - pos[:2]) > 0.07:
          total_objects_far += 1
      
      # Only search for new position if initial one is not suitable
      if total_objects_far != len(obj_pos):
        num_choose_times = 0
        while True:
          random_empty_pos_candidate = [np.random.uniform(place_pos[0] - .08, 
                                                          place_pos[0] + .08), 
                                        np.random.uniform(place_pos[1] - .10, 
                                                          place_pos[1] + .10), 
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
    

    ee_xyz = self.base_env.get_ee_pos()
    # Move to place location.
    while np.linalg.norm(place_xyz - ee_xyz) > 0.01:
      self.base_env.movep(place_xyz)
      self.base_env.step_sim_and_render()
      ee_xyz = self.base_env.get_ee_pos()

    # Place down object.
    while (not self.base_env.gripper.detect_contact()) and (place_xyz[2] > 0.03):
      place_xyz[2] -= 0.001
      self.base_env.movep(place_xyz)
      for _ in range(3):
        self.base_env.step_sim_and_render()
    self.base_env.gripper.release()
    for _ in range(240):
      self.base_env.step_sim_and_render()
    place_xyz[2] = 0.2
    ee_xyz = self.base_env.get_ee_pos()
    while np.linalg.norm(place_xyz - ee_xyz) > 0.01:
      self.base_env.movep(place_xyz)
      self.base_env.step_sim_and_render()
      ee_xyz = self.base_env.get_ee_pos()
    place_xyz = np.float32([0, -0.5, 0.2])
    while np.linalg.norm(place_xyz - ee_xyz) > 0.01:
      self.base_env.movep(place_xyz)
      self.base_env.step_sim_and_render()
      ee_xyz = self.base_env.get_ee_pos()

    if not self.hand_empty():
      return False
    return True

  def move_to_location(self, obj_name):
    """
    Move the robot to location and hover above it
    """
    if obj_name in self.base_env.location_ids:
      loc_id = self.base_env.location_ids[obj_name]
      loc_pos = pybullet.getBasePositionAndOrientation(loc_id)[0]
      
      # Hover above the location
      hover_pos = np.float32([loc_pos[0], loc_pos[1], 0.2])
      
      steps = 0
      ee_xyz = self.base_env.get_ee_pos()
      
      # Move to hover position above location
      while np.linalg.norm(hover_pos - ee_xyz) > 0.01:
        self.base_env.movep(hover_pos)
        self.base_env.step_sim_and_render()
        ee_xyz = self.base_env.get_ee_pos()
        steps += 1
        if steps > self.base_env.max_steps:
          print(f"Max steps reached while moving to location {obj_name}")
          return False
      
      # Stabilize at hover position
      for _ in range(150):
        self.base_env.step_sim_and_render()
      
      return True
    else:
      print(f"Location {obj_name} not found")
      return False
    
  def press_button(self, button_name):
    """
    Press a button on the coffee machine by moving to it and tapping with gripper
    """
    if button_name not in self.base_env.obj_name_to_id:
      print(f"Button {button_name} not found")
      return False
    
    button_id = self.base_env.obj_name_to_id[button_name]
    button_pos = pybullet.getBasePositionAndOrientation(button_id)[0]
    
    # Positions for button pressing sequence
    hover_pos = np.float32([button_pos[0], button_pos[1], button_pos[2] + 0.05])  # Hover above button
    press_pos = np.float32([button_pos[0], button_pos[1], button_pos[2]])  # Press down on button
    
    steps = 0
    ee_xyz = self.base_env.get_ee_pos()
    
    # Move to hover position above button
    while np.linalg.norm(hover_pos - ee_xyz) > 0.01:
      self.base_env.movep(hover_pos)
      self.base_env.step_sim_and_render()
      ee_xyz = self.base_env.get_ee_pos()
      steps += 1
      if steps > self.base_env.max_steps:
        print(f"Max steps reached while moving to button {button_name}")
        return False
    
    # Stabilize above button
    for _ in range(30):
      self.base_env.step_sim_and_render()
    
    # Close gripper before pressing
    self.base_env.gripper.activate()
    for _ in range(240):
      self.base_env.step_sim_and_render()
    
    # Move down to press the button
    ee_xyz = self.base_env.get_ee_pos()
    while np.linalg.norm(press_pos - ee_xyz) > 0.005:
      self.base_env.movep(press_pos)
      self.base_env.step_sim_and_render()
      ee_xyz = self.base_env.get_ee_pos()
      steps += 1
      if steps > self.base_env.max_steps:
        print(f"Max steps reached while pressing button {button_name}")
        self.base_env.gripper.release()
        return False
    
    # Hold press for a moment
    for _ in range(60):
      self.base_env.step_sim_and_render()
    
    # Move back up to hover position
    ee_xyz = self.base_env.get_ee_pos()
    while np.linalg.norm(hover_pos - ee_xyz) > 0.01:
      self.base_env.movep(hover_pos)
      self.base_env.step_sim_and_render()
      ee_xyz = self.base_env.get_ee_pos()
      steps += 1
      if steps > self.base_env.max_steps:
        print(f"Max steps reached while retracting from button {button_name}")
        break
    
    # Release gripper
    self.base_env.gripper.release()
    for _ in range(240):
      self.base_env.step_sim_and_render()
    
    return True

  def on_top_of(self, obj_a, obj_b):
    """
    check if obj_a is on top of obj_b
    condition 1: l2 distance on xy plane is less than a threshold
    condition 2: obj_a is higher than obj_b
    """
    obj_a_pos = self.base_env.get_obj_pos(obj_a)
    obj_b_pos = self.base_env.get_obj_pos(obj_b)
    xy_dist = np.linalg.norm(obj_a_pos[:2] - obj_b_pos[:2])
    if obj_b in CORNER_POS:
      is_near = xy_dist < 0.06
      return is_near
    elif 'bowl' in obj_b:
      is_near = xy_dist < 0.06
      is_higher = obj_a_pos[2] > obj_b_pos[2]
      return is_near and is_higher
    else:
      is_near = xy_dist < 0.12
      is_higher = obj_a_pos[2] >= obj_b_pos[2]
      return is_near and is_higher
    
  def hand_empty(self):
    """
    True if gripper is empty
    """
    return self.base_env.gripper.check_if_gripper_empty()





class PickPlaceEnv:
  def __init__(self, render=False, high_res=False, high_frame_rate=False, max_steps=5000):
    self.render = render
    self.dt = 1/480
    self.sim_step = 0

    if self.render:
        pybullet.connect(pybullet.GUI)
    else:
        pybullet.connect(pybullet.DIRECT)  # pybullet.GUI for local GUI.
    
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

  def reset(self, cups, locations, buttons):
    self._reset_simulation()
    self._load_objects(cups, locations, buttons)

  def _reset_simulation(self):
    # Reset pybullet
    pybullet.resetSimulation(pybullet.RESET_USE_DEFORMABLE_WORLD)
    pybullet.setGravity(0, 0, -9.8)
    self.cache_video = []
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)

    # Reset robot
    pybullet.loadURDF("plane.urdf", [0, 0, -0.001])
    self.robot_id = pybullet.loadURDF("envs/sim/ur5e/ur5e.urdf", [0, 0, 0], flags=pybullet.URDF_USE_MATERIAL_COLORS_FROM_MTL)
    self.ghost_id = pybullet.loadURDF("envs/sim/ur5e/ur5e.urdf", [0, 0, -10])  # For forward kinematics.
    self.joint_ids = [pybullet.getJointInfo(self.robot_id, i) for i in range(pybullet.getNumJoints(self.robot_id))]
    self.joint_ids = [j[0] for j in self.joint_ids if j[2] == pybullet.JOINT_REVOLUTE]

    for i in range(len(self.joint_ids)):
      pybullet.resetJointState(self.robot_id, self.joint_ids[i], self.home_joints[i])

    if self.gripper is not None:
      while self.gripper.constraints_thread.is_alive():
        self.constraints_thread_active = False
    self.gripper = Robotiq2F85(self.robot_id, self.ee_link_id)
    self.gripper.release()

    # Reset workspace
    plane_shape = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents=[0.3, 0.3, 0.001])
    plane_visual = pybullet.createVisualShape(pybullet.GEOM_BOX, halfExtents=[0.3, 0.3, 0.001])
    plane_id = pybullet.createMultiBody(0, plane_shape, plane_visual, basePosition=[0, -0.5, 0])
    pybullet.changeVisualShape(plane_id, -1, rgbaColor=[0.2, 0.2, 0.2, 1.0])

  def _load_objects(self, cups, locations, buttons):
    self.object_list = [str(cup) for cup in cups.values()] + [str(loc) for loc in locations.values()] + [str(btn) for btn in buttons.values()]
    self.obj_name_to_id = {}
    self.location_ids = {}

    # Create location areas
    obj_xyz = np.zeros((0, 3))
    for location in locations.values():
      loc_name = location.loc_name
      loc_type = location.loc_type
      if loc_type in LOCATION_CONFIGS:
        loc_config = LOCATION_CONFIGS[loc_type]
        position = loc_config['position']
        if loc_type == 'coffee-machine':
          # Coffee machine - larger box with buttons
          area_shape = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents=[0.08, 0.08, 0.05])
          area_visual = pybullet.createVisualShape(pybullet.GEOM_BOX, halfExtents=[0.08, 0.08, 0.05])
          area_id = pybullet.createMultiBody(0, area_shape, area_visual, basePosition=[position[0], position[1], 0.05])
          pybullet.changeVisualShape(area_id, -1, rgbaColor=COLORS[loc_config['color']])
          obj_xyz = np.concatenate((obj_xyz, np.array(position).reshape(1, 3)), axis=0)
          # Add buttons to coffee machine
          button_offset = 0.075  # Offset from coffee machine center
          for button in buttons.values():
            button_name = button.button_name
            button_type = button.button_type
            if BUTTON_CONFIGS[button_type]['coffee-machine']:
              btn_config = BUTTON_CONFIGS[button_type]
              btn_x = position[0] + (list(BUTTON_CONFIGS.keys()).index(button_type) - 1) * 0.03
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
          if loc_type == 'dirty-area':
            # Create cleaning button in corner of dirty area
            for button in buttons.values():
              if button.button_type == 'cleaning-button':
                btn_shape = pybullet.createCollisionShape(pybullet.GEOM_CYLINDER, radius=0.01, height=0.005)
                btn_visual = pybullet.createVisualShape(pybullet.GEOM_CYLINDER, radius=0.01, length=0.005)
                btn_id = pybullet.createMultiBody(0, btn_shape, btn_visual, 
                                                basePosition=[position[0], position[1] - 0.1, 0.005])
                pybullet.changeVisualShape(btn_id, -1, rgbaColor=COLORS[BUTTON_CONFIGS['cleaning-button']['color']])
                self.obj_name_to_id[button.button_name] = btn_id
                obj_xyz = np.concatenate((obj_xyz, np.array([position[0], position[1] + 0.05, 0.005]).reshape(1, 3)), axis=0)
        self.obj_name_to_id[loc_name] = area_id
        self.location_ids[loc_name] = area_id
      else:
        raise ValueError(f"Unknown location type: {loc_type}")
      
    # Create cups

    for cup in cups.values():
      cup_name = cup.cup_name
      cup_type = cup.cup_type
      cup_location = locations.get(cup.cup_loc)
      if cup_type in CUP_CONFIGS and cup_location.loc_type in LOCATION_CONFIGS:
        cup_config = CUP_CONFIGS[cup_type]

        base_pos = LOCATION_CONFIGS[cup_location.loc_type]['position']
        

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
        self.obj_name_to_id[cup_name] = object_id
        obj_xyz = np.concatenate((obj_xyz, np.array(object_position).reshape(1, 3)), axis=0)
    
    # Re-enable rendering.
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)

    for _ in range(200):
      pybullet.stepSimulation()

    # record object positions at reset
    self.init_pos = {name: self.get_obj_pos(name) for name in self.object_list}

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

  def locate(self, obj_to_locate):
    if obj_to_locate in self.object_list:
      return True
    else:
      return False

  def step_sim_and_render(self):
    pybullet.stepSimulation()
    self.sim_step += 1
    #TODO: add way to pause simulation
    interval = 40 if self.high_frame_rate else 60
    # Render current image at 8 FPS.
    if self.sim_step % interval == 0 and self.render:
      self.cache_video.append(self.get_camera_image())

  def get_camera_image(self, draw_bounding_boxes=False):
    if not self.high_res:
      image_size = (240, 240)
      intrinsics = (120., 0, 120., 0, 120., 120., 0, 0, 1)
      color, _, _, _, _, bbs = self.render_image(image_size, intrinsics)
    else:
      color, _, _, _, _, bbs = self.render_image()

    return color, bbs


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

    segm_reshaped = np.array(segm).reshape(image_size[0], image_size[1])
    
    bounding_boxes = {}
    for obj_name, obj_id in self.obj_name_to_id.items():
        # Check if this is a cup
        print(obj_name)
        is_cup = 'cup' in obj_name.lower()

        if is_cup:
            # Find pixels belonging to this object
            mask = (segm_reshaped == obj_id)
            
            if np.any(mask):
                # Get coordinates of all pixels belonging to this object
                coords = np.where(mask)
                y_coords, x_coords = coords
                
                # Calculate bounding box
                x_min, x_max = np.min(x_coords), np.max(x_coords)
                y_min, y_max = np.min(y_coords), np.max(y_coords)

                bounding_boxes[obj_name] = {
                    'x_min': int(x_min),
                    'x_max': int(x_max),
                    'y_min': int(y_min),
                    'y_max': int(y_max)
                }
                

    # Get depth image.
    depth_image_size = (image_size[0], image_size[1])
    zbuffer = np.float32(depth).reshape(depth_image_size)
    depth = (zfar + znear - (2 * zbuffer - 1) * (zfar - znear))
    depth = (2 * znear * zfar) / depth
    if noise:
      depth += np.random.normal(0, 0.003, depth.shape)

    intrinsics = np.float32(intrinsics).reshape(3, 3)
    return color, depth, position, orientation, intrinsics, bounding_boxes
  
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