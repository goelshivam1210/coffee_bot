from envs.simulation.configs import COLORS, CUP_CONFIGS, BUTTON_CONFIGS, LOCATION_CONFIGS, CORNER_POS, PIXEL_SIZE, BOUNDS
import os
import pybullet
import pybullet_data
import cv2
import numpy as np
from utils.transform_utils import correct_quaternion_ignore_roll
from envs.simulation.gripper import Robotiq2F85
from envs.wrappers.cup import Cup
from envs.wrappers.location import Location

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

  def reset(self, cups, locations, robot_location):
    return False

  def _reset_simulation(self, robot_location):
    # Reset pybullet
    pybullet.resetSimulation(pybullet.RESET_USE_DEFORMABLE_WORLD)
    pybullet.setGravity(0, 0, -9.8)
    self.cache_video = []
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)

    # Reset robot
    pybullet.loadURDF("plane.urdf", [0, 0, -0.001])
    self.robot_id = pybullet.loadURDF("envs/simulation/ur5e/ur5e.urdf", [0, 0, 0], flags=pybullet.URDF_USE_MATERIAL_COLORS_FROM_MTL)
    self.ghost_id = pybullet.loadURDF("envs/simulation/ur5e/ur5e.urdf", [0, 0, -10])  # For forward kinematics.
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

  def _load_objects(self, object_list):