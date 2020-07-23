import gym
from gym import error, utils, spaces
from gym.utils import seeding


from stable_baselines.sac.policies import MlpPolicy
from stable_baselines import SAC, PPO2

import os
import math
import random
import numpy as np 
import pybullet as p 
import pybullet_data as pd 


class Quadruped(gym.Env): 
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.step_counter = 0
        self.episode_counter = 0
        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pd.getDataPath())
        self.action_space = spaces.Box(
            np.array([-1.0]*12),
            np.array([0]*12)
        )
        self.observation_space = spaces.Box(np.array([-10]*16), np.array([10]*16))

    def step(self, action):
        position, orientationq = p.getBasePositionAndOrientation(self.robotId)
        j = 0
        for i in range(12):
            if i % 4 == 3:
                j += 1
            p.setJointMotorControl2(self.robotId, j,
                                    p.POSITION_CONTROL, targetPosition=action[i],
                                    maxVelocity=5.23, targetVelocity=5.23
            )
            j += 1
        
        p.stepSimulation()
        new_position, new_orientationq = p.getBasePositionAndOrientation(self.robotId)
        new_orientation = p.getEulerFromQuaternion(new_orientationq)
        done = False
        if abs(new_orientation[0] - 1.5707963268) > 1 or abs(new_orientation[1]) > 1:
            done = True
        reward = (600 * ((position[0] - new_position[0]))
                   - 1.2 * (abs(new_orientation[0]) - 0.2 + abs(new_orientation[1]) - 0.15)
                 )
        joint_info = p.getJointStates(self.robotId, [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14])
        motor_positions = []
        for motor_info in joint_info:
            motor_positions.append(motor_info[0])
        base_ang_velocity = p.getBaseVelocity(self.robotId)[1]
        base_angles_and_velocity = [new_orientation[0], new_orientation[1],
                                    base_ang_velocity[0], base_ang_velocity[1]]
        self.observation = np.array(base_angles_and_velocity + motor_positions)
        self.step_counter += 1
        return self.observation, reward, done, {}


    def reset(self):
        self.episode_counter += 1
        self.step_counter = 0
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.resetDebugVisualizerCamera(5, 45, -20, [0, 0, 0])
        self.visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents = [10,10,0.1])
        self.coll_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents = [10,10,0.1])
        print(self.episode_counter)
        self.planeId = p.loadURDF("/Users/caspa/VSCode/RSI_code/experiments/robots/terrain.urdf", [0, 0, 0])
        urdfFlags = p.URDF_USE_SELF_COLLISION
        self.robotId = p.loadURDF("laikago/laikago_toes.urdf", [0, 0, .5], [0, 0.5, 0.5, 0],
                       flags=urdfFlags,
                       useFixedBase=False)
        lower_legs = [2, 5, 8, 11]
        for l0 in lower_legs:
            for l1 in lower_legs:
                if (l1 > l0):
                    enableCollision = 1
                    # print("collision for pair", l0, l1,
                    #       p.getJointInfo(self.robotId, l0)[12],
                    #       p.getJointInfo(self.robotId, l1)[12], "enabled=", enableCollision)
                    p.setCollisionFilterPair(self.robotId, self.robotId, 2, 5, enableCollision)
        self.observation = np.array([0]*16)
        return self.observation

    def render(self, mode='human'):
        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0,0,0],
                                                            distance=5,
                                                            yaw=90,
                                                            pitch=-20,
                                                            roll=0,
                                                            upAxisIndex=2)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                     aspect=float(960) /720,
                                                     nearVal=0.1,
                                                     farVal=100.0)
        (_, _, px, _, _) = p.getCameraImage(width=960,
                                              height=720,
                                              viewMatrix=view_matrix,
                                              projectionMatrix=proj_matrix,
                                              renderer=p.ER_BULLET_HARDWARE_OPENGL)

        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (720,960, 4))

        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    def close(self):
        p.disconnect()
