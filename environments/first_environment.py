import gym
from gym import error, utils, spaces
from gym.utils import seeding


from stable_baselines.sac.policies import MlpPolicy
from stable_baselines import SAC

import os
import math
import random
import numpy as np 
import pybullet as p 

class QuadEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.step_counter = 0
        self.episode_counter = 0
        p.connect(p.DIRECT)
        self.action_space = spaces.Box(
            np.array([-0.55, -0.3, -0.55, -0.3, -1.05, -0.8, -1.05, -0.8]),
            np.array([1.05, 0.8, 1.05, 0.8, 0.55, 0.3, 0.55, 0.3])
        )
        self.observation_space = spaces.Box(np.array([-10]*12), np.array([10]*12))

    def step(self, action):
        position, orientationq = p.getBasePositionAndOrientation(self.robotId)
        p.setJointMotorControlArray(self.robotId, [0, 1, 3, 4, 6, 7, 9, 10],
                                    p.POSITION_CONTROL, targetPositions=action)
        p.stepSimulation()
        new_position, new_orientationq = p.getBasePositionAndOrientation(self.robotId)
        new_orientation = p.getEulerFromQuaternion(new_orientationq)
        done = False
        if abs(new_orientation[0]) > 0.5 or abs(new_orientation[1]) > 0.5:
            done = True
        reward = (((new_position[0] - position[0]) ** 2 + (new_position[1] - position[1]) ** 2)
                  - 0.005 * math.exp(abs(new_orientation[0]) + abs(new_orientation[1])))
        joint_info = p.getJointStates(self.robotId, [0, 1, 3, 4, 6, 7, 9, 10])
        motor_positions = []
        for motor_info in joint_info:
            motor_positions.append(motor_info[0])
        base_ang_velocity = p.getBaseVelocity(self.robotId)[1]
        base_angles_and_velocity = [new_orientation[0], new_orientation[1],
                                    base_ang_velocity[0], base_ang_velocity[1]]
        self.observation = base_angles_and_velocity + motor_positions
        info = {
            "episode" : self.episode_counter,
            "pos" : new_position
        }
        self.step_counter += 1
        return self.observation, reward, done, info


    def reset(self):
        self.episode_counter += 1
        self.step_counter = 0
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.resetDebugVisualizerCamera(1, 45, -20, [0, 0, 0])
        planeId = p.loadURDF("plane_col.urdf")
        self.robotId = p.loadURDF("creation.urdf", [0, 0, 0.28])
        self.observation = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
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

env = QuadEnv()

model = SAC(MlpPolicy, env, verbose=1)
model.learn(total_timesteps=200, log_interval=10)
model.save("first_test")
