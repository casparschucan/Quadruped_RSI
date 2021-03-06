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
    
    #initializes environment
    def __init__(self):
        self.step_counter = 0
        self.episode_counter = 0
        p.connect(p.GUI)
        self.action_space = spaces.Box(
            np.array([-1.0472, 0.0, -1.0472, 0.0]),
            np.array([1.0472, 2.0944, 1.0472, 2.0944])
        )
        self.observation_space = spaces.Box(np.array([-10]*12), np.array([10]*12))
        
    #steps environment
    def step(self, action):
        #set robot in motion
        p.setJointMotorControl2(self.robotId, 0,
                                p.POSITION_CONTROL, targetPosition=action[0],
                                maxVelocity=5.23, targetVelocity=5.23
        )
        p.setJointMotorControl2(self.robotId, 1,
                                p.POSITION_CONTROL, targetPosition=action[1],
                                maxVelocity=5.23, targetVelocity=5.23
        )
        p.setJointMotorControl2(self.robotId, 3,
                                p.POSITION_CONTROL, targetPosition=action[2],
                                maxVelocity=5.23, targetVelocity=5.23
        )
        p.setJointMotorControl2(self.robotId, 4,
                                p.POSITION_CONTROL, targetPosition=action[3],
                                maxVelocity=5.23, targetVelocity=5.23
        )
        p.setJointMotorControl2(self.robotId, 6,
                                p.POSITION_CONTROL, targetPosition=action[2],
                                maxVelocity=5.23, targetVelocity=5.23
        )
        p.setJointMotorControl2(self.robotId, 7,
                                p.POSITION_CONTROL, targetPosition=action[3],
                                maxVelocity=5.23, targetVelocity=5.23
        )
        p.setJointMotorControl2(self.robotId, 9,
                                p.POSITION_CONTROL, targetPosition=action[0],
                                maxVelocity=5.23, targetVelocity=5.23
        )
        p.setJointMotorControl2(self.robotId, 10,
                                p.POSITION_CONTROL, targetPosition=action[1],
                                maxVelocity=5.23, targetVelocity=5.23
        )
        p.stepSimulation()
        
        #calculates reward
        new_position, new_orientationq = p.getBasePositionAndOrientation(self.robotId)
        new_orientation = p.getEulerFromQuaternion(new_orientationq)
        done = False
        if abs(new_orientation[0]) > 0.5 or abs(new_orientation[1]) > 0.5 or self.step_counter > 20000:
            done = True
        Joint_States = p.getJointStates(self.robotId, [0, 1, 3, 4, 6, 7, 9, 10])
        torpun = 0
        for joint_state in Joint_States:
            torpun += joint_state[1] * joint_state[3]
            
        #reward function (only important for training purposes) 
        #both different reward functions are sketched out here
        reward = (600 * ((position[1] - new_position[1]))
                  - 1 * (abs(new_orientation[0]) - 0.2 + abs(new_orientation[1]) - 0.2 )
                  #- 1 / 3 * 10 ** -5 * torpun
                 )
                 
        #gets observation
        joint_info = p.getJointStates(self.robotId, [0, 1, 3, 4, 6, 7, 9, 10])
        motor_positions = []
        for motor_info in joint_info:
            motor_positions.append(motor_info[0])
        base_ang_velocity = p.getBaseVelocity(self.robotId)[1]
        base_angles_and_velocity = [new_orientation[0], new_orientation[1],
                                    base_ang_velocity[0], base_ang_velocity[1]]
        self.observation = np.array(base_angles_and_velocity + motor_positions)
        self.step_counter += 1
        return self.observation, reward, done, new_position

    #resets environment to default conditions
    def reset(self, gravity):
        self.episode_counter += 1
        self.step_counter = 0
        p.resetSimulation()
        p.setGravity(0, 0, -gravity)
        p.resetDebugVisualizerCamera(1, 45, -20, [0, 0, 0])
        self.visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents = [10,10,0.1])
        self.coll_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents = [10,10,0.1])
        print(self.episode_counter)
        p.setAdditionalSearchPath(pd.getDataPath())
        self.planeId = p.loadURDF("plane.urdf", [0, 0, 0])
        
        #insert name of robot here or path of robot if not in same folder
        self.robotId = p.loadURDF("robot_name.urdf", [0, 0, 0.25])
        
        self.observation = np.array([0]*12)
        return self.observation
        
    #renders every step (not used here makes everything look like slowmotion)
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


env = Quadruped()

#insert the name of the model below without .zip
#if the model is not in the same folder insert model path
model = SAC.load("model_name")

#define the value of wanted gravitational acceleration
gravity = 9.81
obs = env.reset(gravity)
g = 0.95
act_filtered_old, _states = model.predict(obs)

#stepping of model and environment
for i in range(100000):
    act_unfiltered, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(act_unfiltered) 
    p.resetDebugVisualizerCamera(1, 45, -20, info)
    if dones:
        env.reset(gravity) 
env.close()
