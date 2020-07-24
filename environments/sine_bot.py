import pybullet as p
import numpy as np
import pybullet_data
from time import sleep
from math import sin,cos,pi
joints = [1,3,6,8,11,13,16,18]
num_params = 6
max_ang_vel = 5.23
max_torque = 1.5

def reset():
    p.resetSimulation()
    p.setPhysicsEngineParameter(enableConeFriction=0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81)
    p.loadURDF("/Users/caspa/VSCode/RSI_code/experiments/robots/terrain.urdf")
    robotID = p.loadURDF("/Users/caspa/VSCode/RSI_code/experiments/robots/robot.urdf",[0,0,0.15],flags = p.URDF_USE_INERTIA_FROM_FILE)
def sine_signal(a,w,t,offset,phi=0,freq_limit = 2*pi/120):
    w = np.clip(w,-freq_limit,freq_limit)
    return a*sin(w*t+phi)+offset

def fitness(paramlist,robotID) :
    reward = 0
    for ij in range(7200):
        sleep( 1/240)
        r_hip = sine_signal(paramlist[0],paramlist[1],ij,paramlist[2])
        r_knee = sine_signal(paramlist[3],paramlist[4],ij,paramlist[5],pi/2)
        l_hip = sine_signal(paramlist[0],paramlist[1],ij,paramlist[2],pi)
        l_knee = sine_signal(paramlist[3],paramlist[4],ij,paramlist[5],3*pi/2)
        action = [r_hip,r_knee,l_hip,l_knee,l_hip,l_knee,r_hip,r_knee]
        for joint,action in zip(joints,action):
            p.setJointMotorControl2(robotID,joint,p.POSITION_CONTROL, action, force = max_torque, maxVelocity = max_ang_vel) 
        p.stepSimulation()
        lin_vel,_ = p.getBaseVelocity(robotID)
        _, orientQuaternion = p.getBasePositionAndOrientation(robotID)
        orientRad = p.getEulerFromQuaternion(orientQuaternion)

        reward += lin_vel[1]

        if abs(orientRad[0]) >= 5 or abs(orientRad[1]) >= 5 or abs(orientRad[2]) >= 1.57 :
            reset()
            return reward
    reset()
    return reward
# F = front, H = hind, R and L = right and left
#1=FR hip motor
#3=FR knee motor

#6 = HR hip motor
#8 = HR knee motor

#11 = FL hip motor
#13 = FL knee motor

#16 = HL hip motor
#18 = HL knee motor
if __name__ == "__main__" :
    client = p.connect(p.GUI)
    p.setPhysicsEngineParameter()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81)
    p.loadURDF("/Users/caspa/VSCode/RSI_code/experiments/robots/terrain.urdf")
    robotID = p.loadURDF("/Users/caspa/VSCode/RSI_code/experiments/robots/robot.urdf",[0,0,0.15],flags = p.URDF_USE_INERTIA_FROM_FILE)
    fitness([-0.23047705, -0.13732716, -0.24319741,  1.41924727, -0.33635504,  0.19141735,],robotID)

    


