import pybullet as p
import time
import numpy as np
import sys

def round_to_zero(val, epsilon=1e-7):
    """Round small values to zero based on a threshold epsilon."""
    if abs(val) < epsilon:
        return 0.
    return val

class pybulletDebug:
    def __init__(self):
        #Camera paramers to be able to yaw pitch and zoom the camera (Focus remains on the robot) 
        self.cyaw=90
        self.cpitch=-7
        self.cdist=0.30
        time.sleep(0.5)
        
        self.xId = p.addUserDebugParameter("x" , -0.02 , 0.02 , 0.)
        self.yId = p.addUserDebugParameter("y" , -0.02 , 0.02 , 0.)
        self.zId = p.addUserDebugParameter("z" , -0.02 , 0.02 , 0.)
        self.rollId = p.addUserDebugParameter("roll" , -np.pi/4 , np.pi/4 , 0.)
        self.pitchId = p.addUserDebugParameter("pitch" , -np.pi/4 , np.pi/4 , 0.)
        self.yawId = p.addUserDebugParameter("yaw" , -np.pi/4 , np.pi/4 , 0.)
        self.LId = p.addUserDebugParameter("L" , -0.50 , 1 , 0.)
        self.LrotId = p.addUserDebugParameter("Lrot" , -1.50 , 1.50 , 0.)
        self.angleId = p.addUserDebugParameter("angleWalk" , -180. , 180. , 0.)
        self.periodId = p.addUserDebugParameter("stepPeriod" , 0.1 , 3. , 1.0)
        self.step_dur_asym = p.addUserDebugParameter("step_dur_asym" , -2 , 2. , 0.0)
        self.trotId = p.addUserDebugParameter("TROT" , 1 , 0 , 1)
        self.boundId = p.addUserDebugParameter("BOUND" , 1 , 0 , 1)
        
    
    def cam_and_robotstates(self , boxId):
        #orientation of camara
        direction=0
        cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
        p.resetDebugVisualizerCamera( cameraDistance=self.cdist, cameraYaw=self.cyaw, cameraPitch=self.cpitch, cameraTargetPosition=cubePos)
        keys = p.getKeyboardEvents()
        #Keys to change camera
        if keys.get(65296): #RIGHT
            self.cyaw-=1
        if keys.get(65295): #LEFT
            self.cyaw+=1
        if keys.get(65297): #UP    
            self.cpitch-=1
        if keys.get(65298): #DOWN
            self.cpitch+=1
        if keys.get(45):  #+
            self.cdist+=0.01
        if keys.get(61):  #-
            self.cdist-=0.01
        if keys.get(44):  #+
            l_add = 0.2
        if keys.get(46):  #-
            l_add = 0.2
            direction = 180
        if keys.get(27):  #ESC
            p.disconnect()
            time.sleep(2)
        if keys:
            for key, status in keys.items():
                if status & p.KEY_IS_DOWN:
                    print(f"Key {key} pressed")
        #   sys.exit()
        #read position from debug
        pos = np.array([p.readUserDebugParameter(self.xId),p.readUserDebugParameter(self.yId), p.readUserDebugParameter(self.zId)])
        orn = np.array([p.readUserDebugParameter(self.rollId),p.readUserDebugParameter(self.pitchId), p.readUserDebugParameter(self.yawId)])
        L = round_to_zero(p.readUserDebugParameter(self.LId))
        Lrot = p.readUserDebugParameter(self.LrotId)
        angle = p.readUserDebugParameter(self.angleId) + direction
        T = p.readUserDebugParameter(self.periodId)
        trot=p.readUserDebugParameter(self.trotId)
        bound=p.readUserDebugParameter(self.boundId)
        
        if trot==1:
          offset=[0.5, 0., 0., 0.5]
        elif bound==1:
          offset=[0.5, 0.5, 0., 0.]
        else:
          offset=[0.5, 0., 0., 0.5]
        
        
        return pos , orn , L , angle , Lrot , T , p.readUserDebugParameter(self.step_dur_asym), offset
