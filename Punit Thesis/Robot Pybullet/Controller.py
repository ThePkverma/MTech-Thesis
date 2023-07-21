import pybullet as p 
import numpy as np
import pybullet_data 
from control import lqr
from data import data as ss
from numpy import rad2deg,deg2rad
import math as m
import time

class Controller():
    def __init__(self,id,pid):
        self.id = id
        self.pid = pid
        self.r = 0.07
        self.d = 0.210
        self.n = p.getNumJoints(self.id)
        p.changeVisualShape(self.id,-1,rgbaColor=[1,0,0,6])
        p.changeVisualShape(self.id,2,rgbaColor=[0, 1, 1, 1])
        p.changeVisualShape(self.id,4,rgbaColor=[0, 1, 1, 1])
        p.changeVisualShape(self.id,12,rgbaColor=[1,0,0,6])
        p.changeVisualShape(self.id,8,rgbaColor=[1,0,0,6])
        self.beta = deg2rad(46.1)
        pos,orn = p.getBasePositionAndOrientation(self.id)
        self.createConstraints([ [9,3], [5,13] ])
        self.Simulation()
        
        for i in range(self.n):
            maxForce = 0
            mode = p.VELOCITY_CONTROL
            p.setJointMotorControl2( self.id, i,
                                    controlMode=mode, force=maxForce)
        thb = p.getEulerFromQuaternion(orn)[1]
        self.get_q4()
        _,self.q3,_ = self.fourbar(self.q4)
        self.q10 = thb-self.q4-self.q3+ 2*np.pi
        self.x = 0
        self.phi= 0 
        # time.sleep(2)
        
    def rtang(self,alpha):
        alpha = np.where(alpha > 0, alpha, alpha + np.pi)
        return alpha

    def fourbar(self,t3):
        tm = t3 - np.pi + np.deg2rad(46.1)
        
        l = np.array([0.188])
        h = np.array([0.093])
        g = np.array([0.193])
        f = np.array([0.05])
        
        ln = np.sqrt(l**2 + h**2 - 2*l*h*np.cos(tm))
        
        alpha = np.arccos((g**2 + f**2 - ln**2) / (2*g*f))
        alpha = self.rtang(alpha)
        
        gamma = np.arccos((h**2 + ln**2 - l**2) / (2*h*ln)) + np.arccos((g**2 + ln**2 - f**2) / (2*g*ln))
        gamma = self.rtang(gamma)
        
        lembda = 2*np.pi - (gamma + alpha + tm)
        
        return np.array([np.pi - alpha, lembda, t3])

    def Calibration(self,ang):
        tm =  ang[2]+ self.beta-np.pi
        ta1 = ang[0]-deg2rad(31.37)
        ta2 = ang[1]-deg2rad(9.62)
        tm = -tm+deg2rad(np.array([80.50,64]))
        return [ta1,ta2,tm[0],tm[1]]

    def createConstraints(self,indPair):
        p.changeDynamics(bodyUniqueId=self.id, 
                        linkIndex=4, 
                        jointLowerLimit=-np.deg2rad(35), 
                        jointUpperLimit=0)
        p.changeDynamics(bodyUniqueId=self.id, 
                        linkIndex=2, 
                        jointLowerLimit=-np.deg2rad(20), 
                        jointUpperLimit= np.deg2rad(15))
        p.changeDynamics(bodyUniqueId=self.id, 
                        linkIndex=0, 
                        jointLowerLimit=np.deg2rad(-1.37), 
                        jointUpperLimit=np.deg2rad( 118.63))
        rf = 0.3
        p.changeDynamics(bodyUniqueId = self.id,
                         linkIndex=12,rollingFriction=rf,
                         restitution=0)
        p.changeDynamics(bodyUniqueId = self.id,
                         linkIndex=8,rollingFriction=rf,
                         restitution=0)

        p.changeVisualShape(self.id, -1, rgbaColor=[0.29, 0.33, 0.64, 1.0])  # Index -1 color: #4a54a4
        p.changeVisualShape(self.id, 8, rgbaColor=[0.29, 0.33, 0.64, 1.0])  # Index 8 color: #4a54a4
        p.changeVisualShape(self.id, 12, rgbaColor=[0.29, 0.33, 0.64, 1.0])  # Index 12 color: #4a54a4

        # Change color for all other indices
        for i in range(p.getNumJoints(self.id)):
            if i not in [-1, 8, 12]:
                p.changeVisualShape(self.id, i, rgbaColor=[0.16, 0.7, 0.29, 1.0])  # Other indices color: #28b34b

        self.c = []
        for i in indPair:
            a,b = i
            ci = p.createConstraint(self.id,a,self.id,b,
                            jointType = p.JOINT_POINT2POINT,
                            jointAxis = [0,0,-1],
                            parentFramePosition = [0,0,0],
                            childFramePosition =  [0,0,0]
                            )
            self.c.append(ci)
            
        

    def ResetState(self,):
        ind = [0,1,2,4,8,12]
        ResetAng = np.append(self.Calibration([np.pi/2,0,3.780]),[0,0])
        # p.resetBasePositionAndOrientation(self.id, [0,0,-0.1], 
        #        p.getQuaternionFromEuler([0,0,0]))
        # p.setJointMotorControlArray(self.id,ind,
        #                         controlMode = p.POSITION_CONTROL,
        #                         targetPositions = ResetAng
        #                         )
        # for i in range(len(ind)):
        #     p.resetJointState(self.id,ind[i],ResetAng[i])

    def Simulation(self,):
        tic = time.time()
        toc = tic
        while toc-tic>5:
            toc = time.time()
            self.ResetState()
            p.stepSimulation()
            time.sleep(1/240)

    def getArmAngle(self):
        qa1,self.qa1_dot = p.getJointState(self.id,0)[:2]
        self.qa1 = qa1+deg2rad(31.37)
        qa2,self.qa2_dot = p.getJointState(self.id,1)[:2]
        self.qa2 = qa2+deg2rad(9.62)
        # print(np.round([self.qa2,self.qa2_dot],3))

    def get_q4(self):       #L   R
        q4l,q4lv = p.getJointState(self.id,2)[:2]
        q4r,q4rv = p.getJointState(self.id,4)[:2]
        off = np.deg2rad(np.array([80.50,64]))
        q4 = np.array([q4l,q4r])
        thm = off - q4
        self.q4 = np.pi + thm - self.beta
        self.q4_dot= -np.array([q4lv,q4rv])
        print(np.round([self.q4,self.q4_dot],3))

    def get_XPHI(self):
        qrdot =   p.getJointState(self.id,12)[1]
        qldot =   p.getJointState(self.id,8)[1]
        self.xdot = -(qrdot + qldot)/2*self.r 
        self.phi_dot = -(qrdot - qldot)*self.r/self.d
        self.x = self.x + self.xdot*self.dt
        self.phi = self.phi + self.phi_dot*self.dt
        # print(np.round([qrdot,qldot,self.xdot,self.phi_dot],3))
    
    def get_q1(self):
        pos,orn = p.getBasePositionAndOrientation(self.id)
        thb = p.getEulerFromQuaternion(orn)[1]
        _,self.q3,_ = self.fourbar(self.q4)
        self.q1 = thb-self.q4-self.q3+ 2*np.pi
        self.q1_dot = (self.q1 - self.q10)/self.dt
        self.q10 = self.q1
    
    def getStates(self,dt):
        self.dt = dt
        self.getArmAngle()
        self.get_q4()
        self.get_XPHI()
        self.get_q1()
        # print(np.round([self.x,self.xdot,self.phi,self.phi_dot],3))

        state = [self.x,self.phi,self.q1,self.q4,self.qa1,self.qa2,
                          self.xdot,self.phi_dot,self.q1_dot,self.q4_dot,self.qa1_dot,
                          self.qa2_dot]
        # print(self.q4_dot,np.diff(self.q4_dot))
        # print(self.q1_dot,np.diff(self.q1_dot))
        # print(rad2deg(self.qa2),self.qa2_dot)
        # print(p.getLinkState(self.id,12)[0][2])
        # print(rad2deg(self.q1),self.q1_dot)
        state = np.array([np.average(st) for st in state])
        print('ss',np.round(state,2))
        # print(self.x)
        return state








