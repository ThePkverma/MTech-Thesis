import pybullet as p
import time
import pybullet_data
import numpy as np
from control import lqr
from Controller import Controller
from data import data as data
from matplotlib.pyplot import *
ss = data()
path = "NewWholeBody.urdf"
physicsClient = p.connect(p.GUI)
physicsClient = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,-0.2+0.07]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,-np.pi])
id = p.loadURDF(path,cubeStartPos, cubeStartOrientation, 
                flags=p.URDF_USE_INERTIA_FROM_FILE,
                useFixedBase = 0 )
n = p.getNumJoints(id)
for i in range(n):
            maxForce = 0
            mode = p.VELOCITY_CONTROL
            p.setJointMotorControl2( id, i,
                                    controlMode=mode, force=maxForce)
cameraDistance = 1.36
cameraYaw =32
cameraPitch = -22
cameraTargetPosition = [0,0.30,0.467] # The position to look at
p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)


cc  = Controller(id,planeId)
def Calibration(ang):
    ta1 = ang[0]-31.37
    ta2 = ang[1]-9.62
    tm = -ang[2]+np.array([80.50,64])
    return np.deg2rad([ta1,ta2,tm[0],tm[1]])

k = ss.kg 
X0 = ss.Ref()
Time  = p.addUserDebugParameter(" TIme", 1/240, 1.2, 0.01)
arm1 = p.addUserDebugParameter(" arm1", 30, 150, 30)
arm2 = p.addUserDebugParameter(" arm2", -180, 180, 0)
Tm  = p.addUserDebugParameter("    Hip", 65, 100, 65)
zval  = p.addUserDebugParameter("    Z val", -0.2, 0.7, 0.5)
wheel = p.addUserDebugParameter("    Wheel", -np.pi*2, np.pi*2, 0)

a1t = p.addUserDebugParameter(" armT1", -1,1, 8.57748881e-02)
a2t = p.addUserDebugParameter(" armT2", -1, 1, 3.52048884e-07)
Ttm  = p.addUserDebugParameter("    HipT", -5, 5, -3.81380970e-01)
wheelT = p.addUserDebugParameter("    WheelT", -5, 5, 0)
kt = p.addUserDebugParameter("  kt", 0, 2, 0)

switch = p.addUserDebugParameter(" switch",0,1,0)
rf = p.addUserDebugParameter(" rolling  ",0,1,1)

t_time = 0
U =[]; Sol = []; T_time = []
# time.sleep(5)
while t_time<6:
# while 1:
    dt = p.readUserDebugParameter(Time)
    a1 = p.readUserDebugParameter(arm1)
    a2 = p.readUserDebugParameter(arm2)
    hip = p.readUserDebugParameter(Tm)
    bzval = p.readUserDebugParameter(zval)
    wheelval = p.readUserDebugParameter(wheel)
    ktv = -p.readUserDebugParameter(kt)
    swn = p.readUserDebugParameter(switch)
    rfv = p.readUserDebugParameter(rf)
    ang = [a1,a2,hip]
    Val= Calibration(ang)
    # dt = 0.05
    X = cc.getStates(dt)

    inputArray = [8,12,2,4,0,1,7,11]

    X0 = ss.Ref(t=t_time)
    X[0] =swn
    t_time = t_time + dt
    u = -np.dot(k,X-X0)
    u[2] = -u[2]
    u = u + ss.u0
    u = np.insert(u,2, u[2])
    u = np.append(u,-np.array([0.85*np.average(cc.q3),0.85*np.average(cc.q3)]))
    if t_time>2:
        p.setJointMotorControlArray(id,inputArray,
                            controlMode = p.TORQUE_CONTROL,
                            forces = u)
        print('u',np.round(u,2))    
        U.append(u)
        Sol.append(X)
        T_time.append(t_time)
        # print('u',np.round(u,2))                           
    p.stepSimulation()
    t_time = dt +t_time
    time.sleep(dt)


    
lab = ['$x$', '$\phi$', '$q_1$', '$q_4$', '$q_{a_{1}}$', '$q_{a_{2}}$', "$x'$", "$\phi'$", "$q_1'$", 
        "$q_4'$", "$q_{a_{1}}'$", "$q_{a_{2}}'$"]
sol = np.array(Sol)
U = np.array(U)
T_time = np.array(T_time)-2
figure()
for i in range(12):
    subplot(4,3,i+1)
    if i<6:
        plot(T_time,sol[:,i],'r')
        plot(T_time,X0[i]*np.ones_like(T_time),'g')
        legend(['Actual','Reference'])
    else:
        plot(T_time,sol[:,i],'r')
        legend(['Actual'])
    xlabel('t (sec)',fontsize = 12)

    ylabel(lab[i],fontsize = 12)
    grid(1)

lab = ['$T_l$', '$T_r$', '$T_m$', '$T_{a_{1}}$', '$T_{a_{2}}$']
figure()
j=0
for i in [0,1,2,4,5]:
    subplot(5,1,j+1)
    plot(T_time,U[:,i],)
    xlabel('t (sec)',fontsize = 12)
    ylabel(lab[j],fontsize = 12)
    j+=1
    grid(1)
      
show()



