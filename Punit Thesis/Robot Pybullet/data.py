import numpy as np
from control import lqr
class data():

    def __init__(self):

        self.A = np.array([[  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              1.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              0.00000000e+00,  0.00000000e+00,  0.00000000e+00],
                            [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              0.00000000e+00,  1.00000000e+00,  0.00000000e+00,
                              0.00000000e+00,  0.00000000e+00,  0.00000000e+00],
                            [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              0.00000000e+00,  0.00000000e+00,  1.00000000e+00,
                              0.00000000e+00,  0.00000000e+00,  0.00000000e+00],
                            [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              1.00000000e+00,  0.00000000e+00,  0.00000000e+00],
                            [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              0.00000000e+00,  1.00000000e+00,  0.00000000e+00],
                            [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              0.00000000e+00,  0.00000000e+00,  1.00000000e+00],
                            [ 0.00000000e+00, -9.69352280e-19,  5.26113845e+00,
                              -4.25737957e+00, -8.72448563e-02,  1.42869087e-02,
                              0.00000000e+00,  1.32367065e-10,  8.78213566e-11,
                              -9.53756568e-10, -2.78016696e-14,  3.10192730e-17],
                            [ 0.00000000e+00,  1.11544932e-18, -1.73302707e-03,
                              1.36886770e-03,  1.37169998e-04, -2.24686726e-05,
                              0.00000000e+00, -4.40409793e-14, -1.23170741e-11,
                              -9.19151632e-12, -3.32268213e-18, -3.62876016e-21],
                            [ 0.00000000e+00, -6.46234854e-19,  5.09960145e+01,
                              -4.95914607e+01, -3.77862341e+00,  6.17654940e-01,
                              0.00000000e+00,  1.18495760e-09,  7.00660625e-09,
                              1.57451565e-08,  8.44050573e-14,  1.03397577e-16],
                            [ 0.00000000e+00,  0.00000000e+00,  4.29724713e+00,
                              -1.43391528e+01, -3.71680938e-02,  4.57205835e-03,
                              0.00000000e+00, -2.12610957e-11,  7.99582036e-09,
                              1.19405028e-08, -7.47047491e-15,  1.03397577e-17],
                            [ 0.00000000e+00, -1.24077092e-16, -6.68159881e+01,
                              4.80439739e+01,  7.82746222e+01, -8.48893725e+01,
                              0.00000000e+00, -1.71013725e-09,  8.71222417e-10,
                              4.37164058e-09, -1.22334636e-11, -1.44756607e-16],
                            [ 0.00000000e+00,  3.30872245e-16,  2.65894895e+01,
                              -1.45216249e+01, -1.01752865e+02,  2.75809609e+02,
                              0.00000000e+00,  5.34107999e-10, -8.82550883e-10,
                              -1.69191613e-08,  3.52316489e-11,  4.13590306e-17]])
        

        self.B = np.array([[ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              0.00000000e+00,  0.00000000e+00],
                            [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              0.00000000e+00,  0.00000000e+00],
                            [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              0.00000000e+00,  0.00000000e+00],
                            [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              0.00000000e+00,  0.00000000e+00],
                            [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              0.00000000e+00,  0.00000000e+00],
                            [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                              0.00000000e+00,  0.00000000e+00],
                            [-4.83589492e+00, -4.83580051e+00,  1.56072013e-01,
                              -1.21334725e+00,  4.82852748e-01],
                            [ 2.29504524e+01, -2.29489325e+01, -2.02444295e-05,
                              8.57810895e-04, -7.59238717e-04],
                            [-2.66264518e+01, -2.66255208e+01,  1.15641227e+01,
                              -2.37913086e+01,  2.08986032e+01],
                            [-6.89687663e+00, -6.89684627e+00,  1.29430982e+01,
                              -4.21898890e-01,  1.86737854e-01],
                            [ 4.11261275e+01,  4.11235541e+01, -8.43797780e-01,
                              5.91790375e+02, -1.33592357e+03],
                            [-2.77976384e+01, -2.77953607e+01,  3.73475707e-01,
                              -1.33592357e+03,  3.80904092e+03]])
        
      
        # self.Q = np.diag([1000,1000,500,500,1,1,
        #                     1,1,1,1,1,1])
        # self.R = np.diag([5000,5000,5000,1000,1000])
        self.Q = np.diag([100,100,100,100,10,10,
                            1,1,1,1,1,1])
        self.R = np.diag([50,50,100,100,100])
        self.kg,_,_ = lqr(self.A,self.B,self.Q,self.R)
        self.u0 = np.array([ 0.00000000e+00,  0.00000000e+00, 0, -1.15687230e-04,
                            1.12203887e-07])
        

    def Ref(self,t=0):
        
        x0 = np.array([-2.50128821e-15,  0.00000000e+00,  7.22915376e-01,  3.78003351e+00,
        1.54146912e+00, -4.04384971e-04,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00])

        x_ =  2*np.pi*3*(t-2)/20
        # ph_ = 2*np.pi*3*(t-2)/20
        ph_ = 2*np.pi*np.sin(t/10)
        x0 = np.array([0 ,0,  7.22915376e-01,  3.78003351e+00,
        1.54146912e+00, -4.04384971e-04,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00])
        
        return x0
        
