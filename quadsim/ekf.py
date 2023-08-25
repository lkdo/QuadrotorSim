import numpy as np
from scipy.integrate import odeint

class EKF:
    """ Implementation of the discrete-time EKF """
      
    def __init__(self, f, dfdx, G, Q, x0, P0, M=0, b=0):

        # ToDo: size checks
        self.f = f
        self.dfdx = dfdx
        self.G = G
        self.Q = Q
        self.x = x0
        self.P = P0
        self.n = self.x.shape[0]
        
        # Implements constraint M@x=b
        if (M==0):
            return 
        # ToDo: Implement size check
        self.M = M
        self.b = b

    def predict(self, u, dt, simple = 1):
        if (simple):  # euler integration (first order hold)
            self.x = self.x + self.f(self.x, 0, u)*dt
        else:
            Y = odeint(self.f,self.x,np.array([0, dt]),args=(u,))
            self.x = Y[1]  
        A = np.eye(self.x.shape[0]) + self.dfdx(self.x, u)*dt
        self.P = A@self.P@A.transpose() + dt*self.G@self.Q@self.G.transpose()

    def update(self, y, h, dhdx, R, var = 0):
        H = dhdx(self.x)
        Pxy = self.P@H.transpose()
        Py = H@self.P@H.transpose()
        K = Pxy@np.linalg.inv(Py+R)
        y_est = h(self.x)
        self.x = self.x + K@(y-y_est)
        if (var == 0):
            # Simple Covariance Update
            self.P = self.P - K@(Py+R)@np.transpose(K)
        else:
            # Joseph Form Covariance Update
            IUK = np.eye(self.n) - K@H
            self.P = IUK@self.P@IUK.transpose()+K@R@K.transpose()

    def apply_eq_constraint(self):
        #W = np.linalg.inv(self.P)
        #Wi = np.linalg.inv(W)

        # Option 1
        #Wi = self.P
        
        # Option 2
        Wi = np.eye(self.x.shape[0])

        A = np.linalg.inv(self.M@Wi@self.M.transpose())
        Lambda = Wi@self.M.transpose()*A
        
        self.x -= Lambda@(self.M@self.x-self.b)
        self.P = (np.eye(self.x.shape[0]) - Lambda@self.M)@self.P
##########################################################