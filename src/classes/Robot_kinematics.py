import numpy as np
import sympy as sp
import rospy

class RRRR_manipulator:
    def __init__(self,l):
        self.FK = self.calculate_FK_matrix(l)
        self.IK = self.calculate_IK_vector(l)
        
        self.joint_home_position = np.array([0, 0, 0, 0])
        cartesian_state = self.get_cartesian_state(self.joint_home_position)

        self.JPOS = self.joint_home_position 
        self.POS = cartesian_state['position']
        self.ROT = cartesian_state['rotation']

    def calculate_FK_matrix(self,l):
        t1,t2,t3,t4 = sp.symbols("t1, t2, t3, t4")
        
        A01 = sp.Matrix([
            [sp.cos(t1), 0, sp.sin(t1), 0],
            [sp.sin(t1), 0, -sp.cos(t1), 0],
            [0, 1, 0, l[0]],
            [0, 0, 0, 1],
            ])

        A12 = sp.Matrix([
            [sp.cos(t2), -sp.sin(t2), 0, l[1]*sp.cos(t2)],
            [sp.sin(t2), sp.cos(t2), 0,  l[1]*sp.sin(t2)],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            ])

        A23 = sp.Matrix([
            [sp.cos(t3), -sp.sin(t3), 0, l[2]*sp.cos(t3)],
            [sp.sin(t3), sp.cos(t3), 0,  l[2]*sp.sin(t3)],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            ])

        A34 = sp.Matrix([
            [sp.cos(t4), -sp.sin(t4), 0, l[3]*sp.cos(t4)],
            [sp.sin(t4), sp.cos(t4), 0,  l[3]*sp.sin(t4)],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            ])
        
        A04 = A01*A12*A23*A34
        FK_matrix = sp.lambdify([t1, t2, t3, t4],A04,'numpy')

        return FK_matrix

    def calculate_IK_vector(self,l):
        px, py, pz, phi = sp.symbols('px, py, pz, phi')
        
        A = pz-l[0]-l[3]*sp.sin(phi)
        B = sp.sqrt(px**2+py**2)-l[3]*sp.cos(phi)
        C = (A**2+B**2+l[1]**2-l[2]**2)/(2*l[1])

        ft1 = sp.atan2(py,px)
        ft2 = 2*sp.atan2(A+sp.sqrt(A**2+B**2-C**2),B+C)     # Normally this angle has 2 possible solutions. upper and lower shoulder but we see that the lower shoulder solution does not work so we only put the upper shoulder solution. see jupyter file for referance
        ft3 = sp.asin((A-l[1]*sp.sin(ft2))/l[2])-ft2        
        ft4 = phi-ft3-ft2

        t1=sp.lambdify([px,py,pz,phi],ft1,'numpy')
        t2=sp.lambdify([px,py,pz,phi],ft2,'numpy')
        t3=sp.lambdify([px,py,pz,phi],ft3,'numpy')
        t4=sp.lambdify([px,py,pz,phi],ft4,'numpy')

        t=[t1, t2, t3, t4]
        return t
    
    def set_joint_position_deg(self, t):

        # update robot state variables
        t_rad=np.deg2rad(t)
        cartesian_state=self.get_cartesian_state(t_rad)
        
        self.JPOS=t_rad
        self.POS = cartesian_state['position']
        self.ROT = cartesian_state['rotation']

        return t_rad
    
    def get_cartesian_state(self,t):
        At = self.FK(t[0],t[1], t[2], t[3])
        pos = np.array([At[0:3, 3]])
        rot = np.array([At[0:3, 0:3]])

        return {'position':pos, 'rotation':rot}

    def set_cartesian_position(self, p):

        try:
            t1= self.IK[0](p[0], p[1], p[2], p[3])
            t2= self.IK[1](p[0], p[1], p[2], p[3])
            t3= self.IK[2](p[0], p[1], p[2], p[3])
            t4= self.IK[3](p[0], p[1], p[2], p[3])
        except RuntimeWarning:
            rospy.logwarn("impossible value given to inverse kinematics calculation.")
            rospy.logwarn("movement not executed")
            return
        except Exception as e:
            rospy.logerr("error in IK function:"+ e)
            rospy.logerr("movement not executed")
        else:
            t=[t1, t2, t3, t4]

            # calculate & update robot state variables 
            cartesian_state = self.get_cartesian_state(t)
            self.JPOS = np.array([t1, t2, t3, t4])
            self.POS = cartesian_state['position']
            self.ROT = cartesian_state['rotation']

        return t 
