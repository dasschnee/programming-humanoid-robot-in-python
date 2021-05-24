'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np
from math import atan2, sqrt, pi, pow
from sets import Set
from numpy import inner, dot 

EPSILON = 1e-3

class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics
        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        
        effector_chain = self.chains[effector_name]
        joints = {key : self.perception.joint[key] for key in effector_chain}
        last_effector = effector_chain[-1]
        
        Te = self.get_joint_values(transform)
        theta = np.zeros(len(effector_chain)) #initial return value

        for i in range(1000): #break loop if angles are satisfiable
            # refresh angles
            self.forward_kinematics(joints)
            # get new transforms
            T = self.transforms 
            #calculate distance between target and actual result
            error = self.get_error(Te, T, last_effector) 
            
            jacobian_matrix = self.get_jacobian_matrix(Te, T, effector_chain)
            
            alpha = self.get_jacobian_scalar(jacobian_matrix, error) 
            
            d_theta = self.jacobian_transpose(alpha, jacobian_matrix, error)
            #update theta value
            theta += d_theta 

            self.refresh_joints(joints, theta)

            if inner(error, error) < EPSILON:
                break
            
        return theta

    def get_error(self, Te, T, last_effector):
        return Te - self.get_joint_values(T[last_effector])

    def refresh_joints(self, joints, theta):
        for i, key in enumerate(joints.keys()):
            joints[key] = theta[i]
     
    def jacobian_transpose(self, alpha, jacobian_matrix, error):
        return alpha * dot(jacobian_matrix.T, error)

    def get_jacobian_scalar(self, jacobian_matrix, error):
        JJTe = dot(dot(jacobian_matrix, jacobian_matrix.T), error)
        return float(inner(error, JJTe)) / float(inner(JJTe, JJTe))

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        effector_chain = self.chains[effector_name]
        self.forward_kinematics(self.perception.joint)
        angles = self.inverse_kinematics(effector_name, transform)

        names = [joint for joint in effector_chain]
        times = [[1.0, 3.0]] * len(effector_chain)
        keys = [[[angle - 0.01, [3, 0, 0], [3, 0, 0]], [angle, [3, 0, 0], [3, 0, 0]]] for angle in angles]
        self.keyframes = (names, times, keys)
        
    def get_jacobian_matrix(self, Te, T, effector_chain):
        jacobi_matrix = np.zeros((6, len(effector_chain)))
        for i, t in enumerate(T.values()):
            x, y, z = self.get_joint_coordinates(t)
            jacobi_matrix[:i] = (Te - np.array([x, y, z, 0, 0, 0]))
        return self.set_joint_axis(jacobi_matrix, effector_chain)
        
    def get_joint_coordinates(self, transform):
        return transform[-1,0] ,transform[-1,1] ,transform[-1,2]
            
    def get_joint_values(self, transform):    
        x, y, z = self.get_joint_coordinates(transform)
        
        if(transform[2,0] > 1e-3):
            omega_x = atan2(transform[2,1], transform[2,2])
            omega_y = atan2(-transform[2,0], sqrt(pow(transform[2,1], 2) + pow(transform[2,2],2)))
            omega_z = atan2(transform[1,0], transform[0,0])
        else:
            omega_z = 0
            if(abs(transform[2,0] + 1) < 1e-3):
                omega_x = atan2(transform[0,1], transform[0,2])
                omega_y = pi / 2
            else:
                omega_x = atan2(-transform[0,1], -transform[0,2])
                omega_y = -pi / 2
        return np.array([x, y, z, omega_x, omega_y, omega_z])
    
    def set_joint_axis(self, jacobi, effector_chain):
        for index, joint_name in enumerate(effector_chain):
            angle_index = 0
            if 'Roll' in joint_name: #x
                angle_index = 3
            elif 'Pitch' in joint_name: #y
                angle_index = 4
            else: #z
                angle_index = 5
            jacobi[angle_index, index] = 1
        return jacobi
    
if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('RLeg', T)
    agent.run()
