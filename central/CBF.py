import random
import math
import numpy as np

from Onstage_Master import Point
from Onstage_Master import robot

def get_velocity(robot, obstacles, alpha=1.0, speed=0.10, obs_clearance=0.01, detect_clearance=0.10):
        #nominal controller
        v_nom = 1.0
        dir = np.array(robot.target.x, robot.target.y) - np.array(robot.coords.x, robot.coords.y)
        dist = np.linalg.norm(dir)
        
        if dist > 1e-6:
            v_nom = speed * dir/dist
        else:
            v_nom = np.zeros(2)
        
        #CBF constraints
        A = []
        B = []
        
        for obs in obstacles:
            dx = robot.coords.x-obs[0]
            dy = robot.coords.y - obs[1]
            dist_obs = sqrt(dx**2 + dy**2)
            
            safe_buffer = obs[2]+detect_clearance
            if dist_obs < safe_buffer:
                dh_dx = dx / dist_obs
                dh_dy = dy / dist_obs
                h = dist_obs-obs[2] - obs_clearance
                
                A.append([-dh_dx, -dh_dy])
                B.append(alpha * h)
            
            A = np.array(A)
            B = np.array(B)
            
        u = cp.Variable(2)
        
        objective = cp.Minimize(cp.sum_squares(u - v_nom))
        constraints = []
        
        if len(A) > 0:
            constraints.append(A @ u <= B)
            
        v_max = 0.05
        constraints += [
            u[0] >= -v_max,
            u[0] <= v_max,
            u[1] >= -v_max,
            u[1] <= v_ma
            
        ]
        
        prob = cp.Problem(objective, constraints)
        prob.solve(solver=cp.OSQP, warm_start = True)
        
        if u is None:
            velocity = v_nom
        else:
            velocity = u.value
        
        return velocity

        
        
                
                


