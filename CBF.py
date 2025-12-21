import random
import math
import numpy as np
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def distance_to(self, other_point):
        dx = self.x - other_point.x
        dy = self.y - other_point.y
        return math.sqrt(dx**2 + dy**2)

    def __str__(self):
        return f"Point({self.x}, {self.y})"
   
   
class robot:
    def __init__(self, IP, port, x, y, tag):
        self.IP = IP;
        self.port = port
        self.x = x
        self.y = x
        self.water_level = 1
        self.state = "plant"
        self.tag = tag;
    def updatePosition(self, x, y):
        self.x = x
        self.y=y
    def setTarget(x, y):
        self.targetX = x
        self.targetY = y
    def depleteWater():
        self.water_level -= 1
    def restoreWater():
        self.water_level += 1
    def atTarget():
        return (math.sqrt((self.x-self.targetX)**2 + (self.y-self.targetY)**2) < 0.5)
    
    def changeState():
        if(atTarget):
            self.state = states[(states.index(self.state)+1)%len(states)]
            
        


def drive(bot, target, obstacles, alpha=1.0, speed=0.10, obs_clearance=0.01, detect_clearance=0.10):
            #nominal controller
        v_nom = 1.0
        dir = target - np.array(bot.x, bot.y)
        dist = np.linalg.norm(dir)
        
        
        if dist > 1e-6:
            v_nom = speed * dir/dist
        else:
            v_nom = np.zeros(2)
        
        #CBF constraints
        A = []
        B = []
        
        for obs in obstacles:
            dx = bot.x-obs[0]
            dy = bot.y - obs[1]
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

        
        
                
                


