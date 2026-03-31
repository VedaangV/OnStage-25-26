import numpy as np
import matplotlib.pyplot as plt
import time

### add goal object to field ###
def add_goal (X, Y, s, r, delx, dely, loc, gridsize):
  gridsize = gridsize
    
  for i in range(len(X)):
    for j in range(len(Y)):
      d= np.sqrt((loc[0]-X[i][j])**2 + (loc[1]-Y[i][j])**2)
      #print(f"{i} and {j}")
      theta = np.arctan2(loc[1]-Y[i][j], loc[0] - X[i][j])
      if d< r:
        delx[i][j] = 0
        dely[i][j] = 0
      elif d>r+s:
        delx[i][j] = gridsize * s *np.cos(theta)
        dely[i][j] = gridsize * s *np.sin(theta)
      else:
        delx[i][j] = gridsize * (d-r) *np.cos(theta)
        dely[i][j] = gridsize * (d-r) *np.sin(theta)
  return delx, dely

### add obstacle object to field ###
def add_obstacle(X, Y, s, obs, delx, dely, goal, gridsize):
  r = obs.radius
  obstacle = [obs.coords.x, obs.coords.y]
  
  for i in range(len(X)):
    for j in range(len(Y)):
      d_goal = np.sqrt((goal[0]-X[i][j])**2 + ((goal[1]-Y[i][j]))**2)
      d_obstacle = np.sqrt((obstacle[0]-X[i][j])**2 + (obstacle[1]-Y[i][j])**2)
      #print(f"{i} and {j}")
      theta_goal= np.arctan2(goal[1] - Y[i][j], goal[0]  - X[i][j])
      theta_obstacle = np.arctan2(obstacle[1] - Y[i][j], obstacle[0]  - X[i][j])
      if d_obstacle < r:
        delx[i][j] = -1*np.sign(np.cos(theta_obstacle))*5 +0
        dely[i][j] = -1*np.sign(np.cos(theta_obstacle))*5  +0
      elif d_obstacle>r+s:
        delx[i][j] += 0 -(gridsize * s *np.cos(theta_goal))
        dely[i][j] += 0 - (gridsize * s *np.sin(theta_goal))
      elif d_obstacle<r+s :
        delx[i][j] += -gridsize*3 *(s+r-d_obstacle)* np.cos(theta_obstacle) #-150
        dely[i][j] += -gridsize*3 * (s+r-d_obstacle)*  np.sin(theta_obstacle) #-150 
      if d_goal <r+s:
        if delx[i][j] != 0:
          delx[i][j]  += (gridsize * (d_goal-r) *np.cos(theta_goal))
          dely[i][j]  += (gridsize * (d_goal-r) *np.sin(theta_goal))
        else:
          
          delx[i][j]  = (gridsize * (d_goal-r) *np.cos(theta_goal))
          dely[i][j]  = (gridsize * (d_goal-r) *np.sin(theta_goal))
          
      if d_goal>r+s:
        if delx[i][j] != 0:
          delx[i][j] += gridsize* s *np.cos(theta_goal)
          dely[i][j] += gridsize* s *np.sin(theta_goal)
        else:
          
          delx[i][j] = gridsize* s *np.cos(theta_goal)
          dely[i][j] = gridsize* s *np.sin(theta_goal) 
      if d_goal<r:
          delx[i][j] = 0
          dely[i][j] = 0
   
  return delx, dely, obstacle, r

### plot axis elements ###
def plot_graph(X, Y, delx, dely, obj, fig, ax, loc,r,i, color,start_goal=np.array([[0,0]]) ):
  ax.quiver(X, Y, delx, dely)
  ax.add_patch(plt.Circle(loc, r, color=color))
  ax.set_title(f'Robot path with {i} obstacles ')
  ax.annotate(obj, xy=loc, fontsize=10, ha="center")
  return ax

### find path based on potential field ###
def pfield_path(robot, obstacles, field_size):
    gridsize = field_size
    fig, ax = plt.subplots(figsize = (gridsize,gridsize))
    
    ### intialization ###
    x = np.arange(-0,gridsize,1)
    y = np.arange(-0,gridsize,1)

    goal = [robot.target.x, robot.target.y] #goal=robot.target
    
    seek_points = np.array([[robot.coords.x, robot.coords.y]]) #start point
    X, Y = np.meshgrid(x,y) #grid x, y
    delx = np.zeros_like(X) #slope field x
    dely = np.zeros_like(Y) #slope field y 
    
    s = 5 #pull/push strength, tune value
    r = 1.5 #radius/size, tune value
    delx, dely = add_goal(X, Y, s, r, delx, dely, goal, gridsize) #add goal to slope field
    plot_graph(X, Y, delx, dely , 'Goal', fig, ax, goal, r, 0, 'b')
    for idx, obs in enumerate(obstacles):
      s = 5 #
      delx, dely, loc, r = add_obstacle(X, Y, s, obs, delx, dely, goal, gridsize)
      plot_graph(X, Y, delx, dely, 'Obstacle', fig, ax, loc, r, idx+1,'m') #
    
    plot_graph(X, Y, delx, dely , '', fig, ax, seek_points[0], 1, 0, 'r')
    
    stream = ax.streamplot(X,Y,delx,dely, start_points=seek_points,linewidth=4, cmap='autu')
    segments = stream.lines.get_segments()
    points = [[robot.coords.x, robot.coords.y]]
    
    point = [0, 0]
    
    start = 0
    for idx in range(len(segments)):
        if ((segments[idx][0][0] == robot.coords.x) and (segments[idx][0][1] == robot.coords.y)):
            start = idx
            break
    start = start + 1
    for idx in range(len(segments) - start + 1):
        point = [segments[start + idx][0][0], segments[start + idx][0][1]]
        points.append(point)
        
    plt.show()
    plt.close()
    return points

# ### testing ###
# class Point:
#     def __init__(self, x, y):
#         self.x = x
#         self.y = y
# 
#     def distance_to(self, other_point):
#         dx = self.x - other_point.x
#         dy = self.y - other_point.y
#         return math.sqrt(dx**2 + dy**2)
# 
#     def __str__(self):
#         return f"Point({self.x}, {self.y})"
#     
# class robot:
#     def __init__(self, x, y):
#         self.coords = Point(x, y)
#         self.target = Point(-1, -1)
#     def setTarget(self, target):
#         self.target = target
# 
# class obstacle:
#     def __init__(self, x, y, radius):
#         self.coords = Point(x, y)
#         self.radius = radius
#         
# robots = [robot(10, 10)]
# robots[0].setTarget(Point(60, 70))
# obstacles = [obstacle(35.6, 37.6, 4), obstacle(47.25, 41.45, 5), obstacle(21.34, 31.34, 2.5)]
#         
# if __name__ == "__main__":
#     pfield_path(robots[0], obstacles, 80)
