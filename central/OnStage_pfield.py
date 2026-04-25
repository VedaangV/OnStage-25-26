import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import time

PLOT_TYPE = "BOUNDARY_PLOT" # "RADIUS_PLOT" "BOUNDARY_PLOT"

mpl.rcParams['path.simplify'] = True
mpl.rcParams['path.simplify_threshold'] = 1.0
mpl.rcParams['agg.path.chunksize'] = 10000

### add goal object to field ###
def add_goal (X, Y, s, r, delx, dely, loc, gridsize):
  gridsize = gridsize
  
  d = np.sqrt((loc[0]-X)**2 + (loc[1]-Y)**2)
  theta = np.arctan2(loc[1]-Y, loc[0]-X)
  
  conds = [d< r, d>r+s]
  xchoices = [0, gridsize * s *np.cos(theta)]
  ychoices = [0, gridsize * s *np.sin(theta)]
  
  delx = np.select(conds, xchoices, default = gridsize * (d-r) *np.cos(theta))
  dely = np.select(conds, ychoices, default = gridsize * (d-r) *np.sin(theta))
        
  return delx, dely

### add obstacle object to field ###
def add_obstacle(X, Y, s, r, delx, dely, loc, goal, gridsize):
  obstacle = [loc[0], loc[1]]
  
  d_goal = np.sqrt((obstacle[0] - X)**2 + (obstacle[1] - Y)**2)
  d_obstacle = np.sqrt((obstacle[0] - X)**2 + (obstacle[1] - Y)**2)
                       
  theta_goal = np.arctan2(goal[1] - Y, goal[0]  - X)
  theta_obstacle = np.arctan2(obstacle[1] - Y, obstacle[0]  - X)

  conds = [d_obstacle < r, d_obstacle>r+s, d_obstacle<r+s]
  xchoices = [-1*np.sign(np.cos(theta_obstacle))*5, delx + 0 -(gridsize * s *np.cos(theta_goal)), delx + (-gridsize*3 *(s+r-d_obstacle)* np.cos(theta_obstacle))]
  ychoices = [-1*np.sign(np.cos(theta_obstacle))*5, dely + 0 - (gridsize * s *np.sin(theta_goal)), dely + (-gridsize*3 * (s+r-d_obstacle)*  np.sin(theta_obstacle))]
  
  delx = np.select(conds, xchoices, default = delx)
  dely = np.select(conds, ychoices, default = dely)
  
  delx = np.where(d_goal <r+s, delx + (gridsize * (d_goal-r) *np.cos(theta_goal)), delx)
  dely = np.where(d_goal <r+s, dely + (gridsize * (d_goal-r) *np.sin(theta_goal)), dely)
  
  delx = np.where(d_goal>r+s, delx + (gridsize* s *np.cos(theta_goal)), delx)
  dely = np.where(d_goal>r+s, dely + (gridsize* s *np.sin(theta_goal)), dely)
  
  delx = np.where(d_goal<r, 0, delx)
  dely = np.where(d_goal<r, 0, dely)
   
  return delx, dely, obstacle, r

### find path based on potential field ###
def pfield_path(robot, obstacles, field_size):
    gridsize = field_size
    fig, ax = plt.subplots(figsize = (gridsize,gridsize))
    
    ### intialization ###
    x = np.arange(-0,gridsize,1)
    y = np.arange(-0,gridsize,1)

    goal = [robot.target.coords.x, robot.target.coords.y] #goal=robot.target
    
    seek_points = np.array([[robot.coords.x, robot.coords.y]]) #start point
    X, Y = np.meshgrid(x,y) #grid x, y
    delx = np.zeros_like(X) #slope field x
    dely = np.zeros_like(Y) #slope field y 
    
    if PLOT_TYPE == "RADIUS_PLOT":
        s = 2 #pull/push strength, tune value
    if PLOT_TYPE == "BOUNDARY_PLOT":
        s = 4
    r = 1.5 #radius/size, tune value
    delx, dely = add_goal(X, Y, s, r, delx, dely, goal, gridsize) #add goal to slope field
    ax.add_patch(plt.Circle(goal, r, color='b'))
    for idx, obs in enumerate(obstacles):
      if PLOT_TYPE == "RADIUS_PLOT":
          if obs.coords.x < 0 or obs.coords.x > field_size or obs.coords.y < 0 or obs.coords.y > field_size:
              continue
          s = 6
          r = obs.radius
          loc = [obs.coords.x, obs.coords.y]
          delx, dely, loc, r = add_obstacle(X, Y, s, r, delx, dely, loc, goal, gridsize)
          ax.add_patch(plt.Circle(loc, r, color='m')) #
      if PLOT_TYPE == "BOUNDARY_PLOT":
        for i, pt in enumerate(obs.border):
          if pt[0] < 0 or pt[0] > field_size or pt[1] < 0 or pt[1] > field_size:
            continue
          s = 6
          r = 1
          loc = pt
          delx, dely, loc, r = add_obstacle(X, Y, s, r, delx, dely, loc, goal, gridsize)
          print(f"Finished #{i}")
          ax.add_patch(plt.Circle(loc, r, color='m')) #
        #ax.add_patch(plt.Polygon(obs.border, facecolor='m')) #
    
    ax.add_patch(plt.Circle(seek_points[0], 1, color='r'))
    ax.quiver(X, Y, delx, dely)
    stream = ax.streamplot(X,Y,delx,dely, density=0.5, start_points=seek_points,linewidth=4, cmap='autu')
    segments = stream.lines.get_segments()
    points = [[robot.coords.x, robot.coords.y]]
    
    point = [0, 0]
    
    start = 0
    for idx in range(len(segments)):
        if ((segments[idx][0][0] == robot.coords.x) and (segments[idx][0][1] == robot.coords.y)):
            start = idx
            break
    for idx in range(len(segments) - start):
        point = [segments[start + idx][0][0], segments[start + idx][0][1]]
        points.append(point)
    
    plt.show()
    plt.close()
    return points
