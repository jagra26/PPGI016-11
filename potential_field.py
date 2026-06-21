import numpy as np

zeta = 2.0
d_star = 0.25
eta = 10.0
Q_star = 0.5
dt = 0.01
goal = np.array([1.7, 1.7])
obstacles = np.array([[-2, -1.7], [-1, 0], [0.5, 0.25],[1, 1], 
                      [2.3, 1.4], [-1.7, -1.5], [0,-1], [-2, 1],
                      [-0.8, -2], #use [-1, -2] para mínimo local
                      [0, 1.6]])
def attractiveForce(pose, goalp):
    diff = pose - goalp
    dist = np.linalg.norm(diff)
    if dist <= d_star:
        return -zeta * diff #conica
    else:
        return -(diff * (d_star*zeta)/(dist)) #parabolica

def repulsiveForce(pose, obstacles):
    force = np.zeros(2)
    for obs in obstacles:
        diff = pose - obs
        dist = np.linalg.norm(diff)
        if dist < Q_star:
            repulsive_force = eta * ((1/dist) - (1/Q_star)) * (1/(dist**2)) * (diff/dist)
            force += repulsive_force
    return force

def total_force(pose, goalp, obstacles):
	return attractiveForce(pose, goalp) + repulsiveForce(pose, obstacles)

def sysCall_init():
    sim = require('sim')
    self.robot_handle = sim.getObject("..") #handle pra base do rob?
    self.pose = sim.getObjectPosition(self.robot_handle, -1)
    self.robot_position = [self.pose[0], self.pose[1]]


def sysCall_actuation():
    force = total_force(self.robot_position, goal, obstacles)
    self.robot_position = self.robot_position + force * dt
    sim.setObjectPosition(self.robot_handle, [self.robot_position[0], self.robot_position[1], self.pose[2]])
    

def sysCall_sensing():
    # put your sensing code here
    pass

def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details
