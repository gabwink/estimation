import numpy as np
import matplotlib.pyplot as plt
from numpy.random import uniform, randn
import random 

# Givens based on prject description 
variance = 0.02                 # process noise and measurement nouise normal with variance = 0.02
sigma = np.sqrt(variance)
T = 6                           # period, timestep, number of time steps
dt = 0.1
N = int(T / dt)
n = 3                           # x, y, theta 3dof  
s0 = np.array([0, 0, np.pi/2])  # initial state 
u = np.array([1, -1/2])         # control 


# functions for process noise and measurement noise
def create_normal_distribution(mean, std, N):
    '''
    Generates normal distribution
    Use to generate initial xt
    '''
    particles = np.empty((3, N))
    particles[0, :] = mean[0] + (randn(N) * std)
    particles[1, :] = mean[1] + (randn(N) * std)
    particles[2, :] = mean[2] + (randn(N) * std)
    return particles

def noise_distribution(mean, std):
    '''
    Generates normal distribution 
    '''
    noise = np.zeros(2)
    N = 1
    noise[0] = mean[0] + (randn(N) * std)
    noise[1] = mean[1] + (randn(N) * std)
    return noise


def weight_distribution(mean, std):
    '''
    Generates normal distribution 
    '''
    N = 1
    return mean + (randn(N) * std)

# dynamics 
def propagate_fwd(particle, u):
    """
    Propagate particle forward + process noise = new particle.
    """
    s = np.copy(particle)
    s_bar = rk4(s, u)
    return s_bar


def rk4(xt, u):
    """
    Runge-Kutta 4th order integration.
    """
    k1 = dt * dynamics(xt, u)   # note u is a constant control input 
    k2 = dt * dynamics(xt + (k1 / 2.), u)
    k3 = dt * dynamics(xt + (k2 / 2.), u)
    k4 = dt * dynamics(xt + k3, u)
    return xt + (1/6.) * (k1 + 2.0*k2 + 2.0*k3 + k4)


def dynamics(particle, u):
    """
    x_dot = f(x, u + noise).
    """
    s = particle
    dsdt = np.array([np.cos(s[2]) * u[0], np.sin(s[2]) * u[0], u[1]])
    return dsdt


def simulate_trajectory(initial_state, control_input):
    """
    Forward simulate initial trajectory given dynamics, initial condition, and timescale.
    """
    traj = np.zeros((n, N+1))
    traj[:, 0] = initial_state
    xt = initial_state

    for i in range(1, N+1):
        traj[:, i] = propagate_fwd(xt, control_input)
        xt = traj[:, i]

    return traj

# particle filter 
def particle_filter(xt, M, timestep):
    '''
    xt : array of M particles (3 X M)
    M  : number of particles
    timestep : current time
    '''
    index_list = [i for i in range(M)]
    xt = np.copy(xt)            # distribution that descirbes current belief            
    xt_bar = np.zeros((3+1,M))  # update samples of distribution - update = integrator scheme + noise

    wt = np.zeros(M) 
    error = np.zeros(M)
    inv_error = np.zeros(M)
    
    # Stage 1A - Move particles to next time step, take ONE measurement, assign weights based on measurement 
    measurement_noise = create_normal_distribution([0,0,0], sigma, 1)
    measurement = initial_trajectory[ : , timestep + 1] + measurement_noise
    
    for m in range(M):
        # Step 1: propogate particle with noise 
        noise = noise_distribution([0,0], sigma)
        u_noise = np.add(u, noise)
        particle = propagate_fwd( xt[:, m], u_noise)

        # Step 2: compute weight of particle based on your measurement
        error = np.linalg.norm(measurement - particle)   
        inv_error[m] = 1/error
        
        # add propagated particle
        xt_bar[0:3, m] = particle.T
    
    # Stage 1B:  add weight to xt_bar   
    sum_inv_error = np.sum(inv_error)
    for m in range(M):
        w = inv_error[m]/sum_inv_error     # particle with smallest error will have highest weight
        xt_bar[3 , m]  = w

    # Stage 2 - resampling (fix particle depression and favor particles with higher weight)
    for m in range(M):

        # Step 1: draw i with probability wt[m] .... weight effects likeliness that you will select the particle 
        weight = xt_bar[3, :]
        i = random.choices(index_list, weight)
        xi = xt_bar[0:3, i]

        # Step 2: add xi to Xt
        xt[:, m] = xi.T
    
    # return distribution at next timestep 
    return xt



# Given the vehicles initial state and dynamics forward simulate and plot initial trajectory
initial_trajectory = simulate_trajectory(s0, u)
M = 1000                                         # number of particles often 1000
xt = create_normal_distribution(s0, sigma, M)    # Initial belief normal distribution of variance 0.02 around IC

# generate particles at each timestep and plot the particles every 1 second for a total of 6 seconds
for i in range(N):
    new_xt = particle_filter(xt, M, i)
    xt = new_xt
    
    # average state estimate 
    avg = np.mean(xt, axis= 1)
    plt.scatter(avg[0], avg[1], color = "black", zorder = 100)
    
    if i%5 == 0:
        print(i)
        plt.scatter(xt[0], xt[1], edgecolors='black', alpha = 0.5)

    
plt.scatter(xt[0], xt[1], edgecolors='black', alpha = 0.5)
plt.xlabel("x position")
plt.ylabel("y position")
plt.show()
