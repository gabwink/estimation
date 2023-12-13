# estimation

The implementation of a particle filter for a differenetial drive vehicle subject to the dynamics, 

$\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} cos(\theta)u_1 \\ sin(\theta)u_1  \\ u_2  \end{bmatrix}$ , $(x(0), y(0), \theta(0)) = (0, 0, \pi/2)$


using dt = 0.1 and process noise and measurement noise normally distributed with a variance of
0.02.  



Figure 24: The particles every 1 second for a total of 6 seconds. The black line is
the state estimate of x and y every dt seconds.




_________________________
Implement the Kalman filter for the system

$\phantom{ccccccccccccccccccccccccccccccccccccccccccc}\begin{bmatrix} \dot{x} \\ \dot{y} \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ -1 & 0 \end{bmatrix} \begin{bmatrix} x \\ y \end{bmatrix}$



with initial condition [1, 1], dt = 0.1, measurements of x, and normally distributed process
and measurement noise with variance of 0.1. Turn In: The evolution of the state prediction
and the covariance of the state prediction for 1 second.
1. For problem 2, numerically demonstrate that the Kalman filter is indeed the optimal linear
filter by simulating the the optimal filter and ten ‘nearby’ filters of your choosing for 100
sample paths and looking at the average error for each filter. Turn In: The plot of error
versus linear filter, indicating which one is the optimal Kalman filter.


__________


# Particle Filter

## Code Overview 
A particle filter implemented for a differenetial drive vehicle subject to the dynamics, 

$\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} cos(\theta)u_1 \\ sin(\theta)u_1  \\ u_2  \end{bmatrix}$ , $(x(0), y(0), \theta(0)) = (0, 0, \pi/2)$


using dt = 0.1 and process noise and measurement noise normally distributed with a variance of 0.02.  

Model: $\dot{x} = f(x, u + noise)$

## Algorithm 

$X_t = []$  
$\bar{X_t} = []$  
**for** m = 1, ..., M **do**  
* sample $x^{[m]}_t$ from $p(x_t|u_t,x^{[m]}_{t-1})$
* $w^{[m]}_t = p(z_t|x^{[m]}_t)$
* add the pair $(x^{[m]}_t, w^{[m]}_t)$ to $\bar{X_t}$  

**end for**  
**for** m = 1, ..., M **do**  
* draw *i* with probability $w^{[m]}_t$ 
* add $x^{[i]}_t$ to $X_t$ 

**end for**  
Return $X_t$  

## Algorithm Explained 
$u_t$ is the control; $x_t$ is the state; $z_t$ is the measurement; *M* is the number of particles

Loop 1:  
* sample $x^{[m]}_t$ from $p(x_t|u_t,x^{[m]}_{t-1})$ **&rarr;**  take one particle and propgate it forward in time (using the dynamic model + noise) to get a new particle  $x^{[m]}_t$
* $w^{[m]}_t = p(z_t|x^{[m]}_t)$ **&rarr;** reassign weight top particle based on the measurement aquired 
* update 
  
This loop moves all the particles forward in time and assigns them weights according to the measurement aquired


Loop 2 (resampling):
* draw *i* with probability $w^{[m]}_t$ 
	* $\tilde{w}^{[m]}_t=\frac{w^{[m]}_t}{\sum{w^{[i]}_t}}$
* then add the particle you draw to $X_t$ 


Works to fix particle deprivation. This resampling will give me multiple copies of particles with high weights (this is ok because you apply you noise model to them). 

## What is a particle  filter

Particle filters are a discrete approximation of a Bayes fitler. They are approximating the intwergral in a continuous domain. They are rwally good for systems where your sensor models are primarily defined by their geometry rather than being defined by a noise distribution. 




Particle Filters are a way of using samples from a distribution to model the impact of nontriv-
ial/nonlinear transition functions (typically arising from physics or some other principled modeling
approach).