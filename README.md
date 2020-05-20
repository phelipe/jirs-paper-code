# A General Framework for Optimal Tuning of PID-like Controllers for Minimum Jerk Robotic Trajectories

This repository contains code associated with the paper 
["A General Framework for Optimal Tuning of PID-like Controllers for Minimum Jerk Robotic Trajectories"](https://link.springer.com/article/10.1007/s10846-019-01121-y), submitted to JIRS. You need [Julia Language](https://julialang.org/) to run this project.



# Packages

  - DiffEqCallbacks v2.13.0 ⚲
  - IJulia v1.21.2
  - PerformanceIndex v0.1.0 #master (https://github.com/phelipe/PerformanceIndex.jl)
  - Plots v1.3.1
  - RigidBodyDynamics v2.2.0
  - RigidBodySim v1.3.0
  - SpecialFunctions v0.10.2
  - StaticArrays v0.12.3
  - Swarm v0.0.0 #master (https://github.com/phelipe/Swarm.jl)
  - Trajectory v0.1.0 #master (https://github.com/phelipe/Trajectory.jl)
  - LinearAlgebra 
  - Statistics 



# Setup
Inside project folder run:
```
julia startup.jl
```
this will install  packages and start jupyter.


# Notebooks
  - [Optimization](1-optimization.ipynb)
  - [Optimization Fractional Control](2-optimization-frac.ipynb)
  - [Results](3-results.ipynb)
  - [Results with disturbance](4-results-dist.ipynb)
  - [Results with model modification](5-results-load.ipynb)
