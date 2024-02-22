## Robot controller with Fixed-Point Arithmetic

### Introduction
This project is built for analyzing how fixed-point arithmetic can be used to control a robot. The control problem is modelled as a quadratic problem, and it is solved using [OSQP](https://osqp.org/). The problem formulation can be seen in Notes.pdf file.

### Dependencies
Essential:
- [OSQP](https://osqp.org/)
- [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html)
- [CMake](https://cmake.org/)
- [FixedPoint](https://github.com/alpylmz/FixedPoint)
- [MuJoCo](http://www.mujoco.org/)
- [Pinocchio](https://stack-of-tasks.github.io/pinocchio/)
- [GLFW](https://www.glfw.org/)
- [Boost](https://www.boost.org/)

Optional:
- [Python](https://www.python.org/)
- [Matplotlib](https://matplotlib.org/)
- [Numpy](https://numpy.org/)

### Installation
In progress, I'll prepare a Docker environment.

#### Extra notes
Need to change the line 

> /opt/homebrew/include/eigen3/Eigen/Core/MathFunctions.h line 1372
template<typename T> EIGEN_DEVICE_FUNC bool (isfinite)(const T &x) { return internal::isfinite_impl(x); }

to
> template<typename T> EIGEN_DEVICE_FUNC bool (isfinite)(const T &x) { return true; }
Otherwise, Eigen refuses to work with our fixed-point arithmetic class. This may be fixable.

### Pointers
Start by reading src/trajectory_tracking.cc. It contains the crucial parts and the main function. The rest(MuJoCo helper functions, renders, trajectories, config files) can be found in include/config.hpp include/mujoco_exec_helper.hpp include/traj.hpp. 
It should be easy to read and well-commented, but I'll simplify and write about them here after I am done with installation part.
