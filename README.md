## Robot controller with Fixed-Point Arithmetic

### Introduction
This project is built for analyzing how fixed-point arithmetic can be used to control a robot. The control problem is modelled as a quadratic problem, and it is solved using [OSQP](https://osqp.org/). The problem formulation can be seen in [Notes](Notes.pdf) file.

### Dependencies
Essential:
- [CMake](https://cmake.org/)
- [GLFW](https://www.glfw.org/)
- [Boost](https://www.boost.org/)
- [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html)
- [OSQP](https://osqp.org/)
- [FixedPoint](https://github.com/alpylmz/FixedPoint)
- [MuJoCo](http://www.mujoco.org/)
- [MuJoCo Menagerie](https://github.com/alpylmz/mujoco_menagerie)
- [Pinocchio](https://stack-of-tasks.github.io/pinocchio/)

Optional:
- [Matplotlib](https://matplotlib.org/)

### Installation
You can use the Dockerfile to install and its dependencies. You may also need to setup X11 forwarding in the host system. In the future I'll try to remove this dependency for headless operation.


#### Extra notes
You need to change the line 
> /opt/homebrew/include/eigen3/Eigen/Core/MathFunctions.h line 1372

from
> template<typename T> EIGEN_DEVICE_FUNC bool (isfinite)(const T &x) { return internal::isfinite_impl(x); }

to
> template<typename T> EIGEN_DEVICE_FUNC bool (isfinite)(const T &x) { return true; }

Otherwise, Eigen refuses to work with our fixed-point arithmetic class. This may be fixable.

### Usage
Currently the project is set up for Franka Panda robot, it can be run with the following command:
```bash
./scripts.sh panda
```

If you'd like to sweep different fixed-point configurations, you can use the following command:
```bash
./scripts.sh panda_sweep
```

### Pointers
Start by reading [trajectory_tracking.cc](src/trajectory_tracking.cc) and [config.hpp](include/config.hpp). They contain the crucial parts, configs, and the main function.
