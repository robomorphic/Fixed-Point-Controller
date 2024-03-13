// This file will consist on glfw and mouse functions that are not crucial for robot control
// TODO: Delete unnecessary includes
#include <typeinfo>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>
#include <time.h> 

#include <osqp/osqp.h>

#include <GLFW/glfw3.h>

#include <mujoco/mujoco.h>

#include "pinocchio/parsers/urdf.hpp"
 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio_plus/aba.hpp"

#include <FixedPoint/fixed_point.hpp>

#include <config.hpp>

#include <Eigen/Core>



template<typename BinaryOp>
struct Eigen::ScalarBinaryOpTraits<double,exp_type,BinaryOp> { typedef exp_type ReturnType;  };
template<typename BinaryOp>
struct Eigen::ScalarBinaryOpTraits<exp_type,double,BinaryOp> { typedef exp_type ReturnType;  };



// Load the urdf model
pinocchio::ModelTpl<double> pinocchio_model_basic;
pinocchio::ModelTpl<exp_type> pinocchio_model;
pinocchio::ModelTpl<exp_type_gravity> pinocchio_model_gravity;
pinocchio::ModelTpl<exp_type_fd> pinocchio_model_fd;
// Create data required by the algorithms
pinocchio::DataTpl<exp_type> pinocchio_data;
pinocchio::DataTpl<exp_type_gravity> pinocchio_data_gravity;
pinocchio::DataTpl<exp_type_fd> pinocchio_data_fd;


// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

MJAPI extern mjfGeneric mjcb_control;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

