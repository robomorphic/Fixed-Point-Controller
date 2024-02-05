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

#include <FixedPoint/fixed_point.hpp>
#include <Eigen/Core>

//typedef FixedPoint<8,8> exp_type;
typedef double exp_type;

// Load the urdf model
pinocchio::ModelTpl<double> pinocchio_model_basic;
pinocchio::ModelTpl<exp_type> pinocchio_model;
// Create data required by the algorithms
pinocchio::DataTpl<exp_type> pinocchio_data;

Eigen::Matrix<exp_type, 6, 1> qpos;
Eigen::Matrix<exp_type, 6, 1> qerr;
Eigen::Matrix<exp_type, 6, 1> qvel;
Eigen::Matrix<exp_type, 6, 1> qacc;
//Eigen::VectorXd qpos;
//Eigen::VectorXd qerr;
//Eigen::VectorXd qvel;
//Eigen::VectorXd qacc;

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

const std::string urdf_filename = std::string("models/panda.urdf");


exp_type home_pos[] = {0, 0, 0, -1.57079, 0, 1.57079};
exp_type fixed_pos[] = {-0.002493706342403138, -0.703703218059273, 0.11392999851084838, -2.205860629386432, 0.06983090103997125, 1.5706197776794442};


// simple controller
void mycontroller(const mjModel* m, mjData* d){
    printf("time: %f\n", d->time);
    printf("nu: %d\n", m->nu);
    printf("nv: %d\n", m->nv);

    // now assign d->qvel to qdot
    for(int i = 0; i < pinocchio_model.nv; i++){
        std::cout << "read i " << i << std::endl;
        std::cout << "qpos " << d->qpos << std::endl;
        std::cout << "qpos " << d->qpos[0] << std::endl;
        qpos[i] = 0;
        qpos[i] = d->qpos[i];
        qvel[i] = d->qvel[i];
        qacc[i] = d->qacc[i];
    }

    //if( m->nu==m->nv ){
    // 1. Compute the error
    exp_type error[7] = {0};
    //auto error = std::vector<double>(m->nv);
    for(int i = 0; i < m->nv; i++){
        error[i] = fixed_pos[i] - d->qpos[i];
    }
    // 2. Compute the control input
    exp_type ctrl[7] = {0};
    //auto ctrl = std::vector<double>(m->nu);
    for(int i = 0; i < m->nu; i++){
        ctrl[i] = 100 * error[i];
    }
    // 3. Apply the control input
    for(int i = 0; i < m->nu; i++){
        d->ctrl[i] = ctrl[i];
    }

    auto dynamic_drift = pinocchio::rnea(pinocchio_model, pinocchio_data, qpos, qvel, qacc);

    pinocchio::computeABADerivatives(pinocchio_model, pinocchio_data, qpos, qvel, qacc);

    // apply dynamic drift
    for(int i = 0; i < m->nv; i++){
        d->ctrl[i] += dynamic_drift[i];
    }

}

// main function
int main(int argc, const char** argv) {

    pinocchio::urdf::buildModel(urdf_filename,pinocchio_model_basic);
    pinocchio_model = pinocchio_model_basic.cast<exp_type>();
    std::cout << "model name: " << pinocchio_model.name << std::endl;

    pinocchio_data = pinocchio::DataTpl<exp_type>(pinocchio_model);

    // check command-line arguments
    if (argc!=2) {
        std::printf(" USAGE:  basic modelfile\n");
        return 0;
    }

    // load and compile model
    char error[1000] = "Could not load binary model";
    if (std::strlen(argv[1])>4 && !std::strcmp(argv[1]+std::strlen(argv[1])-4, ".mjb")) {
        m = mj_loadModel(argv[1], 0);
    } else {
        m = mj_loadXML(argv[1], 0, error, 1000);
    }
    if (!m) {
        mju_error("Load model error: %s", error);
    }

    // make data
    std::cout << "d " << d << std::endl;
    d = mj_makeData(m);
    std::cout << "d " << d << std::endl;
    std::cout << "d->qpos " << d->qpos[0] << std::endl;


    // init GLFW
    if (!glfwInit()) {
        mju_error("Could not initialize GLFW");
    }

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    mjcb_control = mycontroller;

    // run main loop, target real-time simulation and 60 fps rendering
    while (!glfwWindowShouldClose(window)) {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0/60.0) {
            //ARR_PRINT(d->ctrl, m->nu);
            mj_step(m, d);
            // print the control input
        }

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data
    mj_deleteData(d);
    mj_deleteModel(m);

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
    #endif

    return 1;
}




