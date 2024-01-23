// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "osqp/osqp.h"

#include <typeinfo>
#include <cstdio>
#include <cstring>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <iostream>

#include "pinocchio/parsers/urdf.hpp"
 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"

 


const std::string urdf_filename = std::string("../models/panda.urdf");

// Load the urdf model
pinocchio::Model model;
// Create data required by the algorithms
pinocchio::Data data;


MJAPI extern mjfGeneric mjcb_control;

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context



// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

#define ARR_PRINT(vec, size) printf(#vec ": "); for(int i = 0; i < size; i++) printf("%f ", vec[i]); printf("\n");

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

// home is qpos="0 0 0 -1.57079 0 1.57079 -0.7853"
double home_pos[] = {0, 0, 0, -1.57079, 0, 1.57079, -0.7853};
double fixed_pos[] = {-0.002493706342403138, -0.703703218059273, 0.11392999851084838, -2.205860629386432, 0.06983090103997125, 1.5706197776794442, 0.8786033292295339};

// simple controller
void mycontroller(const mjModel* m, mjData* d){
    printf("time: %f\n", d->time);
    printf("nu: %d\n", m->nu);
    printf("nv: %d\n", m->nv);
    //if( m->nu==m->nv ){
    // 1. Compute the error
    double error[7] = {0};
    //auto error = std::vector<double>(m->nv);
    for(int i = 0; i < m->nv; i++){
        error[i] = fixed_pos[i] - d->qpos[i];
    }
    // 2. Compute the control input
    double ctrl[7] = {0};
    //auto ctrl = std::vector<double>(m->nu);
    for(int i = 0; i < m->nu; i++){
        ctrl[i] = 100 * error[i];
    }
    // 3. Apply the control input
    for(int i = 0; i < m->nu; i++){
        d->ctrl[i] = ctrl[i];
    }

    /*
    printf("actuator_limits: \n");
    for(int i = 0; i < m->nu; i++){
        printf("%f %f\n", m->actuator_ctrlrange[2*i], m->actuator_ctrlrange[2*i+1]);
    }
    printf("force_limits: \n");
    for(int i = 0; i < m->nu; i++){
        printf("%f %f\n", m->actuator_forcerange[2*i], m->actuator_forcerange[2*i+1]);
    }
    */
}

void my_controller_PD(const mjModel* m, mjData* d){
    double error[7] = {0};
    double prev_error[7] = {0};
    double kp = 1; // Proportional gain
    double kd = 1;  // Derivative gain

    Eigen::VectorXd q;
    q.resize(model.nv);
    Eigen::VectorXd qvel;
    qvel.resize(model.nv);
    Eigen::VectorXd qacc;
    qacc.resize(model.nv);
    // now assign d->qvel to qdot
    for(int i = 0; i < model.nv; i++){
        q[i] = d->qpos[i];
        qvel[i] = d->qvel[i];
        qacc[i] = d->qacc[i];
    }

    // 1. Compute the error
    for(int i = 0; i < m->nv; i++){
        error[i] = fixed_pos[i] - d->qpos[i];
    }

    // 2. Compute the control input using PD controller
    double ctrl[7] = {0};
    for(int i = 0; i < m->nu; i++){
        ctrl[i] = kp * error[i] + kd * (error[i] - prev_error[i]);
    }

    // 3. Apply the control input
    for(int i = 0; i < m->nu; i++){
        d->ctrl[i] = ctrl[i];
    }

    // Update previous error for the next iteration
    for(int i = 0; i < m->nv; i++){
        prev_error[i] = error[i];
    }

    // let's benchmark this part, get time
    //auto start = std::chrono::high_resolution_clock::now();
    // pinocchio will calculate dynamic drift -- coriolis, centrifugal, and gravity
    auto dynamic_drift = pinocchio::rnea(model, data, q, qvel, qacc);
    auto end = std::chrono::high_resolution_clock::now();
    //std::chrono::duration<double> elapsed = end - start;
    //std::cout << "pinocchio rnea took " << elapsed.count() << " seconds" << std::endl;
    std::cout << "dynamic drift: " << std::endl;
    std::cout << dynamic_drift << std::endl;

    // apply the dynamic drift to the control input
    for(int i = 0; i < model.nv; i++){
        d->ctrl[i] += dynamic_drift[i];
    }

    // how to compute A and B matrices?
    // A = djoint_acc_dq
    // B = djoint_acc_dtau
    //pinocchio::computeRNEADerivatives(model, data, q, qvel, qacc);
    pinocchio::computeABADerivatives(model, data, q, qvel, qacc);
    std::cout << "A: " << std::endl;
    std::cout << data.ddq_dq << std::endl;
    // calculate B
    std::cout << "Minv: " << std::endl;
    std::cout << data.Minv << std::endl;

    ARR_PRINT(error, 7)

}

void my_controller_QP(const mjModel* m, mjData* d){
    double error[7] = {0};
    double prev_error[7] = {0};
    double kp = 1; // Proportional gain
    double kd = 1;  // Derivative gain

    Eigen::VectorXd qpos;
    qpos.resize(model.nv);
    Eigen::VectorXd qerr;
    qerr.resize(model.nv);
    Eigen::VectorXd qvel;
    qvel.resize(model.nv);
    Eigen::VectorXd qacc;
    qacc.resize(model.nv);
    Eigen::VectorXd state;
    state.resize(model.nv*3);
    // now assign d->qvel to qdot
    for(int i = 0; i < model.nv; i++){
        qpos[i] = d->qpos[i];
        qvel[i] = d->qvel[i];
        qacc[i] = d->qacc[i];
        qerr[i] = fixed_pos[i] - d->qpos[i];
        state[i] = d->qpos[i];
        state[i+model.nv] = d->qvel[i];
        state[i+model.nv*2] = 0; // this is torque input!
    }



    // pinocchio will calculate dynamic drift -- coriolis, centrifugal, and gravity
    auto dynamic_drift = pinocchio::rnea(model, data, qpos, qvel, qacc);
    auto end = std::chrono::high_resolution_clock::now();

    std::cout << "dynamic drift: " << std::endl;
    std::cout << dynamic_drift << std::endl;

    // apply the dynamic drift to the control input
    for(int i = 0; i < model.nv; i++){
        d->ctrl[i] = dynamic_drift[i];
    }

    // how to compute A and B matrices?
    pinocchio::computeABADerivatives(model, data, qpos, qvel, qacc);
    std::cout << "A: " << std::endl;
    std::cout << data.ddq_dq << std::endl;
    // calculate B
    std::cout << "Minv: " << std::endl;
    std::cout << data.Minv << std::endl;

    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = true;
    settings.max_iter = 1000;
    std::cout << "Set up OSQP settings" << std::endl;
    
    // Populate matrices for a QP controller
    // min error * P * error
    // s.t. x_dot = A * x + B * u
    
    // P = I
    c_float P[18][18] = 
    {
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    };
    // Now I need to write this in OSQP format
    c_float P_x[18] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 
                       1, 1, 1, 1, 1, 1, 1, 1, 1};
    c_int P_i[18] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
    c_int P_p[18] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};

    c_float q[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // A = [I 0 B; 0 I 0; 0 0 I], B = data.Minv, so every element here is a 6x6 matrix
    c_float A[18][18] = 
    {
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, data.Minv(0, 0), data.Minv(0, 1), data.Minv(0, 2), data.Minv(0, 3), data.Minv(0, 4), data.Minv(0, 5)},
        {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, data.Minv(1, 0), data.Minv(1, 1), data.Minv(1, 2), data.Minv(1, 3), data.Minv(1, 4), data.Minv(1, 5)},
        {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, data.Minv(2, 0), data.Minv(2, 1), data.Minv(2, 2), data.Minv(2, 3), data.Minv(2, 4), data.Minv(2, 5)},
        {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, data.Minv(3, 0), data.Minv(3, 1), data.Minv(3, 2), data.Minv(3, 3), data.Minv(3, 4), data.Minv(3, 5)},
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, data.Minv(4, 0), data.Minv(4, 1), data.Minv(4, 2), data.Minv(4, 3), data.Minv(4, 4), data.Minv(4, 5)},
        {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, data.Minv(5, 0), data.Minv(5, 1), data.Minv(5, 2), data.Minv(5, 3), data.Minv(5, 4), data.Minv(5, 5)},
        {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    };

    // Now we need to write A in sparse format, just like P
    c_float A_x[18+36] = 
    {
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        data.Minv(0, 0), data.Minv(0, 1), data.Minv(0, 2), data.Minv(0, 3), data.Minv(0, 4), data.Minv(0, 5),
        data.Minv(1, 0), data.Minv(1, 1), data.Minv(1, 2), data.Minv(1, 3), data.Minv(1, 4), data.Minv(1, 5),
        data.Minv(2, 0), data.Minv(2, 1), data.Minv(2, 2), data.Minv(2, 3), data.Minv(2, 4), data.Minv(2, 5),
        data.Minv(3, 0), data.Minv(3, 1), data.Minv(3, 2), data.Minv(3, 3), data.Minv(3, 4), data.Minv(3, 5),
        data.Minv(4, 0), data.Minv(4, 1), data.Minv(4, 2), data.Minv(4, 3), data.Minv(4, 4), data.Minv(4, 5),
        data.Minv(5, 0), data.Minv(5, 1), data.Minv(5, 2), data.Minv(5, 3), data.Minv(5, 4), data.Minv(5, 5),
    };
    c_int A_i[18+36] = 
    {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
        0, 0, 0, 0, 0, 0,
        1, 1, 1, 1, 1, 1,
        2, 2, 2, 2, 2, 2,
        3, 3, 3, 3, 3, 3,
        4, 4, 4, 4, 4, 4,
        5, 5, 5, 5, 5, 5,
    };
    c_int A_p[18+36] = 
    {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
        12, 13, 14, 15, 16, 17,
        12, 13, 14, 15, 16, 17,
        12, 13, 14, 15, 16, 17,
        12, 13, 14, 15, 16, 17,
        12, 13, 14, 15, 16, 17,
        12, 13, 14, 15, 16, 17,
    };



    // l and u will be = [data_ddq_dq * q; speed_limits; torque_limits]
    // I guess we can say that speed_limits and torque_limits are 50 for now...
    auto temp_mult = data.ddq_dq * qpos;
    c_float l[18] = {
        temp_mult[0],
        temp_mult[1],
        temp_mult[2],
        temp_mult[3],
        temp_mult[4],
        temp_mult[5],
        -50, -50, -50, -50, -50, -50,
        -50, -50, -50, -50, -50, -50
    };
    c_float u[18] = {
        temp_mult[0],
        temp_mult[1],
        temp_mult[2],
        temp_mult[3],
        temp_mult[4],
        temp_mult[5],
        50, 50, 50, 50, 50, 50,
        50, 50, 50, 50, 50, 50
    };

    // Populate matrices for a QP controller

    OSQPWorkspace *work;
    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

    data->n = 18; // number of variables(this is actually 6, but the program must see it as 18 I guess)
    data->m = 18; // number of constraints
    data->P = csc_matrix(data->n, data->n, 18, P_x, P_i, P_p);
    data->q = q;
    //data->A = csc_matrix(data->m, data->n, 18+36, A_x, A_i, A_p);
    //data->l = l;
    //data->u = u;

    data->A = csc_matrix(18, 18, 0, NULL, NULL, NULL); // this is a dummy matrix, we will update it later
    data->l = NULL;
    data->u = NULL;

    settings.verbose = true;

    osqp_setup(&work, data, &settings);

    // Solve Problem
    osqp_solve(work);

    // Print solution
    printf("Solution:\n");
    for (int i = 0; i < data->n; i++) {
        printf("%f\n", work->solution->x[i]);
    }




    

    ARR_PRINT(error, 7)

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


// main function
int main(int argc, const char** argv) {

    pinocchio::urdf::buildModel(urdf_filename,model);
    std::cout << "model name: " << model.name << std::endl;

    data = pinocchio::Data(model);




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
    d = mj_makeData(m);

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

    mjcb_control = my_controller_QP;

    // run main loop, target real-time simulation and 60 fps rendering
    while (!glfwWindowShouldClose(window)) {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0/60.0) {
            ARR_PRINT(d->ctrl, m->nu);
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
