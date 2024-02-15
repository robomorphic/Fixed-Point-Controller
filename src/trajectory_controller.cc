#include <mujoco_exec_helper.hpp>
#include <util.h>
#include <traj.hpp>

const std::string urdf_filename = std::string("models/panda.urdf");

exp_type home_pos[] = {0, 0, 0, -1.57079, 0, 1.57079};
exp_type fixed_pos[] = {-0.002493706342403138, -0.703703218059273, 0.11392999851084838, -2.205860629386432, 0.06983090103997125, 1.5706197776794442};


Eigen::Matrix<exp_type, 6, 1> qpos;
Eigen::Matrix<exp_type, 6, 1> qerr;
Eigen::Matrix<exp_type, 6, 1> qvel;
Eigen::Matrix<exp_type, 6, 1> qacc;
Eigen::Matrix<double, 6, 1> qpos_double;
Eigen::Matrix<double, 6, 1> qerr_double;
Eigen::Matrix<double, 6, 1> qvel_double;
Eigen::Matrix<double, 6, 1> qacc_double;
//Eigen::VectorXd qpos;
//Eigen::VectorXd qerr;
//Eigen::VectorXd qvel;
//Eigen::VectorXd qacc;
OSQPSettings settings;
c_float q[18]   = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
c_float P_x[6] = {1, 1, 1, 1, 1, 1};
c_int P_i[6] = {0, 1, 2, 3, 4, 5};
c_int P_p[19] = {0, 1, 2, 3, 4, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6};
float T = 1.0/60.0;
OSQPWorkspace *work;
OSQPData      *osqp_data;
int NUM_VAR = 18;
int NUM_CONSTR = 18;

c_int A_i[] = 
{
    0, 1, 2, 3, 4, 5, 0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 11,
    6, 7, 8, 9, 10, 11, 12,
    6, 7, 8, 9, 10, 11, 13,
    6, 7, 8, 9, 10, 11, 14,
    6, 7, 8, 9, 10, 11, 15,
    6, 7, 8, 9, 10, 11, 16,
    6, 7, 8, 9, 10, 11, 17,
};
c_int A_p[] = 
{
    0, 1, 2, 3, 4, 5, 6, 8, 10, 12, 14, 16, 18, 25, 32, 39, 46, 53, 60
};

c_float A_x_temp[60] = 
{
    1, 1, 1, 1, 1, 1, 
    -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1,
    -0.0818755, -0.00405389, 0.0362688, 0.000639645, 0.0568308, -0.00106394, 1,
    -0.00405389, -0.0181714, -0.000157237, -0.0216632, 0.0120603, 0.0356071, 1, 
    0.0362688, -0.000157237, -0.0329305, -0.00377167, -0.00277871, 0.00582964, 1, 
    0.000639645, -0.0216632, -0.00377167, -0.0569292, 0.0371989, 0.119298, 1, 
    0.0568308, 0.0120603, -0.00277871, 0.0371989, -0.59978, -0.0822872, 1, 
    -0.00106394, 0.0356071, 0.00582964, 0.119298, -0.0822872, -0.784927,  1,
};

c_float l_u_temp[18] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -50, -50, -50, -50, -50, -50
};

float hard_limit = 50;


std::string file_name = "../exp/01-29/dklj.txt";
// open a file, we will write the data to this file
std::ofstream file;

// start time
auto program_start = std::chrono::high_resolution_clock::now();


void my_controller_PD(const mjModel* m, mjData* d){
    //auto start = std::chrono::high_resolution_clock::now();
    if(new_traj){
        traj_timer = d->time;
        new_traj = false;
    }
    
    exp_type error[6] = {0};
    exp_type prev_error[6] = {0};
    exp_type kp = 1; // Proportional gain
    exp_type kd = 1;  // Derivative gain

    // now assign d->qvel to qdot
    for(int i = 0; i < pinocchio_model.nv; i++){
        qpos[i] = d->qpos[i];
        qvel[i] = d->qvel[i];
        qacc[i] = d->qacc[i];
    }

    double curr_pos[6] = {0};
    for(int i = 0; i < 6; i++){
        curr_pos[i] = d->qpos[i];
    }
    double curr_vel[6] = {0};
    for(int i = 0; i < 6; i++){
        curr_vel[i] = d->qvel[i];
    }

    double* curr_goal = new double[6];
    calculate_goal(curr_pos, curr_vel, curr_goal, d->time);

    // 1. Compute the error
    for(int i = 0; i < 6; i++){
        error[i] = curr_goal[i] - d->qpos[i];
        std::cout << "current goal: " << curr_goal[i] << std::endl;
    }
    

    // 2. Compute the control input using PD controller
    exp_type ctrl[6] = {0};
    for(int i = 0; i < m->nu; i++){
        ctrl[i] = kp * error[i] + kd * (error[i] - prev_error[i]);
    }

    // 3. Apply the control input
    for(int i = 0; i < m->nu; i++){
        d->ctrl[i] = ctrl[i];
        std::cout << "control input: " << d->ctrl[i] << std::endl;
    }
    

    // Update previous error for the next iteration
    for(int i = 0; i < m->nv; i++){
        prev_error[i] = error[i];
    }

    // pinocchio will calculate dynamic drift -- coriolis, centrifugal, and gravity
    auto dynamic_drift = pinocchio::rnea(pinocchio_model, pinocchio_data, qpos, qvel, qacc);
    
    // apply the dynamic drift to the control input
    for(int i = 0; i < pinocchio_model.nv; i++){
        d->ctrl[i] += dynamic_drift[i];
    }
    //auto end = std::chrono::high_resolution_clock::now();
    // write it in nanoseconds
    //std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << std::endl;

    //ARR_PRINT(error, pinocchio_model.nv)
    delete[] curr_goal;
}



// main function
int main(int argc, const char** argv) {

    pinocchio::urdf::buildModel(urdf_filename,pinocchio_model_basic);
    pinocchio_model = pinocchio_model_basic.cast<exp_type>();
    std::cout << "model name: " << pinocchio_model.name << std::endl;

    pinocchio_data = pinocchio::DataTpl<exp_type>(pinocchio_model);

    pinocchio::urdf::buildModel(urdf_filename,pinocchio_double_model);
    pinocchio_double_data = pinocchio::Data(pinocchio_double_model);

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

    //qp_preparation(m, d);

    mjcb_control = my_controller_PD;

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
