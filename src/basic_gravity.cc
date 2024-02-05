#include <mujoco_exec_helper.hpp>
#include <util.h>

const std::string urdf_filename = std::string("models/panda.urdf");

exp_type home_pos[] = {0, 0, 0, -1.57079, 0, 1.57079};
exp_type fixed_pos[] = {-0.002493706342403138, -0.703703218059273, 0.11392999851084838, -2.205860629386432, 0.06983090103997125, 1.5706197776794442};


Eigen::Matrix<exp_type, -1, 1> qpos;
Eigen::Matrix<exp_type, -1, 1> qerr;
Eigen::Matrix<exp_type, -1, 1> qvel;
Eigen::Matrix<exp_type, -1, 1> qacc;
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


// simple controller
void mycontroller(const mjModel* m, mjData* d){
    printf("time: %f\n", d->time);
    printf("nu: %d\n", m->nu);
    printf("nv: %d\n", m->nv);
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
    //auto start = std::chrono::high_resolution_clock::now();
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

    // 1. Compute the error
    for(int i = 0; i < m->nv; i++){
        error[i] = fixed_pos[i] - d->qpos[i];
    }

    // 2. Compute the control input using PD controller
    exp_type ctrl[6] = {0};
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

    std::cout << "Calculate rnea" << std::endl;
    // pinocchio will calculate dynamic drift -- coriolis, centrifugal, and gravity
    auto dynamic_drift = pinocchio::rnea(pinocchio_model, pinocchio_data, qpos, qvel, qacc);
    std::cout << "Calculate rnea done" << std::endl;
    
    // apply the dynamic drift to the control input
    for(int i = 0; i < pinocchio_model.nv; i++){
        std::cout << "Adding dynamic drift" << std::endl;
        d->ctrl[i] += dynamic_drift[i];
    }
    std::cout << "Adding dynamic drift done" << std::endl;
    //auto end = std::chrono::high_resolution_clock::now();
    // write it in nanoseconds
    //std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << std::endl;

    //ARR_PRINT(error, pinocchio_model.nv)
}

// TODO: need to start OSQP with warm starting in future for better performance
void qp_preparation(const mjModel* m, mjData* d){
    qpos.resize(pinocchio_model.nv);
    qerr.resize(pinocchio_model.nv);
    qvel.resize(pinocchio_model.nv);
    qacc.resize(pinocchio_model.nv);

    osqp_set_default_settings(&settings);
    settings.verbose = false;
    settings.max_iter = 1000;

    osqp_data = (OSQPData *)c_malloc(sizeof(OSQPData));

    osqp_data->n = NUM_VAR; // number of variables(this is actually 6, but the program must see it as 18 I guess)
    osqp_data->m = NUM_CONSTR; // number of constraints
    osqp_data->P = csc_matrix(osqp_data->n, osqp_data->n, NUM_VAR, P_x, P_i, P_p);
    osqp_data->q = q;
    osqp_data->A = csc_matrix(osqp_data->m, osqp_data->n, 18+36, A_x_temp, A_i, A_p);
    osqp_data->l = l_u_temp;
    osqp_data->u = l_u_temp;

    osqp_setup(&work, osqp_data, &settings);

    file.open(file_name);
    program_start = std::chrono::high_resolution_clock::now();
}

void my_controller_QP(const mjModel* m, mjData* d){
    // controller_benchmark_start
    //auto start = std::chrono::high_resolution_clock::now();
    for(int i = 0; i < pinocchio_model.nv; i++){
        qpos[i] = d->qpos[i];
        qvel[i] = d->qvel[i];
        qacc[i] = d->qacc[i];
        qerr[i] = d->qpos[i] - fixed_pos[i];
    }
    // pinocchio derivatives start
    // pinocchio will calculate dynamic drift -- coriolis, centrifugal, and gravity
    auto dynamic_drift = pinocchio::rnea(pinocchio_model, pinocchio_data, qpos, qvel, qacc);

    pinocchio::computeABADerivatives(pinocchio_model, pinocchio_data, qpos, qvel, qacc);
    // pinocchio derivatives end

    // apply the dynamic drift to the control input
    for(int i = 0; i < pinocchio_model.nv; i++){
        d->ctrl[i] = dynamic_drift[i];
    }

    /*
    c_float P[18][18] = 
    {
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    };
    // print P in a Python array form
    std::cout << "P: " << std::endl;
    std::cout << "[";
    for(int i = 0; i < 18; i++){
        std::cout << "[";
        for(int j = 0; j < 18; j++){
            std::cout << P[i][j] << ", ";
        }
        std::cout << "], ";
    }
    std::cout << "]" << std::endl;
    */
    
    // matrix updates start
    auto Minv_temp = -T * pinocchio_data.Minv;
    Eigen::Matrix<exp_type, 6, 6> Minv = Minv_temp; // do I need this?
    
    /*
    c_float A[18][18] = 
    {
        {1, 0, 0, 0, 0, 0, -T, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0, 0, -T, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 1, 0, 0, 0, 0, 0, -T, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 1, 0, 0, 0, 0, 0, -T, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, -T, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, -T, 0, 0, 0, 0, 0, 0},
        
        {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, Minv(0, 0), Minv(0, 1), Minv(0, 2), Minv(0, 3), Minv(0, 4), Minv(0, 5)},
        {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, Minv(1, 0), Minv(1, 1), Minv(1, 2), Minv(1, 3), Minv(1, 4), Minv(1, 5)},
        {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, Minv(2, 0), Minv(2, 1), Minv(2, 2), Minv(2, 3), Minv(2, 4), Minv(2, 5)},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, Minv(3, 0), Minv(3, 1), Minv(3, 2), Minv(3, 3), Minv(3, 4), Minv(3, 5)},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, Minv(4, 0), Minv(4, 1), Minv(4, 2), Minv(4, 3), Minv(4, 4), Minv(4, 5)},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, Minv(5, 0), Minv(5, 1), Minv(5, 2), Minv(5, 3), Minv(5, 4), Minv(5, 5)},
        
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    };
    
    // print A in a Python array form
    std::cout << "A: " << std::endl;
    std::cout << "{";
    for(int i = 0; i < 18; i++){
        std::cout << "{";
        for(int j = 0; j < 18; j++){
            std::cout << A[i][j] << ", ";
        }
        std::cout << "}, " << std::endl;
    }
    std::cout << "}" << std::endl;
    */
    // Now we need to write A in sparse format, just like P
    c_float A_x[60] = 
    {
        1, 1, 1, 1, 1, 1, 
        -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1,
        Minv(0, 0), Minv(1, 0), Minv(2, 0), Minv(3, 0), Minv(4, 0), Minv(5, 0), 1,
        Minv(0, 1), Minv(1, 1), Minv(2, 1), Minv(3, 1), Minv(4, 1), Minv(5, 1), 1,
        Minv(0, 2), Minv(1, 2), Minv(2, 2), Minv(3, 2), Minv(4, 2), Minv(5, 2), 1,
        Minv(0, 3), Minv(1, 3), Minv(2, 3), Minv(3, 3), Minv(4, 3), Minv(5, 3), 1,
        Minv(0, 4), Minv(1, 4), Minv(2, 4), Minv(3, 4), Minv(4, 4), Minv(5, 4), 1,
        Minv(0, 5), Minv(1, 5), Minv(2, 5), Minv(3, 5), Minv(4, 5), Minv(5, 5), 1,    
    };

    // l and u will be = [data_ddq_dq * q; speed_limits; torque_limits]
    // I guess we can say that speed_limits and torque_limits are 50 for now...
    c_float l[18] = {
        qerr[0],
        qerr[1],
        qerr[2],
        qerr[3],
        qerr[4],
        qerr[5],
        qvel[0],
        qvel[1],
        qvel[2],
        qvel[3],
        qvel[4],
        qvel[5],
        -hard_limit, -hard_limit, -hard_limit, -hard_limit, -hard_limit, -hard_limit
    };
    c_float u[18] = {
        qerr[0],
        qerr[1],
        qerr[2],
        qerr[3],
        qerr[4],
        qerr[5],
        qvel[0],
        qvel[1],
        qvel[2],
        qvel[3],
        qvel[4],
        qvel[5],
        hard_limit, hard_limit, hard_limit, hard_limit, hard_limit, hard_limit
    };
    /*
    // print l and u in a Python array form
    std::cout << "l: " << std::endl;
    std::cout << "[";
    for(int i = 0; i < 18; i++){
        std::cout << l[i] << ", ";
    }
    std::cout << "]" << std::endl;
    std::cout << "u: " << std::endl;
    std::cout << "[";
    for(int i = 0; i < 18; i++){
        std::cout << u[i] << ", ";
    }
    std::cout << "]" << std::endl;
    */
    
    // TODO: this currently replaces ALL of A. 
    // We only need to chance Minv part, but I don't want to deal with that right now.
    osqp_update_A(work, A_x, NULL, 60);
    osqp_update_bounds(work, l, u);
    // matrix updates end

    // Solve Problem
    // osqp solve start
    osqp_solve(work);
    // osqp solve end

    // check if the problem is solved
    if(work->info->status_val != 1){
        std::cout << "QP problem not solved!" << std::endl;
        return;
    }
    for(int i = 0; i < 6; i++){
        d->ctrl[i] += work->solution->x[12+i];
    }
    // controller_benchmark_end
    // take program_start - end, if it took longer than 15 seconds, then close the file and exit the program
    //if(std::chrono::duration_cast<std::chrono::seconds>(end - program_start).count() > 15){
    //    file.close();
    //    exit(0);
    //}

    /*
    std::cout << "Completed control input: " << std::endl;
    for(int i = 0; i < 18; i++){
        std::cout << work->solution->x[i] << std::endl;
    }
    for(int i = 0; i < 6; i++){
        std::cout << "Solution: " << -work->solution->x[12+i] << std::endl;
        std::cout << "Error: " << qerr[i] << std::endl;
        std::cout << "Curr qpos: " << qpos[i] << std::endl;
        std::cout << "Curr aim: " << fixed_pos[i] << std::endl << std::endl;
    }
    */
    //ARR_PRINT(qerr, 6)
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

    qp_preparation(m, d);

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
