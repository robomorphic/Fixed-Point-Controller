#include <osqp/osqp.h>

#include <mujoco_exec_helper.hpp>
#include <traj.hpp>
#include <util.hpp>

// It is bad to have a global variable for this, but at the same time there is no way to pass this to the controller function.
OSQPWorkspace *work;

/*
    This function is just for testing purposes. Can be usable for testing or debugging purposes.
*/
void my_controller_PD(const mjModel* m, mjData* d){
    Eigen::Matrix<exp_type, 6, 1> qpos;
    Eigen::Matrix<exp_type, 6, 1> qvel;
    Eigen::Matrix<exp_type, 6, 1> qacc;
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
    for(int i = 0; i < 6; i++){
        error[i] = fixed_pos[i] - d->qpos[i];
    }

    // 2. Compute the control input using PD controller
    exp_type ctrl[6] = {0};
    for(int i = 0; i < 6; i++){
        ctrl[i] = kp * error[i] + kd * (error[i] - prev_error[i]);
    }

    // 3. Apply the control input
    for(int i = 0; i < 6; i++){
        d->ctrl[i] = ctrl[i];
    }

    // Update previous error for the next iteration
    for(int i = 0; i < 6; i++){
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

}

/*
    This function starts the OSQP solver and prepares the constant matrices for the QP problem.
    It can be deleted and its contents can be moved to the my_controller_QP function.
    But we just need to create the problem and update the matrices when needed.
    This function is called only once at the beginning of the simulation.
*/
void qp_preparation(const mjModel* m, mjData* d){
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


    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = false;
    settings.max_iter = 1000;

    OSQPData* osqp_data = (OSQPData *)c_malloc(sizeof(OSQPData));

    c_float q[18]   = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    c_float P_x[6] = {1, 1, 1, 1, 1, 1};
    c_int P_i[6] = {0, 1, 2, 3, 4, 5};
    c_int P_p[19] = {0, 1, 2, 3, 4, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6};

    int NUM_VAR = 18;
    int NUM_CONSTR = 18;    

    osqp_data->n = NUM_VAR; // number of variables(this is actually 6, but the program must see it as 18 I guess)
    osqp_data->m = NUM_CONSTR; // number of constraints
    osqp_data->P = csc_matrix(osqp_data->n, osqp_data->n, NUM_VAR, P_x, P_i, P_p);
    osqp_data->q = q;
    osqp_data->A = csc_matrix(osqp_data->m, osqp_data->n, 18+36, A_x_temp, A_i, A_p);
    osqp_data->l = l_u_temp;
    osqp_data->u = l_u_temp;

    osqp_setup(&work, osqp_data, &settings);
}

/*
    Optimal control function using OSQP solver.
    For now the robot's DoF is set to 6, be careful with other robots.
*/
void my_controller_QP(const mjModel* m, mjData* d){
    // bad, but necessary for now
    Eigen::Matrix<double, 6, 1> qpos;
    Eigen::Matrix<double, 6, 1> qerr;
    Eigen::Matrix<double, 6, 1> qvel;
    Eigen::Matrix<double, 6, 1> qacc;
    Eigen::Matrix<exp_type_gravity, 6, 1> qpos_gravity;
    Eigen::Matrix<exp_type_gravity, 6, 1> qerr_gravity;
    Eigen::Matrix<exp_type_gravity, 6, 1> qvel_gravity;
    Eigen::Matrix<exp_type_gravity, 6, 1> qacc_gravity;
    Eigen::Matrix<exp_type_fd, 6, 1> qpos_fd;
    Eigen::Matrix<exp_type_fd, 6, 1> qerr_fd;
    Eigen::Matrix<exp_type_fd, 6, 1> qvel_fd;
    Eigen::Matrix<exp_type_fd, 6, 1> qacc_fd;
    double traj_time = d->time - TrajectoryVars.traj_start_time;

    // We stop the simulation after a certain time for our experiments
    stop_sim_if_needed(traj_time);

    for(int i = 0; i < pinocchio_model.nv; i++){
        qpos[i] = qpos_gravity[i] = qpos_fd[i] = d->qpos[i];
        qvel[i] = qvel_gravity[i] = qvel_fd[i] = d->qvel[i];
        qacc[i] = qacc_gravity[i] = qacc_fd[i] = d->qacc[i];
    }
    save_position(qpos, qvel, qacc, traj_time);

    double curr_goal[6] = {0};
    calculate_goal(qpos, qvel, curr_goal, traj_time, d->time);
    for(int i = 0; i < 6; i++){
        qerr[i] = qpos[i] - curr_goal[i];
    }

    // Changing the stdout buffer for pinocchio debug prints
    //freopen(model_output_foldername + "/ABA/" + std::to_string(CONTROLLER_ABA_PRINT_INDEX) + "/pinocchio_output.txt", "w", stdout);
    // save the old buf
    // auto old_buf = std::cout.rdbuf();
    // redirect std::cout to out.txt
    //std::cout << "model_output_foldername: " << model_output_foldername << std::endl;
    //std::string temp_str = model_output_foldername + "ABA/" + std::to_string(CONTROLLER_ABA_PRINT_INDEX) + "/";
    //std::filesystem::create_directories(temp_str);
    //std::ofstream file;
    //file.open(temp_str + "pinocchio_output.txt");
    //std::cout.rdbuf(file.rdbuf());

    // pinocchio will calculate dynamic drift -- coriolis, centrifugal, and gravity
    auto dynamic_drift = pinocchio::rnea(pinocchio_model_gravity, pinocchio_data_gravity, qpos_gravity, qvel_gravity, qacc_gravity);
    // this calculates Minv, the inverse of the inertia matrix
    if(PINOCCHIO_VERBOSE){
        pinocchioVerbose::computeMinverseVerbose(pinocchio_model_fd, pinocchio_data_fd, qpos_fd);
        pinocchioVerbose::computeMinverseVerbose(pinocchio_model, pinocchio_data, qpos);
        print_ABA_output(pinocchio_data, pinocchio_data_fd);
    }
    else{
        pinocchioVerbose::computeMinverseVerbose(pinocchio_model_fd, pinocchio_data_fd, qpos_fd);
    }
    
    //std::cout.rdbuf(old_buf);

    // apply the dynamic drift to the control input
    for(int i = 0; i < pinocchio_model.nv; i++){
        d->ctrl[i] = dynamic_drift[i];
    }
    
    auto Minv_temp = -TIME_STEP * pinocchio_data_fd.Minv;
    Eigen::Matrix<exp_type_fd, 6, 6> Minv = Minv_temp; // do I need this?
    
    // Now we need to write A in sparse format, just like P
    c_float A_x[60] = 
    {
        1, 1, 1, 1, 1, 1, 
        -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, // Ideally the -1 elements should be -T, but for some reason it doesn't work
        Minv(0, 0), Minv(1, 0), Minv(2, 0), Minv(3, 0), Minv(4, 0), Minv(5, 0), 1,
        Minv(0, 1), Minv(1, 1), Minv(2, 1), Minv(3, 1), Minv(4, 1), Minv(5, 1), 1,
        Minv(0, 2), Minv(1, 2), Minv(2, 2), Minv(3, 2), Minv(4, 2), Minv(5, 2), 1,
        Minv(0, 3), Minv(1, 3), Minv(2, 3), Minv(3, 3), Minv(4, 3), Minv(5, 3), 1,
        Minv(0, 4), Minv(1, 4), Minv(2, 4), Minv(3, 4), Minv(4, 4), Minv(5, 4), 1,
        Minv(0, 5), Minv(1, 5), Minv(2, 5), Minv(3, 5), Minv(4, 5), Minv(5, 5), 1,    
    };

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
        -TORQUE_HARD_LIMIT, -TORQUE_HARD_LIMIT, -TORQUE_HARD_LIMIT, -TORQUE_HARD_LIMIT, -TORQUE_HARD_LIMIT, -TORQUE_HARD_LIMIT
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
        TORQUE_HARD_LIMIT, TORQUE_HARD_LIMIT, TORQUE_HARD_LIMIT, TORQUE_HARD_LIMIT, TORQUE_HARD_LIMIT, TORQUE_HARD_LIMIT
    };
    
    // TODO: this currently replaces ALL ELEMENTS of A. 
    // We only need to change Minv values, but I don't want to deal with that right now.
    osqp_update_A(work, A_x, NULL, 60);
    osqp_update_bounds(work, l, u);

    // Solve Problem
    osqp_solve(work);

    // check if the problem is solved
    if(work->info->status_val != 1){
        std::cout << "QP problem not solved!" << std::endl;
        return;
    }
    for(int i = 0; i < 6; i++){
        d->ctrl[i] += work->solution->x[12+i];
    }
}


// main function
int main(int argc, const char** argv) {
    // pinocchio takes panda urdf file and creates a model
    // because of a bug, pinocchio::ModelTpl<exp_type> cannot be used
    // Therefore we are creating the robot model with double type and then casting it to exp_type
    pinocchio::urdf::buildModel(urdf_filename, pinocchio_model_basic);
    std::cerr << "pinocchio_model_basic: " << pinocchio_model_basic << std::endl;

    pinocchio_model         = pinocchio_model_basic.cast<exp_type>();
    //model_cast(pinocchio_model_basic, pinocchio_model);
    std::cerr << "pinocchio_model: " << pinocchio_model << std::endl;
    std::cerr << "pinocchio_model_basic: " << pinocchio_model_basic << std::endl;

    pinocchio_model_gravity = pinocchio_model_basic.cast<exp_type_fd>();
    //model_cast(pinocchio_model_basic, pinocchio_model_gravity);
    std::cerr << "pinocchio_model_gravity: " << pinocchio_model_gravity << std::endl;

    pinocchio_model_fd      = pinocchio_model_basic.cast<exp_type_fd>();
    //model_cast(pinocchio_model_basic, pinocchio_model_fd);
    std::cerr << "pinocchio_model_fd: " << pinocchio_model_fd << std::endl;

    if(PINOCCHIO_VERBOSE) print_model(pinocchio_model_basic, pinocchio_model, pinocchio_model_fd);
    //std::cout << "model name: " << pinocchio_model.name << std::endl;

    // create a dummy model
    pinocchio::ModelTpl<FixedPoint<31, 31>> dummy_model;
    model_cast(pinocchio_model, dummy_model);
    std::cerr << "dummy_model: " << dummy_model << std::endl;

    // print "inertias" of these models
    std::cerr << "inertia: " << pinocchio_model.inertias[0] << std::endl;
    std::cerr << "inertia gravity: " << pinocchio_model_gravity.inertias[0] << std::endl;
    std::cerr << "inertia fd: " << pinocchio_model_fd.inertias[0] << std::endl;
    std::cerr << "2:" << std::endl;
    std::cerr << "inertia: " << pinocchio_model.inertias[1] << std::endl;
    std::cerr << "inertia gravity: " << pinocchio_model_gravity.inertias[1] << std::endl;
    std::cerr << "inertia fd: " << pinocchio_model_fd.inertias[1] << std::endl;
    std::cerr << "3:" << std::endl;
    std::cerr << "inertia: " << pinocchio_model.inertias[2] << std::endl;
    std::cerr << "inertia gravity: " << pinocchio_model_gravity.inertias[2] << std::endl;
    std::cerr << "inertia fd: " << pinocchio_model_fd.inertias[2] << std::endl;
    std::cerr << "4:" << std::endl;
    std::cerr << "inertia: " << pinocchio_model.inertias[3] << std::endl;
    std::cerr << "inertia gravity: " << pinocchio_model_gravity.inertias[3] << std::endl;
    std::cerr << "inertia fd: " << pinocchio_model_fd.inertias[3] << std::endl;
    std::cerr << "5:" << std::endl;
    std::cerr << "inertia: " << pinocchio_model.inertias[4] << std::endl;
    std::cerr << "inertia gravity: " << pinocchio_model_gravity.inertias[4] << std::endl;
    std::cerr << "inertia fd: " << pinocchio_model_fd.inertias[4] << std::endl;
    std::cerr << "6:" << std::endl;
    std::cerr << "inertia: " << pinocchio_model.inertias[5] << std::endl;
    std::cerr << "inertia gravity: " << pinocchio_model_gravity.inertias[5] << std::endl;
    std::cerr << "inertia fd: " << pinocchio_model_fd.inertias[5] << std::endl;

    // cast doesn't work for Ubuntu with some systems for some reason

    pinocchio_data          = pinocchio::DataTpl<exp_type>(pinocchio_model);
    //std::cerr << "pinocchio_data: " << pinocchio_data << std::endl;
    pinocchio_data_gravity  = pinocchio::DataTpl<exp_type_gravity>(pinocchio_model_gravity);
    //std::cerr << "pinocchio_data_gravity: " << pinocchio_data_gravity << std::endl;
    pinocchio_data_fd       = pinocchio::DataTpl<exp_type_fd>(pinocchio_model_fd);
    //std::cerr << "pinocchio_data_fd: " << pinocchio_data_fd << std::endl;

    // check command-line arguments
    if (argc!=2) {
        std::printf(" USAGE: ./exec modelfile\n");
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
    if (USE_RENDER && !glfwInit()) {
        mju_error("Could not initialize GLFW");
    }

    GLFWwindow* window;
    if(USE_RENDER){
        window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);
    }
    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    // mjr_defaultContext(&con);
    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    if(USE_RENDER){
        mjr_makeContext(m, &con, mjFONTSCALE_150);
    }

    // install GLFW mouse and keyboard callbacks
    if(USE_RENDER){
        glfwSetKeyCallback(window, keyboard);
        glfwSetCursorPosCallback(window, mouse_move);
        glfwSetMouseButtonCallback(window, mouse_button);
        glfwSetScrollCallback(window, scroll);
    }
    initialize_output_file();

    qp_preparation(m, d);
    mjcb_control = my_controller_QP;

    // The main controller is not exactly 1/T Hz, I believe I can make it 1/T Hz with a global variable
    if(USE_RENDER){
        while(true){
            mjtNum simstart = d->time;
            while (d->time - simstart < TIME_STEP) {
                mj_step(m, d);
            }
            
            mjrRect viewport = {0, 0, 0, 0};
            glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

            mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        
            // update scene and render
            mjr_render(viewport, &scn, &con);
            // swap OpenGL buffers (blocking call due to v-sync)
            glfwSwapBuffers(window);
            // process pending GUI events, call GLFW callbacks
            glfwPollEvents();
        }
    }
    else{
        while(true){
            mj_step(m, d);
            mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        }
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



