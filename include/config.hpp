
#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <filesystem>
#include <FixedPoint/fixed_point.hpp> // for OverflowMode

std::string EXPERIMENT_DIRECTORY = "exp/05-16/";
const int INT_BITS_STANDARD = 8;
const int FRAC_BITS_STANDARD = 8;

const int INT_BITS_GRAVITY = 16;
const int FRAC_BITS_GRAVITY = 16;

const int INT_BITS_FD = 12;
const int FRAC_BITS_FD = 12;

const int INT_BITS_ACT_ON = 12; // let's test uniform precision for now
const int FRAC_BITS_ACT_ON = 12;

const std::string urdf_filename = std::string("models/panda.urdf");

// do not change CONTROLLER_ABA_PRINT_INDEX, it is used to create a new directory for each iteration
long long CONTROLLER_ABA_PRINT_INDEX = 0;
long long temp_time = std::time(0);
// get current path
const std::string current_path = std::filesystem::current_path().string();
const std::string model_output_foldername = current_path + "/experiment_data/" + std::to_string(temp_time) + "/";
const bool PINOCCHIO_VERBOSE = true;
const bool USE_RENDER = false;

OverflowMode OVERFLOW_MODE = OverflowMode::CLAMP;
//typedef FixedPoint<INT_BITS_STANDARD, FRAC_BITS_STANDARD> exp_type;
typedef FixedPoint<INT_BITS_GRAVITY, FRAC_BITS_GRAVITY> exp_type_gravity;
typedef FixedPoint<INT_BITS_FD, FRAC_BITS_FD> exp_type_fd;
typedef FixedPoint<INT_BITS_ACT_ON, FRAC_BITS_ACT_ON> exp_type_act_on;
typedef double exp_type;

// PD controller uses this position as its target
exp_type fixed_pos[] = {-0.002493706342403138, -0.703703218059273, 0.11392999851084838, -2.205860629386432, 0.06983090103997125, 1.5706197776794442};

std::ofstream DATA_FILE;

double TORQUE_HARD_LIMIT = 50;

double TIME_STEP = 0.01;

struct {
    // this is used to signal that a new trajectory has been loaded, true when a new trajectory is loaded, false otherwise
    bool new_traj = true; 
    // in some experiments the trajectory following may start at a different position, 
    // therefore we first need to make sure that the robot is at the correct position before starting the trajectory, 
    // this variable makes sure that the time goals for the trajectory aligns
    double traj_start_time;
    // traj index must start from 1, otherwise calculate_goal will give incorrect results. 
    int traj_index = 1;
    bool went_to_init = false;
    
    // This is tolerance for the joint space
    const double GOAL_TOLERANCE = 0.1;
    const double EXP_HARD_STOP_TIME = 10.0;
} TrajectoryVars;

#endif
