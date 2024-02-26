# this file writes config.hpp as needed, compiles the project and runs the simulation
import os

EXPERIMENT_DIRECTORY = "exp/02-20/"

config_file = """
#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <FixedPoint/fixed_point.hpp> // for OverflowMode

const std::string urdf_filename = std::string("models/panda.urdf");

const bool USE_RENDER = true;
const int INT_BITS_STANDARD = {int_bits};
const int FRAC_BITS_STANDARD = {frac_bits};

const int INT_BITS_GRAVITY = {int_bits};
const int FRAC_BITS_GRAVITY = {frac_bits};

const int INT_BITS_FD = {int_bits};
const int FRAC_BITS_FD = {frac_bits};

OverflowMode OVERFLOW_MODE = OverflowMode::CLAMP;
typedef FixedPoint<INT_BITS, FRAC_BITS> exp_type;
typedef FixedPoint<INT_BITS_GRAVITY, FRAC_BITS_GRAVITY> exp_type_gravity;
typedef FixedPoint<INT_BITS_FD, FRAC_BITS_FD> exp_type_fd;
// typedef double exp_type;

// PD controller uses this position as its target
exp_type fixed_pos[] = {-0.002493706342403138, -0.703703218059273, 0.11392999851084838, -2.205860629386432, 0.06983090103997125, 1.5706197776794442};

std::string EXPERIMENT_DIRECTORY = "exp/02-20/";
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
    const double EXP_HARD_STOP_TIME = 40.0;
} TrajectoryVars;

#endif
"""


# Need to update these!
int_bits_list = range(6, 16)
frac_bits_list = range(3, 16)

for int_bits in int_bits_list:
    for frac_bits in frac_bits_list:
        # if EXPERIMENT_DIRECTORY+str(int_bits)+'_'+str(frac_bits)+'/data.csv' exists, skip
        if os.path.exists(EXPERIMENT_DIRECTORY+str(int_bits)+'_'+str(frac_bits)+'/data.csv'):
            print(f'Experiment with int_bits: {int_bits}, frac_bits: {frac_bits} exists. Skipping...')
            continue
        with open('include/config.hpp', 'w') as f:
            f.write(config_file.format(int_bits=int_bits, frac_bits=frac_bits))

        print(f'Compiling with int_bits: {int_bits}, frac_bits: {frac_bits}')
        # compile the project
        os.system('make > /dev/null')
        os.system('./bin/trajectory_tracking ../mujoco_menagerie/franka_emika_panda/scene.xml > /dev/null')

