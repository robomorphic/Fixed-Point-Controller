# this file writes config.hpp as needed, compiles the project and runs the simulation
import os
import argparse

EXPERIMENT_DIRECTORY = "exp/04-22/"

config_file = """
#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <FixedPoint/fixed_point.hpp> // for OverflowMode

const std::string urdf_filename = std::string("models/panda.urdf");

// do not change CONTROLLER_ABA_PRINT_INDEX, it is used to create a new directory for each iteration
long long CONTROLLER_ABA_PRINT_INDEX = 0;
long long temp_time = std::time(0);
// get current path
const std::string current_path = std::filesystem::current_path().string();
const std::string model_output_foldername = current_path + "/experiment_data/" + std::to_string(temp_time) + "/";
const bool PINOCCHIO_VERBOSE = true;
const bool USE_RENDER = false;
std::string EXPERIMENT_DIRECTORY = "{experiment_directory}";
const int INT_BITS_STANDARD = {int_bits};
const int FRAC_BITS_STANDARD = {frac_bits};

const int INT_BITS_GRAVITY = {int_bits_gravity};
const int FRAC_BITS_GRAVITY = {frac_bits_gravity};

const int INT_BITS_FD = {int_bits_fd};
const int FRAC_BITS_FD = {frac_bits_fd};

const int INT_BITS_ACT_ON = 16;
const int FRAC_BITS_ACT_ON = 16;

OverflowMode OVERFLOW_MODE = OverflowMode::CLAMP;
//typedef FixedPoint<INT_BITS_STANDARD, FRAC_BITS_STANDARD> exp_type;
typedef FixedPoint<INT_BITS_GRAVITY, FRAC_BITS_GRAVITY> exp_type_gravity;
typedef FixedPoint<INT_BITS_FD, FRAC_BITS_FD> exp_type_fd;
typedef FixedPoint<INT_BITS_ACT_ON, FRAC_BITS_ACT_ON> exp_type_act_on;
typedef double exp_type;

// PD controller uses this position as its target
exp_type fixed_pos[] = {{-0.002493706342403138, -0.703703218059273, 0.11392999851084838, -2.205860629386432, 0.06983090103997125, 1.5706197776794442}};

std::ofstream DATA_FILE;

double TORQUE_HARD_LIMIT = 50;

double TIME_STEP = 0.01;

struct {{
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
    const double EXP_HARD_STOP_TIME = 5.0;
}} TrajectoryVars;

#endif
"""

# get min arg max arg for int gravity and int fd from arguments
parser = argparse.ArgumentParser(
    description='Run experiments with different fixed point configurations'
)
parser.add_argument('--min-gravity-int', type=int, required=True)
parser.add_argument('--max-gravity-int', type=int, required=True)
parser.add_argument('--min-gravity-frac', type=int, required=True)
parser.add_argument('--max-gravity-frac', type=int, required=True)
parser.add_argument('--min-fd-int', type=int, required=True)
parser.add_argument('--max-fd-int', type=int, required=True)
parser.add_argument('--min-fd-frac', type=int, required=True)
parser.add_argument('--max-fd-frac', type=int, required=True)

args = parser.parse_args()

min_gravity_int = args.min_gravity_int
max_gravity_int = args.max_gravity_int
min_gravity_frac = args.min_gravity_frac
max_gravity_frac = args.max_gravity_frac
min_fd_int = args.min_fd_int
max_fd_int = args.max_fd_int
min_fd_frac = args.min_fd_frac
max_fd_frac = args.max_fd_frac

gravity_int_bits_list = list(range(min_gravity_int, max_gravity_int+1))
gravity_frac_bits_list = list(range(min_gravity_frac, max_gravity_frac+1))
fd_int_bits_list = list(range(min_fd_int, max_fd_int+1))
fd_frac_bits_list = list(range(min_fd_frac, max_fd_frac+1))
print(f'gravity_int_bits_list: {gravity_int_bits_list}')
print(f'gravity_frac_bits_list: {gravity_frac_bits_list}')
print(f'fd_int_bits_list: {fd_int_bits_list}')
print(f'fd_frac_bits_list: {fd_frac_bits_list}')

for gravity_int_bit in gravity_int_bits_list:
    for gravity_frac_bit in gravity_frac_bits_list:
        for fd_int_bit in fd_int_bits_list:
            for fd_frac_bit in fd_frac_bits_list:
                if os.path.exists(EXPERIMENT_DIRECTORY+str(gravity_int_bit)+'_'+str(gravity_frac_bit)+'_'+str(fd_int_bit)+'_'+str(fd_frac_bit)+'/data.csv'):
                    print(f'Experiment with gravity_int_bit: {gravity_int_bit}, gravity_frac_bit: {gravity_frac_bit}, fd_int_bit: {fd_int_bit}, fd_frac_bit: {fd_frac_bit} exists. Skipping...')
                    continue
                with open('include/config.hpp', 'w') as f:
                    f.write(config_file.format(int_bits=31, frac_bits=31, int_bits_gravity=gravity_int_bit, frac_bits_gravity=gravity_frac_bit, int_bits_fd=fd_int_bit, frac_bits_fd=fd_frac_bit, experiment_directory=EXPERIMENT_DIRECTORY))
                print(f'Compiling with gravity_int_bit: {gravity_int_bit}, gravity_frac_bit: {gravity_frac_bit}, fd_int_bit: {fd_int_bit}, fd_frac_bit: {fd_frac_bit}')
                # compile the project
                print(f"Test: gravity_int_bit: {gravity_int_bit}, gravity_frac_bit: {gravity_frac_bit}, fd_int_bit: {fd_int_bit}, fd_frac_bit: {fd_frac_bit}")
                os.system('make')
                os.system('./bin/trajectory_tracking ../mujoco_menagerie/franka_emika_panda/scene.xml')
                os.system('python3 scripts/detailed_inspection.py')
                #os.system('python3 scripts/experiment_summary_plot.py')



