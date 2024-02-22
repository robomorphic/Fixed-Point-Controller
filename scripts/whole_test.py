# this file writes config.hpp as needed, compiles the project and runs the simulation
import os

EXPERIMENT_DIRECTORY = "exp/02-20/"

config_file = """
#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <FixedPoint/fixed_point.hpp> // for OverflowMode


const int INT_BITS = {int_bits};
const int FRAC_BITS = {frac_bits};

OverflowMode OVERFLOW_MODE = OverflowMode::CLAMP;
typedef FixedPoint<INT_BITS, FRAC_BITS> exp_type;
// typedef double exp_type;

std::string EXPERIMENT_DIRECTORY = "exp/02-20/";
std::ofstream DATA_FILE;

double TORQUE_HARD_LIMIT = 50;

double TIME_STEP = 0.01;

#endif
"""



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

