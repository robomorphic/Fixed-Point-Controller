
#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <FixedPoint/fixed_point.hpp> // for OverflowMode

const std::string urdf_filename = std::string("models/panda.urdf");

const bool USE_RENDER = true;
const int INT_BITS = 30;
const int FRAC_BITS = 30;

OverflowMode OVERFLOW_MODE = OverflowMode::CLAMP;
typedef FixedPoint<INT_BITS, FRAC_BITS> exp_type;
// typedef double exp_type;

// PD controller uses this position as its target
exp_type fixed_pos[] = {-0.002493706342403138, -0.703703218059273, 0.11392999851084838, -2.205860629386432, 0.06983090103997125, 1.5706197776794442};

std::string EXPERIMENT_DIRECTORY = "exp/02-20/";
std::ofstream DATA_FILE;

double TORQUE_HARD_LIMIT = 50;

double TIME_STEP = 0.01;

#endif
