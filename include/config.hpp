#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <FixedPoint/fixed_point.hpp> // for OverflowMode


const int INT_BITS = 8;
const int FRAC_BITS = 12;

OverflowMode OVERFLOW_MODE = OverflowMode::CLAMP;
typedef FixedPoint<INT_BITS, FRAC_BITS> exp_type;
// typedef double exp_type;

std::string EXPERIMENT_DIRECTORY = "exp/02-19/";

double TORQUE_HARD_LIMIT = 50;

double TIME_STEP = 0.01;




#endif