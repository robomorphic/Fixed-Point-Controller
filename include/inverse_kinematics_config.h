#ifndef INVERSE_KINEMATICS_CONFIG_H
#define INVERSE_KINEMATICS_CONFIG_H

const int NU = 7;
const double eps = 1e-4;
const int IT_MAX = 1000;
const double DT = 1e-1;
const double damp = 1e-6;

typedef struct SE3{
    double translation[3];
    double rotation[3][3];
} SE3;

typedef struct pose{
    double position[3];
    double orientation[3];
} pose;

typedef struct joint_position{
    double position[NU];
    double cos[NU];
    double sin[NU];
} joint_position;

typedef struct Matrix6x{
    double data[6][NU];
} Matrix6x;

typedef struct Vector6d{
    double data[6];
} Vector6d;

typedef struct Vectorxd{
    double data[NU];
} VectorXd;

typedef struct SE3s {
    SE3 SE3[NU];
} SE3s;

#endif // INVERSE_KINEMATICS_CONFIG_H