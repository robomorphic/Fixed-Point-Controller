#ifndef INVERSE_KINEMATICS_HELPER_H
#define INVERSE_KINEMATICS_HELPER_H

#include "inverse_kinematics_config.h"

// 3x3 matrix multiplication
void matrix_mult_3x3(double a[3][3], double b[3][3], double c[3][3]){
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            c[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

// 3x3 matrix by 3x1 vector multiplication
void matrix_mult_3x1(double a[3][3], double b[3], double c[3]){
    for (int i = 0; i < 3; i++) {
        c[i] = 0;
        for (int j = 0; j < 3; j++) {
            c[i] += a[i][j] * b[j];
        }
    }
}

// const pinocchio::SE3 iMd = data.oMi[JOINT_ID].actInv(oMdes);
// this calls oMdes.se3ActionInverse(data.oMi[JOINT_ID]);
// rot.transpose() * m2.rotation(), rot.transpose() * (m2.translation() - translation()));
SE3 actInv(SE3 oMdes, SE3 oMi){
    //double oMdes_rot_transpose[3][3] = {
    //    {oMdes.rotation[0][0], oMdes.rotation[1][0], oMdes.rotation[2][0]},
    //    {oMdes.rotation[0][1], oMdes.rotation[1][1], oMdes.rotation[2][1]},
    //    {oMdes.rotation[0][2], oMdes.rotation[1][2], oMdes.rotation[2][2]}
    //};
    double oMi_rot_transpose[3][3] = {
        {oMi.rotation[0][0], oMi.rotation[1][0], oMi.rotation[2][0]},
        {oMi.rotation[0][1], oMi.rotation[1][1], oMi.rotation[2][1]},
        {oMi.rotation[0][2], oMi.rotation[1][2], oMi.rotation[2][2]}
    };
    printf("oMi_rot_transpose:\n");
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            printf("%f ", oMi_rot_transpose[i][j]);
        }
        printf("\n");
    }
    printf("oMdes.rotation:\n");
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            printf("%f ", oMdes.rotation[i][j]);
        }
        printf("\n");
    }


    // multiply oMdes_rot_transpose with oMi.rotation
    double result_rot[3][3];

    matrix_mult_3x3(oMi_rot_transpose, oMdes.rotation, result_rot);

    // rot.transpose() * (m2.translation() - translation())
    double right_side[3] = {
        oMdes.translation[0] - oMi.translation[0],
        oMdes.translation[1] - oMi.translation[1],
        oMdes.translation[2] - oMi.translation[2]
    };

    double result_trans[3];

    matrix_mult_3x1(oMi_rot_transpose, right_side, result_trans);

    SE3 result;
    result.translation[0] = result_trans[0];
    result.translation[1] = result_trans[1];
    result.translation[2] = result_trans[2];

    result.rotation[0][0] = result_rot[0][0];
    result.rotation[0][1] = result_rot[0][1];
    result.rotation[0][2] = result_rot[0][2];
    result.rotation[1][0] = result_rot[1][0];
    result.rotation[1][1] = result_rot[1][1];
    result.rotation[1][2] = result_rot[1][2];
    result.rotation[2][0] = result_rot[2][0];
    result.rotation[2][1] = result_rot[2][1];
    result.rotation[2][2] = result_rot[2][2];

    return result;
}





















#endif // INVERSE_KINEMATICS_HELPER_H