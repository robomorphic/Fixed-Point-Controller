#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#include "forward_kinematics_panda.h"
#include "inverse_kinematics_helper.h"




int main(){

    // const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1., 0., 1.));
    SE3 oMdes = {
        .translation = {0.5, 0.5, 0.5},
        .rotation = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}
        }
    };

    // Eigen::VectorXd q = randomConfiguration(model);
    joint_position q = {
        .position = {    1.97125,      -0.372364,      1.64045,      -0.674883,      2.38533,      0.727269,      -0.954818},
        .cos =      {cos(1.97125), cos(-0.372364), cos(1.64045), cos(-0.674883), cos(2.38533), cos(0.727269), cos(-0.954818)},
        .sin =      {sin(1.97125), sin(-0.372364), sin(1.64045), sin(-0.674883), sin(2.38533), sin(0.727269), sin(-0.954818)}
    };
    

    // pinocchio::Data::Matrix6x J(6, model.nv); 
    Matrix6x J = {0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < NU; j++) {
            J.data[i][j] = 0.0;
        }
    }

    bool success = false;

    //Vector6d err;
    Vector6d err = {0};
    for (int i = 0; i < 6; i++) {
        err.data[i] = 0.0;
    }

    //Eigen::VectorXd v(model.nv);
    VectorXd v = {0};
    for (int i = 0; i < NU; i++) {
        v.data[i] = 0.0;
    }

    for(int i = 0;; i++){
        // pinocchio::forwardKinematics(model, data, q);
        SE3s res = ForwardKinematics(
            q.cos[0], q.cos[1], q.cos[2], q.cos[3], q.cos[4], q.cos[5], q.cos[6],
            q.sin[0], q.sin[1], q.sin[2], q.sin[3], q.sin[4], q.sin[5], q.sin[6],
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0
        );
        //for(int j = 0; j < NU; j++){
        //    for (int k = 0; k < 3; k++) {
        //        for(int l = 0; l < 3; l++) {
        //            printf("%f ", res.SE3[j].rotation[k][l]);
        //        }
        //        printf("\n");
        //    }
        //    printf("translation: %f %f %f\n", res.SE3[j].translation[0], res.SE3[j].translation[1], res.SE3[j].translation[2]);
        //
        //    printf("\n");
        //}


        // const pinocchio::SE3 iMd = data.oMi[JOINT_ID].actInv(oMdes);
        SE3 iMd = actInv(oMdes, res.SE3[NU - 1]);
        printf("iMd translation: %f %f %f\n", iMd.translation[0], iMd.translation[1], iMd.translation[2]);
        printf("iMd rotation: %f %f %f\n %f %f %f\n %f %f %f\n",
            iMd.rotation[0][0], iMd.rotation[0][1], iMd.rotation[0][2],
            iMd.rotation[1][0], iMd.rotation[1][1], iMd.rotation[1][2],
            iMd.rotation[2][0], iMd.rotation[2][1], iMd.rotation[2][2]
        );

        


        break;







    }













}



