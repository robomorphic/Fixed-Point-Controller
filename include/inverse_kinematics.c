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
        SE3s fk = ForwardKinematics(
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
        SE3 iMd = actInv(oMdes, fk.SE3[NU - 1]);
        //printf("iMd translation: %f %f %f\n", iMd.translation[0], iMd.translation[1], iMd.translation[2]);
        //printf("iMd rotation: %f %f %f\n %f %f %f\n %f %f %f\n",
        //    iMd.rotation[0][0], iMd.rotation[0][1], iMd.rotation[0][2],
        //    iMd.rotation[1][0], iMd.rotation[1][1], iMd.rotation[1][2],
        //    iMd.rotation[2][0], iMd.rotation[2][1], iMd.rotation[2][2]
        //);

        // err = pinocchio::log6(iMd).toVector(); // in joint frame
        log6_SE3(&iMd, err.data);
        //printf("Error vector (log6): ");
        //for (int k = 0; k < 6; k++){
        //    printf("%f ", err.data[k]);
        //}
        //printf("\n");

        //if (err.norm() < eps)
        //{
        //  success = true;
        //  break;
        //}
        //if (i >= IT_MAX)
        //{
        //  success = false;
        //  break;
        //}
        double err_norm = 0;
        for (int k = 0; k < 6; k++){
            err_norm += err.data[k] * err.data[k];
        }
        err_norm = sqrt(err_norm);
        //printf("Error norm: %f\n", err_norm);
        if (err_norm < eps)
        {
            success = true;
            break;
        }
        if (i >= IT_MAX)
        {
            success = false;
            break;
        }

        // pinocchio::computeJointJacobian(model, data, q, JOINT_ID, J); // J in joint frame
        computeJointJacobian(&fk, NU - 1, &J);
        //printf("Jacobian:\n");
        //for (int k = 0; k < 6; k++){
        //    for (int l = 0; l < NU; l++){
        //        printf("%f ", J.data[k][l]);
        //    }
        //    printf("\n");
        //}

        //pinocchio::Data::Matrix6 Jlog;
        Matrix6x6 Jlog = {0};
        for (int k = 0; k < 6; k++) {
            for (int l = 0; l < 6; l++) {
                Jlog.data[k][l] = 0.0;
            }
        }

        // pinocchio::Jlog6(iMd.inverse(), Jlog);
        SE3 iMd_inv = inverse_SE3(&iMd);
        //printf("iMd_inv translation: %f %f %f\n", iMd_inv.translation[0], iMd_inv.translation[1], iMd_inv.translation[2]);
        //printf("iMd_inv rotation: %f %f %f\n %f %f %f\n %f %f %f\n",
        //    iMd_inv.rotation[0][0], iMd_inv.rotation[0][1], iMd_inv.rotation[0][2],
        //    iMd_inv.rotation[1][0], iMd_inv.rotation[1][1], iMd_inv.rotation[1][2],
        //    iMd_inv.rotation[2][0], iMd_inv.rotation[2][1], iMd_inv.rotation[2][2]
        //);
        Jlog6(&iMd_inv, Jlog.data);
        //printf("Jlog:\n");
        //for (int k = 0; k < 6; k++){
        //    for (int l = 0; l < 6; l++){
        //        printf("%f ", Jlog.data[k][l]);
        //    }
        //    printf("\n");
        //}

        // J = -Jlog * J;
        {
            int i, j, k;
            double newJ[6][NU];
            // Compute newJ = Jlog * J, then negate.
            for (i = 0; i < 6; i++) {
                for (j = 0; j < NU; j++) {
                    newJ[i][j] = 0.0;
                    for (k = 0; k < 6; k++) {
                        newJ[i][j] += Jlog.data[i][k] * J.data[k][j];
                    }
                    newJ[i][j] = -newJ[i][j];
                }
            }
            // Copy the result back into J.
            for (i = 0; i < 6; i++) {
                for (j = 0; j < NU; j++) {
                    J.data[i][j] = newJ[i][j];
                }
            }
        }
        //printf("Updated Jacobian:\n");
        //for (int k = 0; k < 6; k++){
        //    for (int l = 0; l < NU; l++){
        //        printf("%f ", J.data[k][l]);
        //    }
        //    printf("\n");
        //}

        // pinocchio::Data::Matrix6 JJt;
        Matrix6x6 JJt = {0};
        for (int k = 0; k < 6; k++) {
            for (int l = 0; l < 6; l++) {
                JJt.data[k][l] = 0.0;
            }
        }

        // JJt.noalias() = J * J.transpose();
        // Compute JJt = J * J^T and store directly into the global JJt variable.
        for (int i = 0; i < 6; i++){
            for (int j = 0; j < 6; j++){
                JJt.data[i][j] = 0.0;
                for (int k = 0; k < NU; k++){
                    JJt.data[i][j] += J.data[i][k] * J.data[j][k];
                }
            }
        }

        // JJt.diagonal().array() += damp;
        for (int k = 0; k < 6; k++){
            JJt.data[k][k] += damp;
        }

        // v.noalias() = -J.transpose() * JJt.ldlt().solve(err);

        // Then, in your IK loop after computing JJt and adding damping to its diagonal,
        // solve for 'x' and compute v = -Jᵀ*x as follows:
        {
            double x[6];
            // Solve (JJt)x = err.data
            ldlt_solve_6(&JJt, err.data, x);

            // Compute v = -Jᵀ * x. (J is 6xNU; v is NU-dimensional.)
            for (int j = 0; j < NU; j++) {
                double sum = 0.0;
                for (int i = 0; i < 6; i++) {
                    sum += J.data[i][j] * x[i];
                }
                v.data[j] = -sum;
            }
        }
        //printf("v:\n");
        //for (int k = 0; k < NU; k++){
        //    printf("%f ", v.data[k]);
        //}

        // q = pinocchio::integrate(model, q, v * DT);
        q = integrate(&q, &v);

        //printf("Updated q:\n");
        //for (int k = 0; k < NU; k++){
        //    printf("%f ", q.position[k]);
        //}

        if(i % 10 == 0){
            printf("Iteration %d err: %f \n", i, err_norm);
        }

    }

    if (success)
    {
        printf("Success!");
        printf("Final q: ");
        for (int k = 0; k < NU; k++){
            printf("%f ", q.position[k]);
        }
    }
    else
    {
        printf("Failed to converge.");
        printf("Final q: ");
        for (int k = 0; k < NU; k++){
            printf("%f ", q.position[k]);
        }
    }













}



