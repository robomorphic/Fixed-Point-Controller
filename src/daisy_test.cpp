#include <math.h>
#include <FixedPoint/fixed_point.hpp> 
#include <mujoco_exec_helper.hpp>

//ap_fixed is <total_bits, integer_bits>
// fixed_point is <integer_bits, fractional_bits>
// convert it with define or typedef

template<int TOTAL_BITS, int INT_BITS>
using ap_fixed = FixedPoint<INT_BITS, (TOTAL_BITS - INT_BITS)>;


/* @pre: ((m > 0.0) && (m < 5.0) && (x_lever > -0.12) && (y_lever > -0.12) && (z_lever > -0.12) && (x_lever < 0.18) && (y_lever < 0.18) && (z_lever < 0.18) && (joint_inertia_1 > -0.03) && (joint_inertia_2 > -0.03) && (joint_inertia_3 > -0.03) && (joint_inertia_4 > -0.03) && (joint_inertia_5 > -0.03) && (joint_inertia_6 > -0.03) && (joint_inertia_1 < 1.2) && (joint_inertia_2 < 1.2) && (joint_inertia_3 < 1.2) && (joint_inertia_4 < 1.2) && (joint_inertia_5 < 1.2) && (joint_inertia_6 < 1.2)) */
/* @post: (res) => (res +/- 0.01) */
ap_fixed<32,9> firstPass(ap_fixed<32,9> model_joint_p_rotation_1_1_1, ap_fixed<32,9> model_joint_p_rotation_1_1_2, ap_fixed<32,9> model_joint_p_rotation_1_1_3, ap_fixed<32,9> model_joint_p_rotation_1_2_1, ap_fixed<32,9> model_joint_p_rotation_1_2_2, ap_fixed<32,9> model_joint_p_rotation_1_2_3, ap_fixed<32,9> model_joint_p_rotation_1_3_1, ap_fixed<32,9> model_joint_p_rotation_1_3_2, ap_fixed<32,9> model_joint_p_rotation_1_3_3, ap_fixed<32,9> model_joint_p_translation_1_1, ap_fixed<32,9> model_joint_p_translation_1_2, ap_fixed<32,9> model_joint_p_translation_1_3, ap_fixed<32,9> qpos1) {
  ap_fixed<32,9> sin_qpos1 = sin(qpos1);
  ap_fixed<32,9> cos_qpos1 = cos(qpos1);
  ap_fixed<32,9> rotation_matrix_1_1_1 = cos_qpos1;
  ap_fixed<32,9> rotation_matrix_1_1_2 = sin_qpos1;
  ap_fixed<32,9> rotation_matrix_1_2_1 = -(sin_qpos1);
  ap_fixed<32,9> rotation_matrix_1_2_2 = cos_qpos1;
  ap_fixed<32,9> _tmp = (model_joint_p_rotation_1_1_2 * rotation_matrix_1_2_1);
  ap_fixed<32,9> _tmp1 = (model_joint_p_rotation_1_1_1 * rotation_matrix_1_1_1);
  ap_fixed<32,9> limi_rotation_1_1_1 = (_tmp + _tmp1);
  ap_fixed<32,9> _tmp2 = (model_joint_p_rotation_1_1_2 * rotation_matrix_1_2_2);
  ap_fixed<32,9> _tmp3 = (model_joint_p_rotation_1_1_1 * rotation_matrix_1_1_2);
  ap_fixed<32,9> limi_rotation_1_1_2 = (_tmp2 + _tmp3);
  ap_fixed<32,9> limi_rotation_1_1_3 = model_joint_p_rotation_1_1_3;
  ap_fixed<32,9> _tmp4 = (rotation_matrix_1_1_1 * model_joint_p_rotation_1_2_1);
  ap_fixed<32,9> _tmp5 = (model_joint_p_rotation_1_2_2 * rotation_matrix_1_2_1);
  ap_fixed<32,9> limi_rotation_1_2_1 = (_tmp4 + _tmp5);
  ap_fixed<32,9> _tmp6 = (model_joint_p_rotation_1_2_1 * rotation_matrix_1_1_2);
  ap_fixed<32,9> _tmp7 = (rotation_matrix_1_2_2 * model_joint_p_rotation_1_2_2);
  ap_fixed<32,9> limi_rotation_1_2_2 = (_tmp6 + _tmp7);
  ap_fixed<32,9> limi_rotation_1_2_3 = model_joint_p_rotation_1_2_3;
  ap_fixed<32,9> _tmp8 = (rotation_matrix_1_1_1 * model_joint_p_rotation_1_3_1);
  ap_fixed<32,9> _tmp9 = (rotation_matrix_1_2_1 * model_joint_p_rotation_1_3_2);
  ap_fixed<32,9> limi_rotation_1_3_1 = (_tmp8 + _tmp9);
  ap_fixed<32,9> _tmp10 = (rotation_matrix_1_2_2 * model_joint_p_rotation_1_3_2);
  ap_fixed<32,9> _tmp11 = (model_joint_p_rotation_1_3_1 * rotation_matrix_1_1_2);
  ap_fixed<32,9> limi_rotation_1_3_2 = (_tmp10 + _tmp11);
  ap_fixed<32,9> limi_rotation_1_3_3 = model_joint_p_rotation_1_3_3;
  ap_fixed<32,9> limi_translation_1_1 = model_joint_p_translation_1_1;
  ap_fixed<32,9> limi_translation_1_2 = model_joint_p_translation_1_2;
  ap_fixed<32,9> limi_translation_1_3 = model_joint_p_translation_1_3;
  ap_fixed<32,9> oMi_rotation_1_1_1 = limi_rotation_1_1_1;
  ap_fixed<32,9> oMi_rotation_1_1_2 = limi_rotation_1_1_2;
  ap_fixed<32,9> oMi_rotation_1_1_3 = limi_rotation_1_1_3;
  ap_fixed<32,9> oMi_rotation_1_2_1 = limi_rotation_1_2_1;
  ap_fixed<32,9> oMi_rotation_1_2_2 = limi_rotation_1_2_2;
  ap_fixed<32,9> oMi_rotation_1_2_3 = limi_rotation_1_2_3;
  ap_fixed<32,9> oMi_rotation_1_3_1 = limi_rotation_1_3_1;
  ap_fixed<32,9> oMi_rotation_1_3_2 = limi_rotation_1_3_2;
  ap_fixed<32,9> oMi_rotation_1_3_3 = limi_rotation_1_3_3;
  ap_fixed<32,9> oMi_translation_1_1 = limi_translation_1_1;
  ap_fixed<32,9> oMi_translation_1_2 = limi_translation_1_2;
  ap_fixed<32,9> oMi_translation_1_3 = limi_translation_1_3;
  ap_fixed<32,9> _tmp12 = (oMi_rotation_1_1_2 + oMi_rotation_1_1_3);
  ap_fixed<32,9> _tmp13 = (oMi_rotation_1_1_1 + _tmp12);
  ap_fixed<32,9> _tmp15 = (_tmp13 + oMi_rotation_1_2_1);
  ap_fixed<32,9> _tmp14 = (oMi_rotation_1_2_3 + oMi_rotation_1_3_1);
  ap_fixed<32,9> _tmp16 = (oMi_rotation_1_2_2 + _tmp14);
  ap_fixed<32,9> _tmp20 = (_tmp15 + _tmp16);
  ap_fixed<32,9> _tmp18 = (oMi_rotation_1_3_2 + oMi_rotation_1_3_3);
  ap_fixed<32,9> _tmp17 = (oMi_translation_1_1 + oMi_translation_1_2);
  ap_fixed<32,9> _tmp19 = (_tmp17 + oMi_translation_1_3);
  ap_fixed<32,9> _tmp21 = (_tmp18 + _tmp19);

    //std::cout << fi_1_1 << " " << 0 << " " << 0 << " " << 0 << " " << se_1_2 << " " << se_1_3 << std::endl;
    //std::cout << 0 << " " << fi_2_2 << " " << 0 << " " << se_2_1 << " " << 0 << " " << se_2_3 << std::endl;
    //std::cout << 0 << " " << 0 << " " << fi_3_3 << " " << se_3_1 << " " << se_3_2 << " " << 0 << std::endl;
    //std::cout << 0 << " " << th_1_2 << " " << th_1_3 << " " << fo_1_1 << " " << fo_1_2 << " " << fo_1_3 << std::endl;
    //std::cout << th_2_1 << " " << 0 << " " << th_2_3 << " " << fo_2_1 << " " << fo_2_2 << " " << fo_2_3 << std::endl;
    //std::cout << th_3_1 << " " << th_3_2 << " " << 0 << " " << fo_3_1 << " " << fo_3_2 << " " << fo_3_3 << std::endl;
    std::cout << oMi_rotation_1_1_1 << " " << oMi_rotation_1_1_2 << " " << oMi_rotation_1_1_3 << std::endl;
    std::cout << oMi_rotation_1_2_1 << " " << oMi_rotation_1_2_2 << " " << oMi_rotation_1_2_3 << std::endl;
    std::cout << oMi_rotation_1_3_1 << " " << oMi_rotation_1_3_2 << " " << oMi_rotation_1_3_3 << std::endl;
    std::cout << oMi_translation_1_1 << " " << oMi_translation_1_2 << " " << oMi_translation_1_3 << std::endl;
  return (_tmp20 + _tmp21);
} // [-0.27285183519401485, 28.50581702579514] +/- 0.009724890703458186

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int main(int argc, char* argv[]){
    // JOINT 1
    //"joint_rotation": "1.000 0.000 0.000\n0.000 1.000 0.000\n0.000 0.000 1.000\n"
    //"joint_translation": "0.000\n0.000\n0.333\n"
    //"qpos": "-2.8973\n"
    firstPass(
        // rotation joint 1
        ap_fixed<32,9>(1.0),
        ap_fixed<32,9>(0.0),
        ap_fixed<32,9>(0.0),
        ap_fixed<32,9>(0.0),
        ap_fixed<32,9>(1.0),
        ap_fixed<32,9>(0.0),
        ap_fixed<32,9>(0.0),
        ap_fixed<32,9>(0.0),
        ap_fixed<32,9>(1.0),

        // translation joint 1
        ap_fixed<32,9>(0.0),
        ap_fixed<32,9>(0.0),
        ap_fixed<32,9>(0.333),
    
        // qpos joint 1
        ap_fixed<32,9>(-2.8973)

    );

    /*
    4.97068            0            0           -0    -0.236704    -0.010344          
    0      4.97068            0     0.236704           -0    0.0192614
    0            0      4.97068     0.010344   -0.0192614           -0
    0     0.236704     0.010344             0.714663        -0.000179083    0.00768923
    -0.236704            0   -0.0192614     -0.000179083     0.717956       0.0196616
    -0.010344    0.0192614            0     0.00768923      0.0196616       0.00921316"
    */
}