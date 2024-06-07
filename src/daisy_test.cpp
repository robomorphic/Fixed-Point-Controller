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
ap_fixed<32,9> first_pass_inertia(ap_fixed<32,9> m, ap_fixed<32,9> x_lever, ap_fixed<32,9> y_lever, ap_fixed<32,9> z_lever, ap_fixed<32,9> joint_inertia_1, ap_fixed<32,9> joint_inertia_2, ap_fixed<32,9> joint_inertia_3, ap_fixed<32,9> joint_inertia_4, ap_fixed<32,9> joint_inertia_5, ap_fixed<32,9> joint_inertia_6) {
  ap_fixed<32,9> fi_1_1 = m;
  ap_fixed<32,9> fi_2_2 = m;
  ap_fixed<32,9> fi_3_3 = m;
  ap_fixed<32,9> se_1_2 = (z_lever * m);
  ap_fixed<32,9> _tmp = -(m);
  ap_fixed<32,9> se_1_3 = (_tmp * y_lever);
  ap_fixed<32,9> se_2_1 = -(se_1_2);
  ap_fixed<32,9> se_2_3 = (m * x_lever);
  ap_fixed<32,9> se_3_1 = -(se_1_3);
  ap_fixed<32,9> se_3_2 = -(se_2_3);
  ap_fixed<32,9> th_1_2 = -(se_1_2);
  ap_fixed<32,9> th_1_3 = -(se_1_3);
  ap_fixed<32,9> th_2_1 = -(se_2_1);
  ap_fixed<32,9> th_2_3 = -(se_2_3);
  ap_fixed<32,9> th_3_1 = -(se_3_1);
  ap_fixed<32,9> th_3_2 = -(se_3_2);
  ap_fixed<32,9> _tmp1 = (y_lever * y_lever);
  ap_fixed<32,9> _tmp2 = (z_lever * z_lever);
  ap_fixed<32,9> _tmp3 = (_tmp1 + _tmp2);
  ap_fixed<32,9> _tmp4 = (m * _tmp3);
  ap_fixed<32,9> fo_1_1 = (joint_inertia_1 + _tmp4);
  ap_fixed<32,9> _tmp5 = (m * x_lever);
  ap_fixed<32,9> _tmp6 = (y_lever * _tmp5);
  ap_fixed<32,9> fo_1_2 = (joint_inertia_2 - _tmp6);
  ap_fixed<32,9> _tmp7 = (x_lever * x_lever);
  ap_fixed<32,9> _tmp8 = (z_lever * z_lever);
  ap_fixed<32,9> _tmp9 = (_tmp7 + _tmp8);
  ap_fixed<32,9> _tmp10 = (m * _tmp9);
  ap_fixed<32,9> fo_1_3 = (joint_inertia_4 - _tmp10);
  ap_fixed<32,9> fo_2_1 = fo_1_2;
  ap_fixed<32,9> _tmp11 = (m * x_lever);
  ap_fixed<32,9> _tmp12 = (_tmp11 * z_lever);
  ap_fixed<32,9> fo_2_2 = (_tmp12 + joint_inertia_3);
  ap_fixed<32,9> _tmp13 = (m * y_lever);
  ap_fixed<32,9> _tmp14 = (_tmp13 * z_lever);
  ap_fixed<32,9> fo_2_3 = (joint_inertia_5 - _tmp14);
  ap_fixed<32,9> fo_3_1 = fo_1_3;
  ap_fixed<32,9> fo_3_2 = fo_2_3;
  ap_fixed<32,9> _tmp15 = (y_lever * y_lever);
  ap_fixed<32,9> _tmp16 = (x_lever * x_lever);
  ap_fixed<32,9> _tmp17 = (_tmp15 + _tmp16);
  ap_fixed<32,9> _tmp18 = (m * _tmp17);
  ap_fixed<32,9> fo_3_3 = (joint_inertia_6 + _tmp18);
  ap_fixed<32,9> _tmp19 = (fi_1_1 + fi_2_2);
  ap_fixed<32,9> _tmp20 = (_tmp19 + fi_3_3);
  ap_fixed<32,9> _tmp23 = (_tmp20 + se_1_2);
  ap_fixed<32,9> _tmp21 = (se_2_3 + se_3_1);
  ap_fixed<32,9> _tmp22 = (se_2_1 + _tmp21);
  ap_fixed<32,9> _tmp24 = (se_1_3 + _tmp22);
  ap_fixed<32,9> _tmp25 = (_tmp23 + _tmp24);
  ap_fixed<32,9> _tmp26 = (se_3_2 + _tmp25);
  ap_fixed<32,9> _tmp27 = (_tmp26 + th_1_2);
  ap_fixed<32,9> _tmp28 = (th_1_3 + th_2_1);
  ap_fixed<32,9> _tmp39 = (_tmp27 + _tmp28);
  ap_fixed<32,9> _tmp29 = (th_2_3 + th_3_1);
  ap_fixed<32,9> _tmp30 = (th_3_2 + fo_1_1);
  ap_fixed<32,9> _tmp37 = (_tmp29 + _tmp30);
  ap_fixed<32,9> _tmp31 = (fo_1_3 + fo_2_1);
  ap_fixed<32,9> _tmp34 = (_tmp31 + fo_2_2);
  ap_fixed<32,9> _tmp32 = (fo_2_3 + fo_3_1);
  ap_fixed<32,9> _tmp33 = (fo_3_3 + fo_3_2);
  ap_fixed<32,9> _tmp35 = (_tmp32 + _tmp33);
  ap_fixed<32,9> _tmp36 = (_tmp34 + _tmp35);
  ap_fixed<32,9> _tmp38 = (_tmp36 + fo_1_2);
  ap_fixed<32,9> _tmp40 = (_tmp37 + _tmp38);

    std::cout << fi_1_1 << " " << 0 << " " << 0 << " " << 0 << " " << se_1_2 << " " << se_1_3 << std::endl;
    std::cout << 0 << " " << fi_2_2 << " " << 0 << " " << se_2_1 << " " << 0 << " " << se_2_3 << std::endl;
    std::cout << 0 << " " << 0 << " " << fi_3_3 << " " << se_3_1 << " " << se_3_2 << " " << 0 << std::endl;
    std::cout << 0 << " " << th_1_2 << " " << th_1_3 << " " << fo_1_1 << " " << fo_1_2 << " " << fo_1_3 << std::endl;
    std::cout << th_2_1 << " " << 0 << " " << th_2_3 << " " << fo_2_1 << " " << fo_2_2 << " " << fo_2_3 << std::endl;
    std::cout << th_3_1 << " " << th_3_2 << " " << 0 << " " << fo_3_1 << " " << fo_3_2 << " " << fo_3_3 << std::endl;
  return (_tmp39 + _tmp40);
} // [-0.27285183519401485, 28.50581702579514] +/- 0.009724890703458186

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int main(int argc, char* argv[]){
    first_pass_inertia(
        ap_fixed<32,9>(4.97068),

        ap_fixed<32,9>(0.003875),
        ap_fixed<32,9>(0.002081),
        ap_fixed<32,9>(-0.04762),
        
        ap_fixed<32,9>(0.70337),
        ap_fixed<32,9>(-0.000139),
        ap_fixed<32,9>(0.70661),
        ap_fixed<32,9>(0.006772),
        ap_fixed<32,9>(0.019169),
        ap_fixed<32,9>(0.009117)
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