#include "pinocchio/parsers/urdf.hpp"
 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"
 
#include <iostream>
 
int main(int argc, char ** argv)
{
  using namespace pinocchio;
  
  const std::string urdf_filename = (argc<=1) ? std::string("../models/panda.urdf") : argv[1];
  
  // Load the urdf model
  Model model;
  pinocchio::urdf::buildModel(urdf_filename,model);
  std::cout << "model name: " << model.name << std::endl;
  
  // Create data required by the algorithms
  Data data(model);
  
  // Sample a random configuration
  Eigen::VectorXd q = randomConfiguration(model);
  std::cout << "q: " << q.transpose() << std::endl;
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
 
  // Allocate result container
  Eigen::MatrixXd djoint_acc_dq = Eigen::MatrixXd::Zero(model.nv,model.nv);
  Eigen::MatrixXd djoint_acc_dv = Eigen::MatrixXd::Zero(model.nv,model.nv);
  Eigen::MatrixXd djoint_acc_dtau = Eigen::MatrixXd::Zero(model.nv,model.nv);
  
  // Computes the forward dynamics (ABA) derivatives for all the joints of the robot
  computeABADerivatives(model, data, q, v, tau, djoint_acc_dq, djoint_acc_dv, djoint_acc_dtau);
  computeGeneralizedGravity(model, data, q);
  
  // Get access to the joint acceleration
  std::cout << "Joint acceleration: " << data.ddq.transpose() << std::endl;

  std::cout << "Curr gravity: " << data.g.transpose() << std::endl;
}





