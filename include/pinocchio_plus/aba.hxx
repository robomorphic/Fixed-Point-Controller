//
// Copyright (c) 2016-2020 CNRS, INRIA
//

#ifndef __pinocchio_plus_aba_hxx__
#define __pinocchio_plus_aba_hxx__

#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/algorithm/check.hpp"
#include "config.hpp"

#define PRINT_MATRIX(x, of) for(int i = 0; i < x.rows(); i++) { for(int j = 0; j < x.cols(); j++) { of << std::setprecision(4) << x(i, j) << " "; } of << std::endl; }

/// @cond DEV
// bad, very bad :(
using namespace pinocchio;


namespace internalVerbose
  {

    template<typename Scalar>
    struct SE3actOnVerbose
    {
      template<int Options, typename Matrix6Type>
      static typename PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix6Type)
      run(const SE3Tpl<Scalar,Options> & M,
          const Eigen::MatrixBase<Matrix6Type> & I)
      {
        typedef SE3Tpl<Scalar,Options> SE3;
        typedef typename SE3::Matrix3 Matrix3;
        typedef typename SE3::Vector3 Vector3;

        typedef const Eigen::Block<Matrix6Type,3,3> constBlock3;

        typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix6Type) ReturnType;
        typedef Eigen::Block<ReturnType,3,3> Block3;

        Matrix6Type & I_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6Type,I);
        const constBlock3 & Ai = I_.template block<3,3>(Inertia::LINEAR, Inertia::LINEAR);
        const constBlock3 & Bi = I_.template block<3,3>(Inertia::LINEAR, Inertia::ANGULAR);
        const constBlock3 & Di = I_.template block<3,3>(Inertia::ANGULAR, Inertia::ANGULAR);

        const Matrix3 & R = M.rotation();
        const Vector3 & t = M.translation();

        ReturnType res;
        Block3 Ao = res.template block<3,3>(Inertia::LINEAR, Inertia::LINEAR);
        Block3 Bo = res.template block<3,3>(Inertia::LINEAR, Inertia::ANGULAR);
        Block3 Co = res.template block<3,3>(Inertia::ANGULAR, Inertia::LINEAR);
        Block3 Do = res.template block<3,3>(Inertia::ANGULAR, Inertia::ANGULAR);

        Do.noalias() = R*Ai; // tmp variable
        Ao.noalias() = Do*R.transpose();

        Do.noalias() = R*Bi; // tmp variable
        Bo.noalias() = Do*R.transpose();

        Co.noalias() = R*Di; // tmp variable
        Do.noalias() = Co*R.transpose();

        Do.row(0) += t.cross(Bo.col(0));
        Do.row(1) += t.cross(Bo.col(1));
        Do.row(2) += t.cross(Bo.col(2));

        Co.col(0) = t.cross(Ao.col(0));
        Co.col(1) = t.cross(Ao.col(1));
        Co.col(2) = t.cross(Ao.col(2));
        Co += Bo.transpose();

        Bo = Co.transpose();
        Do.col(0) += t.cross(Bo.col(0));
        Do.col(1) += t.cross(Bo.col(1));
        Do.col(2) += t.cross(Bo.col(2));

        return res;
      }
    };

  }

namespace pinocchioPass
{
  /// A very simple pass
  /// We are calculating the relative and absolute joint placements
  /// And calculate each joint's inertia matrix
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  struct ComputeMinverseForwardStep1Verbose
  : public fusion::JointUnaryVisitorBase< ComputeMinverseForwardStep1Verbose<Scalar,Options,JointCollectionTpl,ConfigVectorType> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;

    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  const ConfigVectorType &
                                  > ArgsType;

    template<typename JointModel>
    static void algo(const pinocchio::JointModelBase<JointModel> & jmodel,
                     pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data,
                     const Eigen::MatrixBase<ConfigVectorType> & q)
    {

      std::string folder_path = model_output_foldername + "ABA/" + std::to_string(CONTROLLER_ABA_PRINT_INDEX) + "/";
      std::filesystem::create_directories(folder_path);
      std::string pass1_file_path = folder_path + "pass1.txt";
      std::ofstream pass1_file;
      pass1_file.open(pass1_file_path, std::ios::app);


      typedef typename Model::JointIndex JointIndex;
      const JointIndex & i = jmodel.id();
      pass1_file << "Joint: " << i << std::endl;
      pass1_file << "model.lever[" << i << "]: " << std::endl;
      pass1_file << model.inertias[i].lever() << std::endl;
      pass1_file << "model.inertia[" << i << "]_before_pass1: " << std::endl;
      pass1_file << model.inertias[i] << std::endl;
      pass1_file << "model.inertia[" << i << "]_before_pass1_matrix: " << std::endl;
      pass1_file << model.inertias[i].matrix() << std::endl;
      //this one is important
      auto old_buf = std::cout.rdbuf();
      std::cout.rdbuf(pass1_file.rdbuf());
      jmodel.calc(jdata.derived(),q.derived());
      std::cout.rdbuf(old_buf);

      // calculate the vector of relative joint placements (w.r.t. the body parent)
      const JointIndex & parent = model.parents[i];
      data.liMi[i] = model.jointPlacements[i] * jdata.M();
      pass1_file << "Relative_joint_placement: " << std::endl;
      pass1_file << data.liMi[i];

      // calculate the absolute joint placements
      if (parent>0)
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
      else
        data.oMi[i] = data.liMi[i];
      pass1_file << "Absolute_joint_placement: " << std::endl;
      pass1_file << data.oMi[i];

      /*
      // This seems to be unnecessary. Opened an issue for this.
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;
      ColsBlock J_cols = jmodel.jointCols(data.J);
      pass1_file << "model.inertias[i].matrix()_before: \n" << model.inertias[i].matrix() << std::endl;
      pass1_file << "J_cols_first_pass: \n" << J_cols << std::endl;
      pass1_file << "data.oMi[i]: \n" << data.oMi[i] << std::endl;
      //pass1_file << "jdata.S(): " << jdata.S() << std::endl;
      J_cols = data.oMi[i].act(jdata.S());
      pass1_file << "J_cols_second_pass: \n" << J_cols << std::endl;
      pass1_file << "data.oMi[i]_new_pass: \n" << data.oMi[i] << std::endl;
      //pass1_file << "jdata.S()_new_pass: " << jdata.S() << std::endl;
      pass1_file << "model.inertias[i].matrix()_after: \n" << model.inertias[i].matrix() << std::endl;
      */

      // Inertia matrix of the "subtree" expressed as dense matrix
      // I currently don't know where the inertias are calculated, but I know it is pretty easy to calculate them
      data.Yaba[i] = model.inertias[i].matrix();
    }

  };

  /// According to Featherstone, this pass is where the articulated-body inertias and bias forces
  /// are calculated. jmodel.calc_aba() is a main part of this pass.
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  struct ComputeMinverseBackwardStepVerbose
  : public fusion::JointUnaryVisitorBase< ComputeMinverseBackwardStepVerbose<Scalar,Options,JointCollectionTpl> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;

    typedef boost::fusion::vector<const Model &,
                                  Data &> ArgsType;

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {

      std::string folder_path = model_output_foldername + "ABA/" + std::to_string(CONTROLLER_ABA_PRINT_INDEX) + "/";
      std::filesystem::create_directories(folder_path);
      std::string pass2_file_path = folder_path + "pass2.txt";
      std::ofstream pass2_file;
      pass2_file.open(pass2_file_path, std::ios::app);

      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Inertia Inertia;

      const JointIndex & i = jmodel.id();
      pass2_file << "Joint: " << i << std::endl;
      const JointIndex & parent  = model.parents[i];

      typename Inertia::Matrix6 & Ia = data.Yaba[i];
      typename Data::RowMatrixXs & Minv = data.Minv;
      typename Data::Matrix6x & Fcrb = data.Fcrb[0];
      typename Data::Matrix6x & FcrbTmp = data.Fcrb.back();

      pass2_file << "Fcrb_before_pass2: " << std::endl;
      pass2_file << Fcrb << std::endl;
      pass2_file << "Minv_before_pass2: " << std::endl;
      pass2_file << Minv << std::endl;
      pass2_file << "Ia_before_pass2: " << std::endl;
      pass2_file << Ia << std::endl;
      // Inverse ABA
      auto old_buf = std::cout.rdbuf();
      std::cout.rdbuf(pass2_file.rdbuf());
      jmodel.calc_aba(jdata.derived(), Ia, parent > 0);
      std::cout.rdbuf(old_buf);

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;

      // What is the purpose of this?
      // Unknown part 2
      ColsBlock U_cols = jmodel.jointCols(data.IS);
      forceSet::se3Action(data.oMi[i],jdata.U(),U_cols); // expressed in the world frame

      // Block of size (p,q), starting at (i,j) matrix.block(i,j,p,q);
      Minv.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),jmodel.nv()) = jdata.Dinv();
      pass2_file << "U_cols: \n" << U_cols << std::endl;
      pass2_file << "idx_v: \n" << jmodel.idx_v() << std::endl;
      pass2_file << "nv: \n" << jmodel.nv() << std::endl;
      pass2_file << "jdata.Dinv(): \n" << jdata.Dinv() << std::endl;
      const int nv_children = data.nvSubtree[i] - jmodel.nv();
      pass2_file << "nvSubtree[i]: \n" << data.nvSubtree[i] << std::endl;
      pass2_file << "nv_children: \n" << nv_children << std::endl;

      if(nv_children > 0)
      {
        pass2_file << "J123: \n" << data.J << std::endl;
        ColsBlock J_cols = jmodel.jointCols(data.J);
        pass2_file << "J_cols123: \n" << J_cols << std::endl;
        // I couldn't find information about where SDinv is populated. 
        // Search for data.SDinv gives only two results and they only use SDinv, not populate it.
        // SDinv_cols = SDinv.middleCols(jmodel.idx_v(),jmodel.nv());
        ColsBlock SDinv_cols = jmodel.jointCols(data.SDinv);
        SDinv_cols.noalias() = J_cols * jdata.Dinv();
        pass2_file << "SDinv_cols: \n" << SDinv_cols << std::endl;
        // the rest of the Minv is filled here
        Minv.block(jmodel.idx_v(),jmodel.idx_v()+jmodel.nv(),jmodel.nv(),nv_children).noalias()
        = -SDinv_cols.transpose() * Fcrb.middleCols(jmodel.idx_v()+jmodel.nv(),nv_children);
        pass2_file << "Minv_updated_little: \n" << Minv << std::endl;

        if(parent > 0)
        {
          // leftCols is the first n columns of the matrix
          // middleCols is the q columns starting at the j-th column
          FcrbTmp.leftCols(data.nvSubtree[i]).noalias()
          = U_cols * Minv.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]);
          Fcrb.middleCols(jmodel.idx_v(),data.nvSubtree[i]) += FcrbTmp.leftCols(data.nvSubtree[i]);
        }
      }
      else
      {
        Fcrb.middleCols(jmodel.idx_v(),data.nvSubtree[i]).noalias()
        = U_cols * Minv.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]);
      }
      pass2_file << "Fcrb_after_pass2(Fcrb is the spacial forces): " << std::endl;
      PRINT_MATRIX(Fcrb, pass2_file);

      if(parent > 0)
        data.Yaba[parent] += internalVerbose::SE3actOnVerbose<Scalar>::run(data.liMi[i], Ia);
    }
  };

  /// According to Featherstone, this pass calculates the accelerations.
  /// This shouldn't be needed, but it has some manipulation of the Minv matrix.
  /// Need to print and see what happens!
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  struct ComputeMinverseForwardStep2Verbose
  : public fusion::JointUnaryVisitorBase< ComputeMinverseForwardStep2Verbose<Scalar,Options,JointCollectionTpl> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;

    typedef boost::fusion::vector<const Model &,
                                  Data &> ArgsType;

    template<typename JointModel>
    static void algo(const pinocchio::JointModelBase<JointModel> & jmodel,
                     pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      std::string folder_path = model_output_foldername + "ABA/" + std::to_string(CONTROLLER_ABA_PRINT_INDEX) + "/";
      std::filesystem::create_directories(folder_path);
      std::string pass3_file_path = folder_path + "pass3.txt";
      std::ofstream pass3_file;
      pass3_file.open(pass3_file_path, std::ios::app);
      typedef typename Model::JointIndex JointIndex;

      const JointIndex & i = jmodel.id();
      pass3_file << "Joint: " << i << std::endl;
      const JointIndex & parent = model.parents[i];
      typename Data::RowMatrixXs & Minv = data.Minv;
      typename Data::Matrix6x & FcrbTmp = data.Fcrb.back();
      pass3_file << "pass3_jmodel.nv: " << std::endl;
      pass3_file << jmodel.nv() << std::endl;
      pass3_file << "pass3_jmodel.idx_v: " << std::endl;
      pass3_file << jmodel.idx_v() << std::endl;
      pass3_file << "pass3_model.nv: " << std::endl;
      pass3_file << model.nv << std::endl;

      pass3_file << "Minv_before_pass3: " << std::endl;
      pass3_file << Minv << std::endl;
      pass3_file << "UDinv_before_pass3: " << std::endl;
      pass3_file << data.UDinv << std::endl;

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;
      ColsBlock UDinv_cols = jmodel.jointCols(data.UDinv);
      pass3_file << "UDinv_cols_pass3_1: " << std::endl;
      pass3_file << UDinv_cols << std::endl;
      forceSet::se3Action(data.oMi[i],jdata.UDinv(),UDinv_cols); // expressed in the world frame
      pass3_file << "UDinv_cols_pass3_2: " << std::endl;
      pass3_file << UDinv_cols << std::endl;
      ColsBlock J_cols = jmodel.jointCols(data.J);
      pass3_file << "J_pass3: " << std::endl;
      pass3_file << data.J << std::endl;
      pass3_file << "J_cols_pass3: " << std::endl;
      pass3_file << J_cols << std::endl;
      pass3_file << "FcrbTmp_pass3_prev: " << std::endl;
      pass3_file << FcrbTmp << std::endl;
      pass3_file << "FcrbTmp_assigned_pass3: " << std::endl;
      pass3_file << FcrbTmp.topRows(jmodel.nv()).rightCols(model.nv - jmodel.idx_v()) << std::endl;
      if(parent > 0)
      {
        FcrbTmp.topRows(jmodel.nv()).rightCols(model.nv - jmodel.idx_v()).noalias()
        = UDinv_cols.transpose() * data.Fcrb[parent].rightCols(model.nv - jmodel.idx_v());
        Minv.middleRows(jmodel.idx_v(),jmodel.nv()).rightCols(model.nv - jmodel.idx_v())
        -= FcrbTmp.topRows(jmodel.nv()).rightCols(model.nv - jmodel.idx_v());
      }
      pass3_file << "FcrbTmpUpdate: " << std::endl;
      pass3_file << UDinv_cols.transpose() * data.Fcrb[parent].rightCols(model.nv - jmodel.idx_v()) << std::endl;
      pass3_file << "FcrbTmp_pass3: " << std::endl;
      pass3_file << FcrbTmp << std::endl;
      pass3_file << "Minv_final_pass3: " << std::endl;
      PRINT_MATRIX(Minv, pass3_file);

      pass3_file << "Fcrb[i]_before_pass3: " << std::endl;
      data.Fcrb[i].rightCols(model.nv - jmodel.idx_v()).noalias() = J_cols * Minv.middleRows(jmodel.idx_v(),jmodel.nv()).rightCols(model.nv - jmodel.idx_v());
      if(parent > 0)
        data.Fcrb[i].rightCols(model.nv - jmodel.idx_v()) += data.Fcrb[parent].rightCols(model.nv - jmodel.idx_v());
      pass3_file << "Fcrb[i]_final_pass3: " << std::endl;
      PRINT_MATRIX(data.Fcrb[i], pass3_file);
    }
  };

}

namespace pinocchioVerbose
{
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline const typename pinocchio::DataTpl<Scalar,Options,JointCollectionTpl>::RowMatrixXs &
  computeMinverseVerbose(const pinocchio::ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  pinocchio::DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<ConfigVectorType> & q)
  {
    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(q.size(), model.nq, "The joint configuration vector is not of right size");
    typedef typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointIndex JointIndex;
    data.Minv.template triangularView<Eigen::Upper>().setZero();

    typedef pinocchioPass::ComputeMinverseForwardStep1Verbose<Scalar,Options,JointCollectionTpl,ConfigVectorType> Pass1;
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,q.derived()));
    }
    data.Fcrb[0].setZero();
    typedef pinocchioPass::ComputeMinverseBackwardStepVerbose<Scalar,Options,JointCollectionTpl> Pass2;
    for(JointIndex i=(JointIndex)model.njoints-1; i>0; --i)
    {
      Pass2::run(model.joints[i],data.joints[i],
                 typename Pass2::ArgsType(model,data));
    }

    //typedef pinocchioPass::ComputeMinverseForwardStep2Verbose<Scalar,Options,JointCollectionTpl> Pass3;
    //for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    //{
    //  Pass3::run(model.joints[i],data.joints[i],
    //             typename Pass3::ArgsType(model,data));
    //}

    return data.Minv;
  }

} // namespace pinocchio

/// @endcond

#endif // ifndef __pinocchio_aba_hxx__
