//
// Copyright (c) 2016-2020 CNRS, INRIA
//

#ifndef __pinocchio_plus_aba_hxx__
#define __pinocchio_plus_aba_hxx__

#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/algorithm/check.hpp"
#include "config.hpp"

typedef FixedPoint<1, 8> FixedPoint1;
typedef FixedPoint<2, 8> FixedPoint2;
typedef FixedPoint<3, 8> FixedPoint3;
typedef FixedPoint<4, 8> FixedPoint4;
typedef FixedPoint<5, 8> FixedPoint5;
typedef FixedPoint<6, 8> FixedPoint6;
typedef FixedPoint<7, 8> FixedPoint7;
typedef FixedPoint<8, 8> FixedPoint8;

#define PRINT_MATRIX(x, of) for(int i = 0; i < x.rows(); i++) { for(int j = 0; j < x.cols(); j++) { of << std::setprecision(4) << x(i, j) << " "; } of << std::endl; }

// bad, very bad :(
using namespace pinocchio;

// https://stackoverflow.com/a/28050328/13399661
template<typename T>
inline T cos_fast(T x) noexcept
{
    std::cout << "input x is " << x << std::endl;
    T tp = 1./(2.*M_PI);
    std::cout << "tp is " << tp << std::endl;
    x *= tp;
    std::cout << "x is " << x << std::endl;
    x -= T(.25) + std::floor(x + T(.25));
    std::cout << "x is " << x << std::endl;
    x *= T(16.) * (std::abs(x) - T(.5));
    std::cout << "x is " << x << std::endl;
    x += T(.225) * x * (std::abs(x) - T(1.));
    std::cout << "x is " << x << std::endl;
    return x;
}

template <typename T>
T sin_fast(T x) noexcept
{
    const T B = 4/M_PI;
    const T C = -4/(M_PI*M_PI);

    T y = B * x + C * x * std::abs(x);
    //  const float Q = 0.775;
        const float P = 0.225;

        y = P * (y * abs(y) - y) + y;   // Q * y + P * y * abs(y)
    return y;
}



namespace internalVerbose
  {
    // Usually this is a terrible approach to put pass2_file variable here
    // But I need to create it in pass2, and pass it to internalVerbose::SE3actOnVerbose::run
    // And when I tried that I got the error
    // error: call to implicitly-deleted copy constructor of 'std::ofstream' (aka 'basic_ofstream<char>')
    std::ofstream pass2_file;
    template<typename Scalar>
    struct SE3actOnVerbose
    {
      template<int Options, typename Matrix6Type>
      static typename PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix6Type)
      run(const SE3Tpl<Scalar,Options> & M,
          const Eigen::MatrixBase<Matrix6Type> & I)
      {
        typedef SE3Tpl<exp_type_act_on,Options> SE3;
        typedef typename SE3::Matrix3 Matrix3;
        typedef typename SE3::Vector3 Vector3;

        // new Matrix6Type which has exp_type_act_on as Scalar
        typedef typename Eigen::Matrix<exp_type_act_on,6,6,Options> Matrix6TypeActOn;

        typedef const Eigen::Block<Matrix6TypeActOn,3,3> constBlock3;

        typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix6TypeActOn) ReturnType;
        typedef Eigen::Block<ReturnType,3,3> Block3;

        //Matrix6Type I_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6TypeActOn,I);
        Matrix6TypeActOn I_ = I.template cast<exp_type_act_on>();
        const constBlock3 & Ai = I_.template block<3,3>(Inertia::LINEAR, Inertia::LINEAR);
        const constBlock3 & Bi = I_.template block<3,3>(Inertia::LINEAR, Inertia::ANGULAR);
        const constBlock3 & Di = I_.template block<3,3>(Inertia::ANGULAR, Inertia::ANGULAR);

        auto M_cast = M.template cast<exp_type_act_on>();
        const Matrix3 & R = M_cast.rotation();
        const Vector3 & t = M_cast.translation();

        pass2_file << "Rotation: " << std::endl << R << std::endl;
        pass2_file << "Translation: " << std::endl << t << std::endl;

        ReturnType res;
        // Rotation * Linear * Rotation^T
        Block3 Ao = res.template block<3,3>(Inertia::LINEAR, Inertia::LINEAR);
        // (Rotation * Lin_Ang * Rotation^T)^T + [translation*Linear[0], translation*Linear[1], translation*Linear[2]]
        Block3 Bo = res.template block<3,3>(Inertia::LINEAR, Inertia::ANGULAR);
        // Bo^T
        Block3 Co = res.template block<3,3>(Inertia::ANGULAR, Inertia::LINEAR);
        // Rotation * Angular * Rotation^T +
        // [translation*(Rotation * Lin_Ang * Rotation^T)[0], translation*(Rotation * Lin_Ang * Rotation^T)[1], translation*(Rotation * Lin_Ang * Rotation^T)[2] +
        // above_line^T
        Block3 Do = res.template block<3,3>(Inertia::ANGULAR, Inertia::ANGULAR);

        Do.noalias() = R*Ai; // tmp variable
        Ao.noalias() = Do*R.transpose(); // Rotation * Linear * Rotation^T
        pass2_file << "Ao_Rotation_Linear_Rotation^T: " << std::endl << Ao << std::endl;

        Do.noalias() = R*Bi; // tmp variable
        Bo.noalias() = Do*R.transpose();
        pass2_file << "Bo_Rotation_Lin_Ang_Rotation^T: " << std::endl << Bo << std::endl;

        Co.noalias() = R*Di; // tmp variable
        Do.noalias() = Co*R.transpose();
        pass2_file << "Do_Rotation_Ang_Rotation^T: " << std::endl << Do << std::endl;

        Do.row(0) += t.cross(Bo.col(0));
        Do.row(1) += t.cross(Bo.col(1));
        Do.row(2) += t.cross(Bo.col(2));
        pass2_file << "Do_Rotation_Ang_Rotation^T_after: " << std::endl << Do << std::endl;

        Co.col(0) = t.cross(Ao.col(0));
        Co.col(1) = t.cross(Ao.col(1));
        Co.col(2) = t.cross(Ao.col(2));
        Co += Bo.transpose();
        pass2_file << "Co_Rotation_Ang_Rotation^T_after: " << std::endl << Co << std::endl;

        Bo = Co.transpose();
        Do.col(0) += t.cross(Bo.col(0));
        Do.col(1) += t.cross(Bo.col(1));
        Do.col(2) += t.cross(Bo.col(2));
        pass2_file << "Do_Rotation_Ang_Rotation^T_after2: " << std::endl << Do << std::endl;

        // Do not forget to convert the result to the original type
        auto res_cast = res.template cast<Scalar>();

        return res_cast;
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
      pass1_file << "model.inertia_matrix: " << std::endl;
      pass1_file << model.inertias[i].matrix() << std::endl;

      //jmodel.calc(jdata.derived(),q.derived());
      // I decided to move the jmodel.calc function to here, for the same purposes as calc_aba
      const Scalar & qj = q[jmodel.idx_q()];

      FixedPoint<1, 8> sin, cos;
      SINCOS(qj, &sin, &cos); // For now sincos is calculated with doubles
      // Please see: Fixed-Point Trigonometric Functions on FPGAs
      
      //jdata.derived().M.setValues(sa,ca);
      // axis is always 2, may change for other robots
      //auto jdata_M = TransformRevoluteTpl<FixedPoint1, Options, 2>(sin.cast<FixedPoint1>(), cos.cast<FixedPoint1>());
      auto jdata_M = TransformRevoluteTpl<Scalar, Options, 2>(sin.cast<Scalar>(), cos.cast<Scalar>());

      // calculate the vector of relative joint placements (w.r.t. the body parent)
      const JointIndex & parent = model.parents[i];

      //data.liMi[i] = model.jointPlacements[i] * jdata_M;
      SE3Tpl<FixedPoint2, Options> model_jointPlacements_i_cast = SE3Tpl<FixedPoint2, Options>(model.jointPlacements[i].rotation().template cast<FixedPoint2>(), model.jointPlacements[i].translation().template cast<FixedPoint2>());
      auto jdata_M_cast = TransformRevoluteTpl<FixedPoint2, Options, 2>(sin.cast<FixedPoint2>(), cos.cast<FixedPoint2>());
      auto data_liMi_i_cast = model_jointPlacements_i_cast * jdata_M_cast;
      
      pass1_file << "Relative_joint_placement_rotation: " << std::endl;
      pass1_file << data_liMi_i_cast.rotation() << std::endl;
      pass1_file << "Relative_joint_placement_translation: " << std::endl;
      pass1_file << data_liMi_i_cast.translation() << std::endl;

      data.liMi[i] = data_liMi_i_cast.template cast<Scalar>();

      // make data.oMi[i] FixedPoint2
      auto data_oMi_i_cast = data_liMi_i_cast.template cast<FixedPoint2>();
      auto data_oMi_parent_cast = data.oMi[parent].template cast<FixedPoint2>();

      // calculate the absolute joint placements
      if (parent>0)
        data_oMi_i_cast = data_oMi_parent_cast * data_liMi_i_cast;
      else
        data_oMi_i_cast = data_liMi_i_cast;
      pass1_file << "Absolute_joint_placement_rotation: " << std::endl;
      pass1_file << data_oMi_i_cast.rotation() << std::endl;
      pass1_file << "Absolute_joint_placement_translation: " << std::endl;
      pass1_file << data_oMi_i_cast.translation() << std::endl;

      data.oMi[i] = data_oMi_i_cast.template cast<Scalar>();

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
      using namespace internalVerbose;
      std::string folder_path = model_output_foldername + "ABA/" + std::to_string(CONTROLLER_ABA_PRINT_INDEX) + "/";
      std::filesystem::create_directories(folder_path);
      std::string pass2_file_path = folder_path + "pass2.txt";
      //std::ofstream pass2_file;
      pass2_file.open(pass2_file_path, std::ios::app);

      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Inertia Inertia;

      const JointIndex & i = jmodel.id();
      pass2_file << "Joint: " << i << std::endl;
      const JointIndex & parent  = model.parents[i];

      // I guess 4 bits are enough for Ia
      typename Inertia::Matrix6 & Ia = data.Yaba[i];
      typename Data::RowMatrixXs & Minv = data.Minv;

      // jmodel.calc_aba(jdata.derived(), Ia, parent > 0);
      // I couldn't use the joint data variables directly, as their types are decided already
      // I'll create new types for data.U, data.Dinv, data.UDinv
      // Since I currently have no other algorithm using these, it should effect nothing.
      // When I add pass3 for other robots, it will be a problem as pass3 uses these variables
      // axis is always 2 for Panda arm, may change for other robots
      int joint_axis = 2;
      // FixedPoint2 is enough for data_U
      auto data_U = Ia.col(Inertia::ANGULAR + joint_axis);
      pass2_file << "data.U = I.col(Inertia::ANGULAR + joint_axis): " << std::endl << data_U << std::endl;
      auto data_Dinv = jdata.Dinv(); // jdata.Dinv() is always full of zeros for Panda arm
      // FixedPoint7 is needed for this element
      data_Dinv(0) = Scalar(1) / Ia(Inertia::ANGULAR + joint_axis, Inertia::ANGULAR + joint_axis);
      pass2_file << "data.Dinv[0] = Scalar(1) / Ia(Inertia::ANGULAR + joint_axis, Inertia::ANGULAR + joint_axis): " << std::endl << data_Dinv(0) << std::endl;
      // FixedPoint4 is needed for UDinv
      auto data_UDinv = data_U * data_Dinv(0);
      pass2_file << "data.UDinv = data.U * data.Dinv[0]: " << std::endl << data_UDinv << std::endl;
      if(parent > 0){
        Ia -= data_UDinv * data_U.transpose();
        pass2_file << "I: " << std::endl << Ia << std::endl;
      }

      // Block of size (p,q), starting at (i,j) matrix.block(i,j,p,q);
      Minv.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),jmodel.nv()) = data_Dinv;
      // idx_v is jmodel.id - 1
      // nv is always 1, don'w know what it represents
      pass2_file << "jdata.Dinv()_Minv_element: \n" << data_Dinv << std::endl;

      if(parent > 0){
        data.Yaba[parent] += internalVerbose::SE3actOnVerbose<Scalar>::run(data.liMi[i], Ia);
        pass2_file << "Yaba[parent]: " << std::endl;
        pass2_file << data.Yaba[parent] << std::endl;
      }
      pass2_file.close();
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
