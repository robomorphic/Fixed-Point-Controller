//
// Copyright (c) 2016-2018 CNRS
//

#ifndef __pinocchio_plus_aba_hpp__
#define __pinocchio_plus_aba_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace pinocchioVerbose
{
  ///
  /// \brief Computes the inverse of the joint space inertia matrix using Articulated Body formulation.
  /// \remarks Only the upper triangular part of the matrix is filled.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  ///
  /// \return The inverse of the joint space inertia matrix stored in data.Minv.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline const typename pinocchio::DataTpl<Scalar,Options,JointCollectionTpl>::RowMatrixXs &
  computeMinverseVerbose(const pinocchio::ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  pinocchio::DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<ConfigVectorType> & q);
} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio_plus/aba.hxx"

#endif // ifndef __pinocchio_aba_hpp__
