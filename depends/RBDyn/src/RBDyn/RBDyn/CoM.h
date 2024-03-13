/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

// std
#include <vector>

// Eigen
#include <Eigen/Core>

// RBDyn
#include <rbdyn/config.hh>

#include "Jacobian.h"

namespace rbd
{
class MultiBody;
struct MultiBodyConfig;

/**
 * Compute the Center of Mass (CoM) position of a multibody.
 * @param mb MultiBody used has model.
 * @param mbc Use bodyPosW.
 * @return CoM position in world frame.
 */
RBDYN_DLLAPI Eigen::Vector3d computeCoM(const MultiBody & mb, const MultiBodyConfig & mbc);

/**
 * Compute the Center of Mass (CoM) velocity of a multibody.
 * @param mb MultiBody used has model.
 * @param mbc Use bodyPosW and bodyVelB.
 * @return CoM velocity in world frame.
 */
RBDYN_DLLAPI Eigen::Vector3d computeCoMVelocity(const MultiBody & mb, const MultiBodyConfig & mbc);

/**
 * Compute the Center of Mass (CoM) acceleration of a multibody.
 * @param mb MultiBody used has model.
 * @param mbc Use bodyPosW, bodyVelW, bodyVelB, and bodyAccB.
 * @return CoM velocity in world frame.
 */
RBDYN_DLLAPI Eigen::Vector3d computeCoMAcceleration(const MultiBody & mb, const MultiBodyConfig & mbc);

/**
 * Compute the CoM jacobian with a simple but slow algorithm.
 */
class RBDYN_DLLAPI CoMJacobianDummy
{
public:
  CoMJacobianDummy();

  /// @param mb MultiBody used has model.
  CoMJacobianDummy(const MultiBody & mb);
  /**
   * @param mb MultiBody used has model.
   * @param weight Per body weight.
   */
  CoMJacobianDummy(const MultiBody & mb, std::vector<double> weight);

  ~CoMJacobianDummy();

  /**
   * Compute the CoM jacobian.
   * @param mb MultiBody used has model.
   * @param mbc Use bodyPosW and motionSubspace.
   * @return CoM Jacobian of mb with mbc configuration.
   */
  const Eigen::MatrixXd & jacobian(const MultiBody & mb, const MultiBodyConfig & mbc);

  /**
   * Access the last computed CoM jacobian
   * @return Latest CoM jacobian that was computed by this object
   */
  inline const Eigen::MatrixXd & jacobian() const noexcept
  {
    return jac_;
  }

  /**
   * Compute the time derivative of the CoM jacobian.
   * @param mb MultiBody used has model.
   * @param mbc Use bodyPosW, bodyVelB, bodyVelW, and motionSubspace.
   * @return Time derivativo of the jacobian of mb with mbc configuration.
   */
  const Eigen::MatrixXd & jacobianDot(const MultiBody & mb, const MultiBodyConfig & mbc);

  /**
   * Access the last computed derivative of the CoM jacobian
   * @return Latest derivative of the CoM jacobian that was computed by this object
   */
  inline const Eigen::MatrixXd & jacobianDot() const noexcept
  {
    return jacDot_;
  }

  // safe version for python binding

  /** safe version of @see jacobian.
   * @throw std::domain_error If mb don't match mbc.
   */
  const Eigen::MatrixXd & sJacobian(const MultiBody & mb, const MultiBodyConfig & mbc);

  /** safe version of @see jacobianDot.
   * @throw std::domain_error If mb don't match mbc.
   */
  const Eigen::MatrixXd & sJacobianDot(const MultiBody & mb, const MultiBodyConfig & mbc);

private:
  void init(const rbd::MultiBody & mb);

private:
  Eigen::MatrixXd jac_;
  Eigen::MatrixXd jacDot_;
  Eigen::MatrixXd jacFull_;

  std::vector<Jacobian> jacVec_;
  double totalMass_;
  std::vector<double> bodiesWeight_;
};

/**
 * Compute the CoM jacobian
 */
class RBDYN_DLLAPI CoMJacobian
{
public:
  CoMJacobian();

  /// @param mb MultiBody used as model.
  CoMJacobian(const MultiBody & mb);
  /**
   * @param mb MultiBody used has model.
   * @param weight Per body weight.
   */
  CoMJacobian(const MultiBody & mb, std::vector<double> weight);

  /**
   * Compute bodies CoM position and mass based on a MultiBody.
   * This method allow to update some pre-computed parameters
   * when MultiBody inertia has changed.
   * @param mb MultiBody used as model.
   */
  void updateInertialParameters(const MultiBody & mb);

  /// @return Per bodies weight.
  const std::vector<double> & weight() const;

  /// Per bodies weight setter.
  void weight(const MultiBody & mb, std::vector<double> w);

  /**
   * Compute the CoM jacobian.
   * @param mb MultiBody used as model.
   * @param mbc Use bodyPosW and motionSubspace.
   * @return CoM Jacobian of mb with mbc configuration.
   */
  const Eigen::MatrixXd & jacobian(const MultiBody & mb, const MultiBodyConfig & mbc);

  /**
   * Access the last computed CoM jacobian
   * @return Latest CoM jacobian that was computed by this object
   */
  inline const Eigen::MatrixXd & jacobian() const noexcept
  {
    return jac_;
  }

  /**
   * Compute the time derivative of the CoM jacobian.
   * @param mb MultiBody used as model.
   * @param mbc Use bodyPosW, bodyVelB, bodyVelW, and motionSubspace.
   * @return Time derivativo of the jacobian of mb with mbc configuration.
   */
  const Eigen::MatrixXd & jacobianDot(const MultiBody & mb, const MultiBodyConfig & mbc);

  /**
   * Access the last computed derivative of the CoM jacobian
   * @return Latest derivative of the CoM jacobian that was computed by this object
   */
  inline const Eigen::MatrixXd & jacobianDot() const noexcept
  {
    return jacDot_;
  }

  /**
   * Compute the com velocity (with weight) (J·alpha).
   * @param mb MultiBody used as model.
   * @param mbc Use bodyPosW, bodyVelB.
   * @return CoM velocity (with weight).
   */
  Eigen::Vector3d velocity(const MultiBody & mb, const MultiBodyConfig & mbc) const;

  /**
   * Compute the com normal acceleration (with weight) (JDot·alpha).
   * @param mb MultiBody used as model.
   * @param mbc Use bodyPosW, bodyVelW, bodyVelB, jointVelocity, parentToSon.
   * @return CoM normal acceleration (with weight).
   */
  Eigen::Vector3d normalAcceleration(const MultiBody & mb, const MultiBodyConfig & mbc);

  /**
   * Compute the com normal acceleration (with weight) (JDot·alpha).
   * @param mb MultiBody used as model.
   * @param mbc Use bodyPosW, bodyVelW, bodyVelB.
   * @param normalAccB Normal bodies acceleration in body frame.
   * @return CoM normal acceleration (with weight).
   */
  Eigen::Vector3d normalAcceleration(const MultiBody & mb,
                                     const MultiBodyConfig & mbc,
                                     const std::vector<sva::MotionVecd> & normalAccB) const;

  // safe version for python binding

  /** safe version of @see updateInertialParameters.
   * @throw std::domain_error If mb don't match the mb used in constructor.
   */
  void sUpdateInertialParameters(const MultiBody & mb);

  /** safe version of @see weight.
   * @throw std::domain_error If mb don't match the mb used in constructor
   * or w missmatch mb.
   */
  void sWeight(const MultiBody & mb, std::vector<double> w);

  /** safe version of @see jacobian.
   * @throw std::domain_error If mb don't match mbc.
   */
  const Eigen::MatrixXd & sJacobian(const MultiBody & mb, const MultiBodyConfig & mbc);

  /** safe version of @see jacobianDot.
   * @throw std::domain_error If mb don't match mbc.
   */
  const Eigen::MatrixXd & sJacobianDot(const MultiBody & mb, const MultiBodyConfig & mbc);

  /** safe version of @see velocity.
   * @throw std::domain_error If mb don't match mbc.
   */
  Eigen::Vector3d sVelocity(const MultiBody & mb, const MultiBodyConfig & mbc) const;

  /** safe version of @see normalAcceleration.
   * @throw std::domain_error If mb don't match mbc.
   */
  Eigen::Vector3d sNormalAcceleration(const MultiBody & mb, const MultiBodyConfig & mbc);

  /** safe version of @see normalAcceleration.
   * @throw std::domain_error If mb don't match mbc or normalAccB.
   */
  Eigen::Vector3d sNormalAcceleration(const MultiBody & mb,
                                      const MultiBodyConfig & mbc,
                                      const std::vector<sva::MotionVecd> & normalAccB) const;

private:
  void init(const rbd::MultiBody & mb);

private:
  Eigen::MatrixXd jac_;
  Eigen::MatrixXd jacDot_;

  std::vector<double> bodiesCoeff_;

  /// @brief list of CoM of the bodies. Bodies with null mass have a (0,0,0) CoM
  std::vector<sva::PTransformd> bodiesCoM_;
  std::vector<std::vector<int>> jointsSubBodies_;

  // jacobian, jacobianDot computation buffer
  std::vector<sva::PTransformd> bodiesCoMWorld_;
  std::vector<sva::MotionVecd> bodiesCoMVelB_;
  // store normal acceleration of each bodies when calling normal acceleration
  std::vector<sva::MotionVecd> normalAcc_;

  std::vector<double> weight_;
};

// safe version for python binding

/**
 * Safe version.
 * @see computeCoM.
 * @throw std::domain_error If there is a mismatch between mb and mbc.
 */
RBDYN_DLLAPI Eigen::Vector3d sComputeCoM(const MultiBody & mb, const MultiBodyConfig & mbc);

/**
 * Safe version.
 * @see computeCoMVelocity.
 * @throw std::domain_error If there is a mismatch between mb and mbc.
 */
RBDYN_DLLAPI Eigen::Vector3d sComputeCoMVelocity(const MultiBody & mb, const MultiBodyConfig & mbc);

/**
 * Safe version.
 * @see computeCoMAcceleration.
 * @throw std::domain_error If there is a mismatch between mb and mbc.
 */
RBDYN_DLLAPI Eigen::Vector3d sComputeCoMAcceleration(const MultiBody & mb, const MultiBodyConfig & mbc);

} // namespace rbd
