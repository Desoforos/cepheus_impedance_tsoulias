/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <iostream>

// associated header
#include "RBDyn/IS.h"

// includes
// sva
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
#include "RBDyn/FK.h"
#include "RBDyn/FV.h"
#include "RBDyn/Jacobian.h"
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"
#include "RBDyn/util.hh"

namespace rbd
{
using namespace Eigen;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 6, 1> Vector6d;

InverseStatics::InverseStatics(const MultiBody & mb)
: f_(static_cast<size_t>(mb.nrBodies())), df_(static_cast<size_t>(mb.nrBodies())),
  jointTorqueDiff_(static_cast<size_t>(mb.nrJoints())), jacW_(static_cast<size_t>(mb.nrBodies())),
  fullJac_(6, mb.nrDof()), jacobianSizeHasBeenSet_(false)
{
  fullJac_.setZero();
  for(size_t i = 0; i < static_cast<size_t>(mb.nrBodies()); ++i)
  {
    jacW_[i] = Jacobian(mb, mb.body(static_cast<int>(i)).name());
    df_[i] = MatrixXd::Zero(6, mb.nrDof());
  }

  for(size_t i = 0; i < static_cast<size_t>(mb.nrJoints()); ++i)
  {
    jointTorqueDiff_[i].resize(mb.joint(static_cast<int>(i)).dof(), mb.nrDof());
    jointTorqueDiff_[i].setZero();
  }
}

void InverseStatics::setJacobianSize(const MultiBody & mb,
                                     const MultiBodyConfig & mbc,
                                     const std::vector<Eigen::MatrixXd> & jacMomentsAndForces)
{
  const std::vector<Body> & bodies = mb.bodies();

  long nColsWanted = 0;
  for(std::size_t i = 0; i < bodies.size(); ++i)
  {
    if(jacMomentsAndForces[i].cols() > nColsWanted)
    {
      nColsWanted = static_cast<long>(jacMomentsAndForces[i].cols());
    }
  }
  for(std::size_t i = 0; i < bodies.size(); ++i)
  {
    if(nColsWanted > df_[i].cols())
    {
      df_[i].resize(6, nColsWanted);
    }
    df_[i].setZero();
    jointTorqueDiff_[i].resize(mbc.motionSubspace[i].cols(), df_[i].cols());
    jointTorqueDiff_[i].setZero();
  }
  jacobianSizeHasBeenSet_ = true;
}

void InverseStatics::inverseStatics(const MultiBody & mb, MultiBodyConfig & mbc)
{
  const std::vector<Body> & bodies = mb.bodies();
  const std::vector<Joint> & joints = mb.joints();
  const std::vector<int> & pred = mb.predecessors();

  sva::MotionVecd a_0(Vector3d::Zero(), mbc.gravity);

  for(std::size_t i = 0; i < bodies.size(); ++i)
  {
    mbc.bodyAccB[i] = mbc.bodyPosW[i] * a_0;
    f_[i] = bodies[i].inertia() * mbc.bodyAccB[i] - mbc.bodyPosW[i].dualMul(mbc.force[i]);
  }

  for(int i = static_cast<int>(joints.size()) - 1; i >= 0; --i)
  {
    // jointTorque is a vector<vector<double>> thus it is necessary to use
    // Eigen::Map to set a vector of elements at once
    // This is identical to do that:
    //
    //    $for (int j = 0; j < joints[i].dof(); ++j)
    //    $  mbc.jointTorque[i][j] = mbc.motionSubspace[i].col(j).transpose() *
    //    f_[i].vector();
    //
    const auto ui = static_cast<size_t>(i);
    VectorXd::Map(mbc.jointTorque[ui].data(), joints[ui].dof()) = f_[ui].vector().transpose() * mbc.motionSubspace[ui];

    if(pred[ui] != -1) f_[static_cast<size_t>(pred[ui])] += mbc.parentToSon[ui].transMul(f_[ui]);
  }
}

void InverseStatics::computeTorqueJacobianJoint(const MultiBody & mb,
                                                MultiBodyConfig & mbc,
                                                const std::vector<MatrixXd> & jacMomentsAndForces)
{
  assert(jacMomentsAndForces.size() == static_cast<size_t>(mb.nrBodies()));

  auto transMat = [](const sva::PTransformd & T)
  {
    Eigen::Matrix6d res;
    Eigen::Matrix3d Rt = T.rotation().transpose();
    res.block(3, 0, 3, 3).setZero();
    res.block(0, 0, 3, 3) = Rt;
    res.block(0, 3, 3, 3) = vector3ToCrossMatrix(T.translation()) * Rt;
    res.block(3, 3, 3, 3) = Rt;
    return res;
  };

  if(!jacobianSizeHasBeenSet_) setJacobianSize(mb, mbc, jacMomentsAndForces);

  const std::vector<Body> & bodies = mb.bodies();
  const std::vector<int> & pred = mb.predecessors();
  const std::vector<Joint> & joints = mb.joints();

  sva::MotionVecd a_0(Vector3d::Zero(), mbc.gravity);

  Matrix6d M;
  Matrix6d N;

  Vector3d aC = a_0.angular();
  Vector3d aF = a_0.linear();
  Matrix3d hatAF, hatAC;
  hatAF = vector3ToCrossMatrix(aF);
  hatAC = vector3ToCrossMatrix(aC);

  for(std::size_t i = 0; i < bodies.size(); ++i)
  {
    df_[i].setZero();
    M.setZero();
    N.setZero();
    // Complete the previously computed jacobian to a full jacobian
    jacW_[i].fullJacobian(mb, jacW_[i].jacobian(mb, mbc), fullJac_);

    mbc.bodyAccB[i] = mbc.bodyPosW[i] * a_0;

    Matrix3d & RW = mbc.bodyPosW[i].rotation();
    Vector3d & tW = mbc.bodyPosW[i].translation();
    Vector3d & fC = mbc.force[i].couple();
    Vector3d & fF = mbc.force[i].force();
    Matrix3d hatFC, hatFF, hathattaC, hathattfF, hattW;
    hatFF = vector3ToCrossMatrix(fF);
    hatFC = vector3ToCrossMatrix(fC);
    hattW = vector3ToCrossMatrix(tW);
    Vector3d hattWaC = hattW * aC;
    Vector3d hattWfF = hattW * fF;
    hathattaC = vector3ToCrossMatrix(hattWaC);
    hathattfF = vector3ToCrossMatrix(hattWfF);

    f_[i] = bodies[i].inertia() * mbc.bodyAccB[i] - mbc.bodyPosW[i].dualMul(mbc.force[i]);

    M.block(0, 0, 3, 3) = RW * hatAC;
    M.block(3, 0, 3, 3) = RW * (-hathattaC + hatAF);
    M.block(3, 3, 3, 3) = RW * hatAC;

    N.block(0, 0, 3, 3) = RW * (hatFC - hathattfF);
    N.block(0, 3, 3, 3) = RW * hatFF;
    N.block(3, 0, 3, 3) = RW * hatFF;

    df_[i].block(0, 0, fullJac_.rows(), fullJac_.cols()) = (bodies[i].inertia().matrix() * M - N) * fullJac_;

    if(jacMomentsAndForces[i].cols() > 0)
    {
      df_[i] += mbc.bodyPosW[i].dualMatrix() * jacMomentsAndForces[i];
    }
  }

  for(int i = static_cast<int>(joints.size()) - 1; i >= 0; --i)
  {
    const auto ui = static_cast<size_t>(i);
    jointTorqueDiff_[ui] = mbc.motionSubspace[ui].transpose() * df_[ui];

    if(pred[ui] != -1)
    {
      const auto pred_index = static_cast<size_t>(pred[ui]);
      Matrix6d transPtS = transMat(mbc.parentToSon[ui]);

      f_[pred_index] += mbc.parentToSon[ui].transMul(f_[ui]);
      df_[pred_index] += transPtS * df_[ui];

      Matrix3d & R = mbc.jointConfig[ui].rotation();
      Vector3d & t = mbc.jointConfig[ui].translation();
      Vector3d RfC = R.transpose() * f_[ui].couple();
      Vector3d RfF = R.transpose() * f_[ui].force();
      Matrix3d hatRfC = vector3ToCrossMatrix(RfC);
      Matrix3d hatRfF = vector3ToCrossMatrix(RfF);
      Matrix6d MJ;

      MJ.block(0, 0, 3, 3) = hatRfC + RfF * t.transpose() - RfF.dot(t) * Matrix3d::Identity();
      MJ.block(0, 3, 3, 3) = -hatRfF;
      MJ.block(3, 0, 3, 3) = hatRfF;
      MJ.block(3, 3, 3, 3).setZero();

      df_[pred_index].block(0, mb.jointPosInDof(i), 6, joints[ui].dof()) -=
          transMat(mb.transforms()[ui]) * MJ * mbc.motionSubspace[ui];
    }
  }
}

void InverseStatics::computeTorqueJacobianJoint(const MultiBody & mb, MultiBodyConfig & mbc)
{
  std::vector<MatrixXd> jacMandF;
  for(int i = 0; i < mb.nrJoints(); ++i) jacMandF.push_back(MatrixXd(0, 0));

  computeTorqueJacobianJoint(mb, mbc, jacMandF);
}

void printMBC(const MultiBody & mb, const MultiBodyConfig & mbc)
{
  std::cout << "mb.bodies() = " << mb.bodies() << std::endl;
  std::cout << "mb.joints() = " << mb.joints() << std::endl;
  std::cout << "mb.predecessors() = " << mb.predecessors() << std::endl;
  std::cout << "mbc.gravity = " << mbc.gravity << std::endl;
  std::cout << "mbc.parentToSon = \n" << mbc.parentToSon << std::endl;
  std::cout << "mbc.bodyPosW = \n" << mbc.bodyPosW << std::endl;
  std::cout << "mbc.force = \n" << mbc.force << std::endl;
  std::cout << "mbc.motionSubspace = \n" << mbc.motionSubspace << std::endl;
}

void InverseStatics::sInverseStatics(const MultiBody & mb, MultiBodyConfig & mbc)
{
  checkMatchAlphaD(mb, mbc);
  checkMatchForce(mb, mbc);
  checkMatchJointConf(mb, mbc);
  checkMatchJointVelocity(mb, mbc);
  checkMatchBodyPos(mb, mbc);
  checkMatchParentToSon(mb, mbc);
  checkMatchBodyVel(mb, mbc);
  checkMatchMotionSubspace(mb, mbc);

  checkMatchBodyAcc(mb, mbc);
  checkMatchJointTorque(mb, mbc);

  inverseStatics(mb, mbc);
}

} // namespace rbd
