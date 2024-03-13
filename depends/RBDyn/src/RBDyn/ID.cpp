/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// associated header
#include "RBDyn/ID.h"

// includes
// RBDyn
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"

namespace rbd
{

InverseDynamics::InverseDynamics(const MultiBody & mb) : f_(static_cast<size_t>(mb.nrBodies())) {}

void InverseDynamics::inverseDynamics(const MultiBody & mb, MultiBodyConfig & mbc)
{
  const std::vector<Body> & bodies = mb.bodies();
  const std::vector<Joint> & joints = mb.joints();
  const std::vector<int> & pred = mb.predecessors();

  sva::MotionVecd a_0(Eigen::Vector3d::Zero(), mbc.gravity);

  for(std::size_t i = 0; i < bodies.size(); ++i)
  {
    const sva::PTransformd & X_p_i = mbc.parentToSon[i];

    const sva::MotionVecd & vj_i = mbc.jointVelocity[i];
    sva::MotionVecd ai_tan = joints[i].tanAccel(mbc.alphaD[i]);

    const sva::MotionVecd & vb_i = mbc.bodyVelB[i];

    if(pred[i] != -1)
      mbc.bodyAccB[i] = X_p_i * mbc.bodyAccB[static_cast<size_t>(pred[i])] + ai_tan + vb_i.cross(vj_i);
    else
      mbc.bodyAccB[i] = X_p_i * a_0 + ai_tan + vb_i.cross(vj_i);

    f_[i] = bodies[i].inertia() * mbc.bodyAccB[i] + vb_i.crossDual(bodies[i].inertia() * vb_i)
            - mbc.bodyPosW[i].dualMul(mbc.force[i]);
  }

  computeJointTorques(mb, mbc);
}

void InverseDynamics::inverseDynamicsNoInertia(const MultiBody & mb, MultiBodyConfig & mbc)
{
  for(size_t i = 0; i < static_cast<size_t>(mb.nrBodies()); ++i)
  {
    f_[i] = mbc.bodyPosW[i].dualMul(mbc.force[i]);
  }

  computeJointTorques(mb, mbc);
}

void InverseDynamics::sInverseDynamics(const MultiBody & mb, MultiBodyConfig & mbc)
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

  inverseDynamics(mb, mbc);
}

void InverseDynamics::sInverseDynamicsNoInertia(const MultiBody & mb, MultiBodyConfig & mbc)
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

  inverseDynamicsNoInertia(mb, mbc);
}

const std::vector<sva::ForceVecd> & InverseDynamics::f() const
{
  return f_;
}

/*
 * Private functions
 */

void InverseDynamics::computeJointTorques(const MultiBody & mb, MultiBodyConfig & mbc)
{
  const std::vector<Body> & bodies = mb.bodies();
  const std::vector<Joint> & joints = mb.joints();
  const std::vector<int> & pred = mb.predecessors();

  for(int i = static_cast<int>(bodies.size()) - 1; i >= 0; --i)
  {
    const auto ui = static_cast<size_t>(i);
    for(int j = 0; j < joints[ui].dof(); ++j)
    {
      const auto uj = static_cast<size_t>(j);
      mbc.jointTorque[ui][uj] = mbc.motionSubspace[ui].col(j).transpose() * f_[ui].vector();
    }

    if(pred[ui] != -1)
    {
      const sva::PTransformd & X_p_i = mbc.parentToSon[ui];
      f_[static_cast<size_t>(pred[ui])] = f_[static_cast<size_t>(pred[ui])] + X_p_i.transMul(f_[ui]);
    }
  }
}

} // namespace rbd
