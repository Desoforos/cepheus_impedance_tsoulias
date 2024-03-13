/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "RBDyn/Coriolis.h"

namespace rbd
{

Coriolis::Coriolis(const rbd::MultiBody & mb) : coriolis_(mb.nrDof(), mb.nrDof()), res_(0, 0)
{
  Eigen::Vector3d com;
  double mass;
  for(int i = 0; i < mb.nrBodies(); ++i)
  {
    mass = mb.body(i).inertia().mass();
    if(mass > 0)
    {
      com = mb.body(i).inertia().momentum() / mass;
    }
    else
    {
      com.setZero();
    }
    jacs_.push_back(rbd::Jacobian(mb, mb.body(i).name(), com));
    compactPaths_.push_back(jacs_.back().compactPath(mb));
    if(jacs_.back().dof() > res_.rows())
    {
      res_.resize(jacs_.back().dof(), jacs_.back().dof());
    }
  }
}

const Eigen::MatrixXd & Coriolis::coriolis(const rbd::MultiBody & mb, const rbd::MultiBodyConfig & mbc)
{
  Eigen::Matrix3d rot;
  Eigen::Matrix3d rDot;

  Eigen::Matrix3d inertia;

  coriolis_.setZero();

  for(int i = 0; i < mb.nrBodies(); ++i)
  {
    const auto body_sindex = static_cast<int>(i);
    const auto body_uindex = static_cast<size_t>(i);

    const auto & jac = jacs_[body_uindex].jacobian(mb, mbc);
    const auto & jacDot = jacs_[body_uindex].jacobianDot(mb, mbc);

    rot = mbc.bodyPosW[body_uindex].rotation().transpose();
    rDot.noalias() = sva::vector3ToCrossMatrix(mbc.bodyVelW[body_uindex].angular()) * rot;

    auto jvi = jac.bottomRows<3>();
    auto jDvi = jacDot.bottomRows<3>();

    auto jwi = jac.topRows<3>();
    auto jDwi = jacDot.topRows<3>();

    double mass = mb.body(body_sindex).inertia().mass();
    inertia = mb.body(body_sindex).inertia().inertia()
              - sva::vector3ToCrossMatrix<double>(mass * jacs_[body_uindex].point())
                    * sva::vector3ToCrossMatrix(jacs_[body_uindex].point()).transpose();

    Eigen::Matrix3d ir = inertia * rot.transpose();

    /* C = \sum m_i J_{v_i}^T \dot{J}_{v_i}
     *        + J_{w_i}^T R_i I_i R_i^T \dot{J}_{w_i}
     *        + J_{w_i}^T \dot{R}_i I_i R_i^T J_{w_i} */

    res_.topLeftCorner(jacs_[body_uindex].dof(), jacs_[body_uindex].dof()).noalias() =
        mass * jvi.transpose() * jDvi + jwi.transpose() * (rot * ir * jDwi + rDot * ir * jwi);

    jacs_[body_uindex].expandAdd(compactPaths_[body_uindex],
                                 res_.topLeftCorner(jacs_[body_uindex].dof(), jacs_[body_uindex].dof()), coriolis_);
  }

  return coriolis_;
}

} // namespace rbd
