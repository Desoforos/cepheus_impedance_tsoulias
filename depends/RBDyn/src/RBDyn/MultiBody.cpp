/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// associated header
#include "RBDyn/MultiBody.h"

// includes
// RBDyn
#include "RBDyn/Body.h"
#include "RBDyn/Joint.h"

namespace rbd
{

MultiBody::MultiBody() : nrParams_(0), nrDof_(0) {}

MultiBody::MultiBody(std::vector<Body> bodies,
                     std::vector<Joint> joints,
                     std::vector<int> pred,
                     std::vector<int> succ,
                     std::vector<int> parent,
                     std::vector<sva::PTransformd> Xto)
: bodies_(std::move(bodies)), joints_(std::move(joints)), pred_(std::move(pred)), succ_(std::move(succ)),
  parent_(std::move(parent)), Xt_(std::move(Xto)), jointPosInParam_(joints_.size()), jointPosInDof_(joints_.size()),
  nrParams_(0), nrDof_(0)
{
  for(int i = 0; i < static_cast<int>(bodies_.size()); ++i)
  {
    const auto ui = static_cast<size_t>(i);

    bodyNameToInd_[bodies_[ui].name()] = i;
    jointNameToInd_[joints_[ui].name()] = i;

    jointPosInParam_[ui] = nrParams_;
    jointPosInDof_[ui] = nrDof_;

    nrParams_ += joints_[ui].params();
    nrDof_ += joints_[ui].dof();
  }
}

} // namespace rbd
