/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// includes
// std
#include <iostream>

// boost
#define BOOST_TEST_MODULE Algo
#include <boost/test/unit_test.hpp>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
#include "RBDyn/Body.h"
#include "RBDyn/FA.h"
#include "RBDyn/FK.h"
#include "RBDyn/FV.h"
#include "RBDyn/ID.h"
#include "RBDyn/IK.h"
#include "RBDyn/Jacobian.h"
#include "RBDyn/Joint.h"
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"
#include "RBDyn/MultiBodyGraph.h"
#include "RBDyn/NumericalIntegration.h"

// arm
#include "XYZSarm.h"
#include "XYZarm.h"

namespace rbd
{
static constexpr double PI = 3.141592653589793238462643383279502884e+00;
}

const double TOL = 0.0000001;

BOOST_AUTO_TEST_CASE(FKTest)
{
  using namespace Eigen;
  using namespace sva;
  using namespace rbd;

  rbd::MultiBody mb, mb2;
  rbd::MultiBodyConfig mbc, mbc2;
  rbd::MultiBodyGraph mbg, mbg2;
  std::tie(mb, mbc, mbg) = makeXYZarm();
  std::tie(mb2, mbc2, mbg2) = makeXYZSarm();

  // check identity
  mbc.q = {{}, {0.}, {0.}, {0.}};

  forwardKinematics(mb, mbc);

  std::vector<PTransformd> res = {PTransformd(Vector3d(0., 0., 0.)), PTransformd(Vector3d(0., 0.5, 0)),
                                  PTransformd(Vector3d(0., 1.5, 0.)), PTransformd(Vector3d(0., 2.5, 0))};

  BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(), mbc.bodyPosW.begin(), mbc.bodyPosW.end());

  // check rotX
  mbc.q = {{}, {rbd::PI / 2.}, {0.}, {0.}};

  forwardKinematics(mb, mbc);

  res = {PTransformd(Vector3d(0., 0., 0.)), PTransformd(RotX(rbd::PI / 2.), Vector3d(0., 0.5, 0.)),
         PTransformd(RotX(rbd::PI / 2.), Vector3d(0., 0.5, 1.)),
         PTransformd(RotX(rbd::PI / 2.), Vector3d(0., 0.5, 2.))};

  for(size_t i = 0; i < res.size(); ++i)
  {
    BOOST_CHECK_SMALL((res[i].translation() - mbc.bodyPosW[i].translation()).norm(), TOL);
    BOOST_CHECK_SMALL((res[i].rotation() - mbc.bodyPosW[i].rotation()).norm(), TOL);
  }

  // check rotY
  mbc.q = {{}, {0.}, {rbd::PI / 2.}, {0.}};

  forwardKinematics(mb, mbc);

  res = {PTransformd(Vector3d(0., 0., 0.)), PTransformd(Vector3d(0., .5, 0.)),
         PTransformd(RotY(rbd::PI / 2.), Vector3d(0., 1.5, 0.)),
         PTransformd(RotY(rbd::PI / 2.), Vector3d(0., 2.5, 0.))};

  BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(), mbc.bodyPosW.begin(), mbc.bodyPosW.end());

  // check rotZ
  mbc.q = {{}, {0.}, {0.}, {rbd::PI / 2.}};

  forwardKinematics(mb, mbc);

  res = {PTransformd(Vector3d(0., 0., 0.)), PTransformd(Vector3d(0., 0.5, 0.)), PTransformd(Vector3d(0., 1.5, 0.)),
         PTransformd(RotZ(rbd::PI / 2.), Vector3d(0., 2.5, 0.))};

  BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(), mbc.bodyPosW.begin(), mbc.bodyPosW.end());

  // check identity
  mbc2.q = {{}, {0.}, {0.}, {0.}, {1., 0., 0., 0.}};

  forwardKinematics(mb2, mbc2);

  res = {PTransformd(Vector3d(0., 0., 0.)), PTransformd(Vector3d(0., 0.5, 0)), PTransformd(Vector3d(0., 1.5, 0.)),
         PTransformd(Vector3d(0., 2.5, 0)), PTransformd(Vector3d(0.5, 1., 0.))};

  BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(), mbc2.bodyPosW.begin(), mbc2.bodyPosW.end());
  // check sphere rot Y
  Quaterniond q(AngleAxisd(rbd::PI / 2., Vector3d::UnitY()));
  mbc2.q = {{}, {0.}, {0.}, {0.}, {q.w(), q.x(), q.y(), q.z()}};

  forwardKinematics(mb2, mbc2);

  res = {PTransformd(Vector3d(0., 0., 0.)), PTransformd(Vector3d(0., 0.5, 0)), PTransformd(Vector3d(0., 1.5, 0.)),
         PTransformd(Vector3d(0., 2.5, 0)), PTransformd(RotY(rbd::PI / 2.), Vector3d(0.5, 1., 0.))};

  for(size_t i = 0; i < res.size(); ++i)
  {
    BOOST_CHECK_SMALL((res[i].translation() - mbc2.bodyPosW[i].translation()).norm(), TOL);
    BOOST_CHECK_SMALL((res[i].rotation() - mbc2.bodyPosW[i].rotation()).norm(), TOL);
  }

  // check j1 rotX
  mbc2.q = {{}, {rbd::PI / 2.}, {0.}, {0.}, {1., 0., 0., 0.}};

  forwardKinematics(mb2, mbc2);

  res = {PTransformd(Vector3d(0., 0., 0.)), PTransformd(RotX(rbd::PI / 2.), Vector3d(0., 0.5, 0.)),
         PTransformd(RotX(rbd::PI / 2.), Vector3d(0., 0.5, 1.)), PTransformd(RotX(rbd::PI / 2.), Vector3d(0., 0.5, 2.)),
         PTransformd(RotX(rbd::PI / 2.), Vector3d(0.5, 0.5, 0.5))};

  for(size_t i = 0; i < res.size(); ++i)
  {
    BOOST_CHECK_SMALL((res[i].translation() - mbc2.bodyPosW[i].translation()).norm(), TOL);
    BOOST_CHECK_SMALL((res[i].rotation() - mbc2.bodyPosW[i].rotation()).norm(), TOL);
  }

  // test safe version
  BOOST_CHECK_NO_THROW(sForwardKinematics(mb2, mbc2));

  // bad number of body
  MultiBodyConfig mbcBadNrBody = mbc2;
  mbcBadNrBody.bodyPosW.resize(4);

  BOOST_CHECK_THROW(sForwardKinematics(mb2, mbcBadNrBody), std::domain_error);

  // bad number of generalized position variable
  MultiBodyConfig mbcBadNrQ = mbc2;
  mbcBadNrQ.q = {{0.}, {0.}, {0.}, {1., 0., 0., 0.}};

  BOOST_CHECK_THROW(sForwardKinematics(mb2, mbcBadNrQ), std::domain_error);

  // bad generalized position variable size
  MultiBodyConfig mbcBadQSize = mbc2;
  mbcBadQSize.q = {{}, {0.}, {0.}, {0.}, {1., 0., 0.}};

  BOOST_CHECK_THROW(sForwardKinematics(mb2, mbcBadQSize), std::domain_error);
}

BOOST_AUTO_TEST_CASE(FVTest)
{
  using namespace Eigen;
  using namespace sva;
  using namespace rbd;

  rbd::MultiBody mb, mb2;
  rbd::MultiBodyConfig mbc, mbc2;
  rbd::MultiBodyGraph mbg, mbg2;
  std::tie(mb, mbc, mbg) = makeXYZarm();
  std::tie(mb2, mbc2, mbg2) = makeXYZSarm();

  // check identity
  mbc.q = {{}, {0.}, {0.}, {0.}};
  mbc.alpha = {{}, {0.}, {0.}, {0.}};

  forwardKinematics(mb, mbc);
  forwardVelocity(mb, mbc);

  std::vector<MotionVecd> res = {MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero()),
                                 MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero())};

  BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(), mbc.bodyVelW.begin(), mbc.bodyVelW.end());

  // check rot X
  mbc.alpha = {{}, {1.}, {0.}, {0.}};
  forwardVelocity(mb, mbc);

  res = {MotionVecd(Vector6d::Zero()), MotionVecd(Vector3d(1., 0., 0.), Vector3d(0., 0., 0.)),
         MotionVecd(Vector3d(1., 0., 0.), Vector3d(0., 0., 1.)),
         MotionVecd(Vector3d(1., 0., 0.), Vector3d(0., 0., 2.))};

  BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(), mbc.bodyVelW.begin(), mbc.bodyVelW.end());

  // check rot Y
  mbc.alpha = {{}, {0.}, {1.}, {0.}};
  forwardVelocity(mb, mbc);

  res = {MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero()),
         MotionVecd(Vector3d(0., 1., 0.), Vector3d(0., 0., 0.)),
         MotionVecd(Vector3d(0., 1., 0.), Vector3d(0., 0., 0.))};

  BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(), mbc.bodyVelW.begin(), mbc.bodyVelW.end());

  // check rot Z
  mbc.alpha = {{}, {0.}, {0.}, {1.}};
  forwardVelocity(mb, mbc);

  res = {MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero()),
         MotionVecd(Vector3d(0., 0., 1.), Vector3d(0., 0., 0.))};

  BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(), mbc.bodyVelW.begin(), mbc.bodyVelW.end());

  // check rot X with 90 X rotation
  mbc.q = {{}, {rbd::PI / 2.}, {0.}, {0.}};
  mbc.alpha = {{}, {1.}, {0.}, {0.}};

  forwardKinematics(mb, mbc);
  forwardVelocity(mb, mbc);

  res = {MotionVecd(Vector6d::Zero()), MotionVecd(Vector3d(1., 0., 0.), Vector3d(0., 0., 0.)),
         MotionVecd(Vector3d(1., 0., 0.), Vector3d(0., -1., 0.)),
         MotionVecd(Vector3d(1., 0., 0.), Vector3d(0., -2., 0.))};

  for(size_t i = 0; i < res.size(); ++i)
  {
    BOOST_CHECK_SMALL((res[i].vector() - mbc.bodyVelW[i].vector()).norm(), TOL);
  }

  // check rot X with 90 Y rotation
  mbc.alpha = {{}, {0.}, {1.}, {0.}};

  forwardVelocity(mb, mbc);

  res = {MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero()),
         MotionVecd(Vector3d(0., 0., 1.), Vector3d(0., 0., 0.)),
         MotionVecd(Vector3d(0., 0., 1.), Vector3d(0., 0., 0.))};

  for(size_t i = 0; i < res.size(); ++i)
  {
    BOOST_CHECK_SMALL((res[i].vector() - mbc.bodyVelW[i].vector()).norm(), TOL);
  }

  // check rot X with 90 Z rotation
  mbc.alpha = {{}, {0.}, {0.}, {1.}};

  forwardVelocity(mb, mbc);

  res = {MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero()),
         MotionVecd(Vector3d(0., -1., 0.), Vector3d(0., 0., 0.))};

  // check identity
  mbc2.q = {{}, {0.}, {0.}, {0.}, {1., 0., 0., 0.}};
  mbc2.alpha = {{}, {0.}, {0.}, {0.}, {0., 0., 0.}};

  forwardKinematics(mb2, mbc2);
  forwardVelocity(mb2, mbc2);

  res = {MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero()),
         MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero())};

  BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(), mbc2.bodyVelW.begin(), mbc2.bodyVelW.end());

  // check spherical X
  mbc2.alpha = {{}, {0.}, {0.}, {0.}, {1., 0., 0.}};

  forwardVelocity(mb2, mbc2);

  res = {MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero()),
         MotionVecd(Vector6d::Zero()), MotionVecd(Vector3d(1., 0., 0.), Vector3d(0., 0., 0.))};

  BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(), mbc2.bodyVelW.begin(), mbc2.bodyVelW.end());

  // check spherical Y
  mbc2.alpha = {{}, {0.}, {0.}, {0.}, {0., 1., 0.}};

  forwardVelocity(mb2, mbc2);

  res = {MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero()),
         MotionVecd(Vector6d::Zero()), MotionVecd(Vector3d(0., 1., 0.), Vector3d(0., 0., 0.))};

  BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(), mbc2.bodyVelW.begin(), mbc2.bodyVelW.end());

  // check spherical Z
  mbc2.alpha = {{}, {0.}, {0.}, {0.}, {0., 0., 1.}};

  forwardVelocity(mb2, mbc2);

  res = {MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero()),
         MotionVecd(Vector6d::Zero()), MotionVecd(Vector3d(0., 0., 1.), Vector3d(0., 0., 0.))};

  BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(), mbc2.bodyVelW.begin(), mbc2.bodyVelW.end());
}

BOOST_AUTO_TEST_CASE(FreeFlyerTest)
{
  using namespace Eigen;
  using namespace sva;
  using namespace rbd;

  MultiBodyGraph mbg;

  double mass = 1.;
  Matrix3d I = Matrix3d::Identity();
  Vector3d h = Vector3d::Zero();

  RBInertiad rbi(mass, h, I);

  Body b0(rbi, "b0");

  mbg.addBody(b0);

  MultiBody mb = mbg.makeMultiBody("b0", false);

  MultiBodyConfig mbc(mb);

  // check identity
  mbc.q = {{1., 0., 0., 0., 0., 0., 0.}};
  mbc.alpha = {{0., 0., 0., 0., 0., 0.}};

  forwardKinematics(mb, mbc);
  forwardVelocity(mb, mbc);

  std::vector<MotionVecd> res = {MotionVecd(Vector6d::Zero())};

  BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(), mbc.bodyVelW.begin(), mbc.bodyVelW.end());

  // check Y Rot
  Quaterniond q = Quaterniond::Identity();
  mbc.q = {{q.w(), q.x(), q.y(), q.z(), 1., 0., 0.}};
  mbc.alpha = {{0., 1., 0., 0., 0., 0.}};

  forwardKinematics(mb, mbc);
  forwardVelocity(mb, mbc);

  res = {MotionVecd(Vector3d(0., 1., 0.), Vector3d(0., 0., 0.))};

  BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(), mbc.bodyVelW.begin(), mbc.bodyVelW.end());
}

BOOST_AUTO_TEST_CASE(EulerTest)
{
  using namespace std;
  using namespace Eigen;
  using namespace rbd;

  // 1 dof joint

  // static
  vector<double> q = {0.};

  jointIntegration(Joint::Rev, {0.}, {0.}, 1., q);

  BOOST_CHECK_EQUAL(q[0], 0.);

  // moving
  jointIntegration(Joint::Rev, {1.}, {0.}, 1., q);

  BOOST_CHECK_EQUAL(q[0], 1.);

  // free

  // static
  q = {1., 0., 0., 0., 0., 0., 0.};
  vector<double> goalQ = q;
  jointIntegration(Joint::Spherical, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, 1., q);
  BOOST_CHECK_EQUAL_COLLECTIONS(q.begin(), q.end(), goalQ.begin(), goalQ.end());

  // X unit move
  goalQ = {1., 0., 0., 0., 1., 0., 0.},
  jointIntegration(Joint::Free, {0., 0., 0., 1., 0., 0.}, {0., 0., 0., 0., 0., 0.}, 1., q);
  BOOST_CHECK_EQUAL_COLLECTIONS(q.begin(), q.end(), goalQ.begin(), goalQ.end());

  // X unit rot
  const double pi = rbd::PI;
  q = {1., 0., 0., 0., 0., 0., 0.};
  goalQ = {std::cos(pi / 4), std::sin(pi / 4), 0., 0., 0., 0., 0.},
  jointIntegration(Joint::Free, {pi / 2., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, 1., q);
  BOOST_CHECK_EQUAL_COLLECTIONS(q.begin(), q.end(), goalQ.begin(), goalQ.end());

  // planar

  // static
  q = {0., 0., 0.};
  goalQ = {0, 0., 0.};
  jointIntegration(Joint::Planar, {0, 0., 0.}, {0., 0., 0.}, 1., q);
  BOOST_CHECK_EQUAL_COLLECTIONS(q.begin(), q.end(), goalQ.begin(), goalQ.end());

  // rotation only
  goalQ = {pi / 2, 0., 0.};
  jointIntegration(Joint::Planar, {pi / 2, 0., 0.}, {0., 0., 0.}, 1., q);
  BOOST_CHECK_EQUAL_COLLECTIONS(q.begin(), q.end(), goalQ.begin(), goalQ.end());

  // X unit move
  q = {0., 0., 0.};
  goalQ = {0., 1., 0.};
  jointIntegration(Joint::Planar, {0., 1., 0.}, {0., 0., 0.}, 1., q);
  BOOST_CHECK_EQUAL_COLLECTIONS(q.begin(), q.end(), goalQ.begin(), goalQ.end());
}

/// @return norm of the finite diff motion vector minus model motion vector
double testEulerInteg(rbd::Joint::Type jType,
                      const Eigen::Vector3d & axis,
                      const Eigen::VectorXd & q,
                      const Eigen::VectorXd & alpha,
                      double timeStep = 0.001)
{
  using namespace std;
  using namespace Eigen;
  using namespace rbd;

  Joint j(jType, axis, true, std::string("0"));
  const auto nparams = static_cast<size_t>(j.params());
  const auto ndofs = static_cast<size_t>(j.dof());
  std::vector<double> qVec(nparams), alphaVec(ndofs), alphaDVec(ndofs);
  for(int i = 0; i < j.params(); ++i)
  {
    const auto ui = static_cast<size_t>(i);
    qVec[ui] = q(i);
  }
  for(int i = 0; i < j.dof(); ++i)
  {
    const auto ui = static_cast<size_t>(i);
    alphaVec[ui] = alpha(i);
    alphaDVec[ui] = 0.;
  }

  sva::PTransformd initPos(j.pose(qVec));
  sva::MotionVecd motion(j.motion(alphaVec));

  jointIntegration(jType, alphaVec, alphaDVec, timeStep, qVec);
  sva::PTransformd endPos(j.pose(qVec));

  // linear velocity is set in initPos frame
  Vector3d linVel((initPos.rotation() * (endPos.translation() - initPos.translation())) / timeStep);
  // rotation velocity is also in initPos frame
  Matrix3d rotErr(endPos.rotation() * initPos.rotation().transpose());
  Vector3d angVel = sva::rotationVelocity(rotErr) / timeStep;
  sva::MotionVecd motionDiff(angVel, linVel);

  return (motionDiff - motion).vector().norm();
}

BOOST_AUTO_TEST_CASE(EulerTestV2)
{
  using namespace Eigen;
  using namespace rbd;

  for(int i = 0; i < 100; ++i)
  {
    BOOST_CHECK_SMALL(
        testEulerInteg(Joint::Rev, Vector3d::Random().normalized(), VectorXd::Random(1), VectorXd::Random(1)), 1e-4);
  }

  for(int i = 0; i < 100; ++i)
  {
    BOOST_CHECK_SMALL(
        testEulerInteg(Joint::Prism, Vector3d::Random().normalized(), VectorXd::Random(1), VectorXd::Random(1)), 1e-4);
  }

  for(int i = 0; i < 100; ++i)
  {
    BOOST_CHECK_SMALL(
        testEulerInteg(Joint::Spherical, Vector3d::UnitZ(), VectorXd::Random(4).normalized(), VectorXd::Random(3)),
        1e-4);
  }

  // The current test does not make sense for free joints
  // for(int i = 0; i < 100; ++i)
  //{
  //  VectorXd q(VectorXd::Random(7));
  //  q.head<4>() /= q.head<4>().norm();
  //  BOOST_CHECK_SMALL(testEulerInteg(Joint::Free, Vector3d::UnitZ(), q, VectorXd::Random(6)), 1e-4);
  //}

  for(int i = 0; i < 100; ++i)
  {
    BOOST_CHECK_SMALL(testEulerInteg(Joint::Planar, Vector3d::UnitZ(), VectorXd::Random(3), VectorXd::Random(3), 1e-4),
                      1e-3);
  }

  for(int i = 0; i < 100; ++i)
  {
    BOOST_CHECK_SMALL(testEulerInteg(Joint::Cylindrical, Vector3d::UnitZ(), VectorXd::Random(2), VectorXd::Random(2)),
                      1e-4);
  }
}

BOOST_AUTO_TEST_CASE(FATest)
{
  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;
  std::tie(mb, mbc, mbg) = makeXYZSarm();

  forwardKinematics(mb, mbc);
  forwardVelocity(mb, mbc);
  forwardAcceleration(mb, mbc);

  for(size_t i = 0; i < static_cast<size_t>(mb.nrBodies()); ++i)
  {
    BOOST_CHECK_SMALL(mbc.bodyAccB[i].vector().norm(), TOL);
  }

  std::vector<rbd::Jacobian> jacs(static_cast<size_t>(mb.nrBodies()));
  for(int i = 0; i < mb.nrBodies(); ++i)
  {
    const auto ui = static_cast<size_t>(i);
    jacs[ui] = rbd::Jacobian(mb, mb.body(i).name());
  }

  Eigen::MatrixXd fullJac(6, mb.nrDof());
  Eigen::MatrixXd fullJacDot(6, mb.nrDof());

  for(int i = 0; i < 10; ++i)
  {
    Eigen::VectorXd q(mb.nrParams()), alpha(mb.nrDof()), alphaD(mb.nrDof());
    q.setRandom();
    q.tail<4>().normalize();
    alpha.setRandom();
    alphaD.setRandom();

    rbd::vectorToParam(q, mbc.q);
    rbd::vectorToParam(alpha, mbc.alpha);
    rbd::vectorToParam(alphaD, mbc.alphaD);

    forwardKinematics(mb, mbc);
    forwardVelocity(mb, mbc);
    forwardAcceleration(mb, mbc);

    for(size_t j = 0; j < static_cast<size_t>(mb.nrBodies()); ++j)
    {
      const Eigen::MatrixXd & jac = jacs[j].bodyJacobian(mb, mbc);
      const Eigen::MatrixXd & jacDot = jacs[j].bodyJacobianDot(mb, mbc);

      jacs[j].fullJacobian(mb, jac, fullJac);
      jacs[j].fullJacobian(mb, jacDot, fullJacDot);

      Eigen::Vector6d acc = fullJac * alphaD + fullJacDot * alpha;
      BOOST_CHECK_SMALL((mbc.bodyAccB[j].vector() - acc).norm(), TOL);
    }
  }
}

// test forward acceleration against inverse dynamics
BOOST_AUTO_TEST_CASE(FAGravityTest)
{
  using namespace Eigen;

  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc, mbcId;
  rbd::MultiBodyGraph mbg;

  std::tie(mb, mbc, mbg) = makeXYZSarm();

  rbd::InverseDynamics id(mb);
  for(int i = 0; i < 10; ++i)
  {
    Eigen::VectorXd q(mb.nrParams()), alpha(mb.nrDof()), alphaD(mb.nrDof());
    q.setRandom();
    q.tail<4>().normalize();
    alpha.setRandom();
    alphaD.setRandom();

    rbd::vectorToParam(q, mbc.q);
    rbd::vectorToParam(alpha, mbc.alpha);
    rbd::vectorToParam(alphaD, mbc.alphaD);

    forwardKinematics(mb, mbc);
    forwardVelocity(mb, mbc);
    mbcId = mbc;

    // compute acceleration through forwardAcceleration and
    // inverseDynamics
    forwardAcceleration(mb, mbc, sva::MotionVecd(Vector3d::Zero(), mbc.gravity));
    id.inverseDynamics(mb, mbcId);

#ifdef __i386__
    for(size_t j = 0; j < mbc.bodyAccB.size(); ++j)
    {
      BOOST_CHECK_SMALL((mbc.bodyAccB[j] - mbcId.bodyAccB[j]).vector().array().abs().sum(), TOL);
    }
#else
    BOOST_CHECK_EQUAL_COLLECTIONS(mbc.bodyAccB.begin(), mbc.bodyAccB.end(), mbcId.bodyAccB.begin(),
                                  mbcId.bodyAccB.end());
#endif
  }
}

BOOST_AUTO_TEST_CASE(IKTest)
{
  using namespace Eigen;
  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;

  std::tie(mb, mbc, mbg) = makeXYZarm();

  rbd::InverseKinematics ik(mb, 3);

  rbd::forwardKinematics(mb, mbc);
  rbd::forwardVelocity(mb, mbc);

  sva::PTransformd target(mbc.bodyPosW[3]);
  BOOST_CHECK(ik.inverseKinematics(mb, mbc, target));

  Eigen::Vector3d pos_vec(mbc.q[1][0], mbc.q[2][0], mbc.q[3][0]);
  Eigen::Vector3d solution(0, 0, 0);
  BOOST_CHECK_SMALL((pos_vec - solution).norm(), TOL);

  solution[0] = 1.;
  mbc.q[1][0] = 1.;
  rbd::forwardKinematics(mb, mbc);
  target = sva::PTransformd(mbc.bodyPosW[3]);
  mbc.q[1][0] = 0.;
  rbd::forwardKinematics(mb, mbc);
  BOOST_CHECK(ik.inverseKinematics(mb, mbc, target));
  pos_vec = Eigen::Vector3d(mbc.q[1][0], mbc.q[2][0], mbc.q[3][0]);
  BOOST_CHECK_SMALL((pos_vec - solution).norm(), TOL);

  solution = Eigen::Vector3d(0., 1., 0.);
  mbc.q[1][0] = 0.;
  mbc.q[2][0] = 1.;
  mbc.q[3][0] = 0.;
  rbd::forwardKinematics(mb, mbc);
  target = sva::PTransformd(mbc.bodyPosW[3]);
  mbc.q[2][0] = 0.;
  rbd::forwardKinematics(mb, mbc);
  BOOST_CHECK(ik.inverseKinematics(mb, mbc, target));
  pos_vec = Eigen::Vector3d(mbc.q[1][0], mbc.q[2][0], mbc.q[3][0]);
  BOOST_CHECK_SMALL((pos_vec - solution).norm(), TOL);

  solution = Eigen::Vector3d::Random();
  mbc.q[1][0] = solution[0];
  mbc.q[2][0] = solution[1];
  mbc.q[3][0] = solution[2];
  rbd::forwardKinematics(mb, mbc);
  target = sva::PTransformd(mbc.bodyPosW[3]);
  mbc.zero(mb);
  rbd::forwardKinematics(mb, mbc);
  BOOST_CHECK(ik.inverseKinematics(mb, mbc, target));
  pos_vec = Eigen::Vector3d(mbc.q[1][0], mbc.q[2][0], mbc.q[3][0]);
  BOOST_CHECK_SMALL((pos_vec - solution).norm(), TOL);
}

BOOST_AUTO_TEST_CASE(FailureIKTest)
{
  using namespace Eigen;
  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;

  std::tie(mb, mbc, mbg) = makeXYZarm();

  rbd::InverseKinematics ik(mb, 3);

  rbd::forwardKinematics(mb, mbc);
  rbd::forwardVelocity(mb, mbc);

  // This target is outside the reach of the arm
  sva::PTransformd target(sva::RotX(rbd::PI / 2), Eigen::Vector3d(0., 0.5, 2.5));
  BOOST_CHECK(!ik.inverseKinematics(mb, mbc, target));

  Eigen::VectorXd q_target(mb.nrParams());
  Eigen::VectorXd q(mb.nrParams());

  q_target << rbd::PI / 2, 0, 0;
  rbd::paramToVector(mbc.q, q);

  BOOST_CHECK_SMALL((q_target - q).norm(), TOL);

  /* This target is reachable, but IK will fail if given
   * a too low maximum number of iterations */
  ik.max_iterations_ = 10;
  sva::PTransformd reachable_target(sva::RotX(-rbd::PI / 2), Eigen::Vector3d(0., 0.5, -2.));
  BOOST_CHECK(!ik.inverseKinematics(mb, mbc, reachable_target));
  ik.max_iterations_ = 40;
  BOOST_CHECK(ik.inverseKinematics(mb, mbc, reachable_target));
}
