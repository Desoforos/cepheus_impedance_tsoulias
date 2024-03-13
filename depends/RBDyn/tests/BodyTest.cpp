/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// includes
// std
#include <iostream>

// boost
#define BOOST_TEST_MODULE BodyTest
#include <boost/test/unit_test.hpp>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
#include "RBDyn/Body.h"

BOOST_AUTO_TEST_CASE(BodyTest)
{
  using namespace Eigen;
  using namespace sva;
  using namespace rbd;

  double mass = 1.;
  Matrix3d I;
  I << 1., 2., 3., 2., 1., 4., 3., 4., 1.;
  Vector3d h = Vector3d::Random() * 100.;

  RBInertiad rbi(mass, h, I);

  // Test first constructor
  Body b1(rbi, "b1");

  BOOST_CHECK_EQUAL(b1.name(), "b1");
  BOOST_CHECK_EQUAL(b1.inertia().mass(), rbi.mass());
  BOOST_CHECK_EQUAL(b1.inertia().momentum(), rbi.momentum());
  BOOST_CHECK_EQUAL(b1.inertia().inertia(), rbi.inertia());

  // Test second constructor
  Body b2(mass, Vector3d::UnitX(), I, "b2");

  BOOST_CHECK_EQUAL(b2.name(), "b2");
  BOOST_CHECK_EQUAL(b2.inertia().mass(), mass);
  BOOST_CHECK_EQUAL(b2.inertia().momentum(), mass * Vector3d::UnitX());
  BOOST_CHECK_EQUAL(b2.inertia().inertia(), I);

  // Test operator==
  BOOST_CHECK_EQUAL(b1, b1);
  BOOST_CHECK_NE(b1, b2);

  // Test operator!=
  BOOST_CHECK(!(b1 != b1));
  BOOST_CHECK(b1 != b2);
}
