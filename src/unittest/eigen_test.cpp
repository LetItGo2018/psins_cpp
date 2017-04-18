#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "gtest/gtest.h"

TEST(EIGENTEST, _eig)
{
  /*Eigen::Matrix4d matfortest;
  matfortest << 0.005431096264413, -1.078089277776487e-04, -0.010263434660863, 1.584060717549041e-04,
  -1.078089277776487e-04, 0.019405362096755, -1.584060723162137e-04, -0.010263565667247,
  -0.010263434660863, -1.584060723162137e-04, 0.019405553609840, -1.078688018776524e-04,
  1.584060717549041e-04, -0.010263565667247, -1.078688018776524e-04, 0.005431289625682;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eig(matfortest);
  Eigen::Matrix4d vtype = eig.eigenvectors();*/
  Eigen::Vector3d w(1, 2, 3),v(4,5,6);
  Eigen::Matrix3d b = w*v.transpose();
  EXPECT_EQ(1, 1);
}