//IMU earth parameter struct unit test
//Author:zqlee@whu.edu.cn
//DATE  :2017/04/10

#include "IMU_LIB/imuearth.h"

#include <Eigen/Core>
#include "gtest/gtest.h"

TEST(IMUEarthParaStructTEST, _update)
{
  IMU_LIB::IMUCommonStruct imuvommonpara;
  Eigen::Vector3d initialpos(34.246048*imuvommonpara.deg_, 108.909664*imuvommonpara.deg_, 380);
  Eigen::Vector3d initialvn = Eigen::Vector3d::Zero();
  IMU_LIB::IMUEarthPara imuearth;
  EXPECT_DOUBLE_EQ(imuvommonpara.Re_, imuearth.a_)<< "test initial a_";
  imuearth.Update(initialpos, initialvn);
  EXPECT_DOUBLE_EQ(0.562747911987778, imuearth.sl_) << "test sl";
  EXPECT_DOUBLE_EQ(0.82662856686264863, imuearth.cl_) << "test cl";
  EXPECT_DOUBLE_EQ(6.356019635085974e+06, imuearth.RMh_) << "test RMh_";
  EXPECT_DOUBLE_EQ(0.0, imuearth.wnie_(0)) << "test wnie_(0)";
  EXPECT_DOUBLE_EQ(6.027870693114031e-05, imuearth.wnie_(1)) << "test wnie_(1)";
  EXPECT_DOUBLE_EQ(4.1036225727798722e-05, imuearth.wnie_(2)) << "test wnie_(2)";
  EXPECT_DOUBLE_EQ(0.0, imuearth.wnin_(0)) << "test wnin_(0)";
  EXPECT_DOUBLE_EQ(6.027870693114031e-05, imuearth.wnin_(1)) << "test wnin_(1)";
  EXPECT_DOUBLE_EQ(4.1036225727798722e-05, imuearth.wnin_(2)) << "test wnin_(2)";
}