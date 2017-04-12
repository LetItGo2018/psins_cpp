//IMU common paraments struct unit test
//Author:zqlee@whu.edu.cn
//DATE  :2017/04/10

#include "IMU_LIB/imucommon.h"
#include "gtest/gtest.h"

TEST(IMUCommonStrcutTEST, _initial)
{
  const IMU_LIB::IMUCommonStruct imucommonfortest;
  double Re = (6378137.0);
  double f = (1.0 / 298.257); 
  double wie = (7.2921151467e-5);
  double g0 = (9.7803267714);

  EXPECT_DOUBLE_EQ(6378137.0, imucommonfortest.Re_) << "test Re_";
  EXPECT_DOUBLE_EQ(1.0 / 298.257, imucommonfortest.f_) << "test f_";
  EXPECT_DOUBLE_EQ(7.2921151467e-5, imucommonfortest.wie_) << "test wie_";
  EXPECT_DOUBLE_EQ(9.7803267714, imucommonfortest.g0_) << "test g0_";
}
