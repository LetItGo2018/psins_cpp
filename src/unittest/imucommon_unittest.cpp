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

TEST(IMUCommonStructTEST, _rv2q)
{
  Eigen::Vector3d vfortest(0.1, 0.1, 0.1);
  Eigen::Quaterniond quafortest = IMU_LIB::rv2q(vfortest);

  EXPECT_DOUBLE_EQ(0.996252343164141, quafortest.w());
  EXPECT_DOUBLE_EQ(0.049937523433315173, quafortest.x());
  EXPECT_DOUBLE_EQ(0.049937523433315173, quafortest.y());
  EXPECT_DOUBLE_EQ(0.049937523433315173, quafortest.z());
}

TEST(IMUCommonStructTEST, _q2rv)
{
  Eigen::Vector3d vfortest(0.1, 0.1, 0.1);
  Eigen::Quaterniond quafortest = IMU_LIB::rv2q(vfortest);
  Eigen::Vector3d vret = IMU_LIB::q2rv(quafortest);
  EXPECT_DOUBLE_EQ(0.996252343164141, quafortest.w());
  EXPECT_DOUBLE_EQ(0.049937523433315173, quafortest.x());
  EXPECT_DOUBLE_EQ(0.049937523433315173, quafortest.y());
  EXPECT_DOUBLE_EQ(0.049937523433315173, quafortest.z());

  for (int i = 0; i < 3; ++i)
  {
    EXPECT_DOUBLE_EQ(vret(i), vfortest(i));
  }
}

TEST(IMUCommonStructTEST, _mat2rv)
{
  Eigen::Vector3d vfortest = Eigen::Vector3d::Zero();
  Eigen::Matrix3d matfortest;
  matfortest << 0.085309194526092, 0.996249649044780, -0.014456075105735,
    -0.996341454551220, 0.085224776096737, -0.006359519008647,
    -0.005103652816334, 0.014945712342151, 0.999875281427897;
  Eigen::Quaterniond quafortest(0.736615444457066, 0.007230784906533335, -0.0031741196711855342, -0.676265722701711);
  Eigen::Quaterniond qua(matfortest);
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      EXPECT_FLOAT_EQ(quafortest.matrix()(i, j), matfortest(i, j));
    }
  }
  vfortest = IMU_LIB::mat2rv(matfortest);

  for (int i = 0; i < 3; ++i)
  {
    EXPECT_DOUBLE_EQ(qua.coeffs()(i), quafortest.coeffs()(i));
  }
  EXPECT_DOUBLE_EQ(0.01588209124499573, vfortest(0));
  EXPECT_DOUBLE_EQ(-0.0069718099614269162, vfortest(1));
  EXPECT_DOUBLE_EQ(-1.485387001915522, vfortest(2));
}

TEST(IMUCommonStructTEST, _cnb2qua)
{
  Eigen::Vector3d vfortest = Eigen::Vector3d::Zero();
  Eigen::Matrix3d matfortest;
  matfortest << 0.085309194526092, 0.996249649044780, -0.014456075105735,
    -0.996341454551220, 0.085224776096737, -0.006359519008647,
    -0.005103652816334, 0.014945712342151, 0.999875281427897;
  Eigen::Quaterniond quafortest(0.736615444457066, 0.007230784906533335, -0.0031741196711855342, -0.676265722701711);

  Eigen::Quaterniond qua=IMU_LIB::cnb2qua(matfortest);
  for (int i = 0; i < 4; ++i)
  {
    EXPECT_FLOAT_EQ(qua.coeffs()(i), quafortest.coeffs()(i));
  }

}

TEST(IMUCommonStructTEST, _cnb2att)
{
  Eigen::Matrix3d matfortest;
  matfortest << 0.085309194526092, 0.996249649044780, -0.014456075105735,
    -0.996341454551220, 0.085224776096737, -0.006359519008647,
    -0.005103652816334, 0.014945712342151, 0.999875281427897;
  
  Eigen::Vector3d att = IMU_LIB::cnb2att(matfortest);
  Eigen::Matrix3d cnb = IMU_LIB::att2cnb(att);

  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      EXPECT_FLOAT_EQ(cnb(i, j), matfortest(i, j));
    }
  }
}