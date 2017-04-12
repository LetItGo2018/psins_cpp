//IMU unit test
//Author:zqlee@whu.edu.cn
//DATE  :2017/04/11

#include "IMU_LIB/imu.h"
#include "gtest/gtest.h"

TEST(IMUTEST,_cnscl)
{
  Eigen::Vector3d wm1(-0.099134701513277898, 0.14730578886832138, 0.02722713633111154), 
                  wm2(-0.099134701513277898, 0.14032447186034408, 0.029321531433504733), 
                  wm3(-0.098436569812480182, 0.12775810124598494, 0.037699111843077518),
                  wm4(-0.10262536001726656, 0.11588986233242347, 0.045378560551852569);
  Eigen::Vector3d vm1(8.1476917083333333, -0.37592158333333331, -2.4026292499999999),
                  vm2(8.033280791666666, -0.40861041666666664, -2.4026292499999999), 
                  vm3(7.8861810416666662, -0.42495483333333334, -2.4353180833333332),
                  vm4(7.8289755833333325, -0.37592158333333331, -2.4680069166666665);
  std::vector<Eigen::Vector3d> wm, vm;
  wm.push_back(wm1);
  wm.push_back(wm2);
  wm.push_back(wm3);
  wm.push_back(wm4);
  vm.push_back(vm1);
  vm.push_back(vm2);
  vm.push_back(vm3);
  vm.push_back(vm4);

  IMU_LIB::IMU imufortest;
  imufortest.cnscl(wm, vm, 4);
  EXPECT_DOUBLE_EQ(-0.392002857415100, imufortest.Getphim()(0)) << "phim(0)";
  EXPECT_DOUBLE_EQ(0.534678650577220, imufortest.Getphim()(1)) << "phim(1)";
  EXPECT_DOUBLE_EQ(0.14751580368281764, imufortest.Getphim()(2)) << "phim(2)";
  EXPECT_DOUBLE_EQ(29.236648481417728, imufortest.Getdvbm()(0)) << "dvbm(0)";
  EXPECT_DOUBLE_EQ(-1.606843525348773, imufortest.Getdvbm()(1)) << "dvbm(1)";
  EXPECT_DOUBLE_EQ(-18.313343943483485, imufortest.Getdvbm()(2)) << "dvbm(2)";
}