//kalman filter unit test
//Author:zqlee@whu.edu.cn
//DATE  :2017/04/15


#include "IMU_LIB/imucommon.h"
#include "IMU_LIB/kalmanfilter.h"
#include "gtest/gtest.h"

TEST(KalmanFilterTEST, _initial)
{
  IMU_LIB::KalmanFilter kffortest(15, 6);
  EXPECT_EQ(15, kffortest.q_);
  EXPECT_EQ(15, kffortest.Xk_.rows());
  EXPECT_EQ(6, kffortest.r_);
  EXPECT_EQ(6, kffortest.Rk_.cols());
}

TEST(KalmanFilterTEST, _setval)
{
  Eigen::Vector3d att0(0.0, 0.0, -1.5812683023068610),
    vn0(0.0,0.0,0.0),
    pos0(0.59770629339601844, 1.9008322240407358, 380.00000000000000);
  IMU_LIB::PSINS sins(IMU_LIB::rv2q(att0), vn0, pos0);
  const IMU_LIB::IMUCommonStruct imucommonpara;
  IMU_LIB::KalmanFilter kffortest(15, 6);

  //setPk
  kffortest.SetPk(imucommonpara.deg_,
    imucommonpara.deg_,
    10.0*imucommonpara.deg_,
    1.0,
    1.0,
    1.0,
    10.0 / imucommonpara.Re_,
    10.0 / imucommonpara.Re_,
    10.0,
    0.01*imucommonpara.dph_,
    0.01*imucommonpara.dph_,
    0.01*imucommonpara.dph_,
    1.000*imucommonpara.ug_,
    1.000*imucommonpara.ug_,
    100.0*imucommonpara.ug_);
  for (int i = 0; i < kffortest.q_; ++i)
  {
    for (int j = 0; j < kffortest.q_; ++j)
    {
      if (i == j)
      {
        continue;
      }
      else
      {
        EXPECT_DOUBLE_EQ(0.0, kffortest.Pk_(i, j));
      }
    }
  }
  EXPECT_DOUBLE_EQ(0.00030461741978670797, kffortest.Pk_(0, 0));
  EXPECT_DOUBLE_EQ(0.00030461741978670797, kffortest.Pk_(1, 1));
  EXPECT_DOUBLE_EQ(0.030461741978670798, kffortest.Pk_(2, 2)); 
  EXPECT_DOUBLE_EQ(1.0000000000000000, kffortest.Pk_(3, 3));
  EXPECT_DOUBLE_EQ(1.0000000000000000, kffortest.Pk_(4, 4));
  EXPECT_DOUBLE_EQ(1.0000000000000000, kffortest.Pk_(5, 5));
  EXPECT_DOUBLE_EQ(2.4581722576473319e-012, kffortest.Pk_(6, 6));
  EXPECT_DOUBLE_EQ(2.4581722576473319e-012, kffortest.Pk_(7, 7));
  EXPECT_DOUBLE_EQ(100.00000000000000, kffortest.Pk_(8, 8));

  //setQt
  kffortest.SetQt(0.001*imucommonpara.dpsh_,
    0.001*imucommonpara.dpsh_,
    0.001*imucommonpara.dpsh_,
    1.0*imucommonpara.ugpsHz_,
    1.0*imucommonpara.ugpsHz_,
    1.0*imucommonpara.ugpsHz_,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    10.0*imucommonpara.ugpsh_,
    10.0*imucommonpara.ugpsh_,
    10.0*imucommonpara.ugpsh_);
  for (int i = 0; i < kffortest.q_; ++i)
  {
    for (int j = 0; j < kffortest.q_; ++j)
    {
      if (i == j)
      {
        continue;
      }
      else
      {
        EXPECT_DOUBLE_EQ(0.0, kffortest.Qt_(i, j));
      }
    }
  }
  EXPECT_DOUBLE_EQ(8.4615949940752211e-014, kffortest.Qt_(0, 0));
  EXPECT_DOUBLE_EQ(8.4615949940752211e-014, kffortest.Qt_(1, 1));
  EXPECT_DOUBLE_EQ(8.4615949940752211e-014, kffortest.Qt_(2, 2));
  EXPECT_DOUBLE_EQ(9.5654791755363532e-011, kffortest.Qt_(3, 3));
  EXPECT_DOUBLE_EQ(9.5654791755363532e-011, kffortest.Qt_(4, 4));
  EXPECT_DOUBLE_EQ(9.5654791755363532e-011, kffortest.Qt_(5, 5));
  EXPECT_DOUBLE_EQ(2.6570775487600983e-012, kffortest.Qt_(12, 12));
  EXPECT_DOUBLE_EQ(2.6570775487600983e-012, kffortest.Qt_(13, 13));
  EXPECT_DOUBLE_EQ(2.6570775487600983e-012, kffortest.Qt_(14, 14));

  //setRk
  kffortest.SetRk(0.1, 0.1, 0.1, 1.0 / imucommonpara.Re_, 1.0 / imucommonpara.Re_, 1.0);
  for (int i = 0; i < kffortest.r_; ++i)
  {
    for (int j = 0; j < kffortest.r_; ++j)
    {
      if (i == j)
      {
        continue;
      }
      else
      {
        EXPECT_DOUBLE_EQ(0.0, kffortest.Rk_(i, j));
      }
    }
  }
  EXPECT_DOUBLE_EQ(0.010000000000000002, kffortest.Rk_(0, 0));
  EXPECT_DOUBLE_EQ(0.010000000000000002, kffortest.Rk_(1, 1));
  EXPECT_DOUBLE_EQ(0.010000000000000002, kffortest.Rk_(2, 2));
  EXPECT_DOUBLE_EQ(2.4581722576473315e-014, kffortest.Rk_(3, 3));
  EXPECT_DOUBLE_EQ(2.4581722576473315e-014, kffortest.Rk_(4, 4));
  EXPECT_DOUBLE_EQ(1.0000000000000000, kffortest.Rk_(5, 5));

  kffortest.SetHk();
  //setFt
  Eigen::Vector3d wm1(0.00000000000000000, 9.2114599410811746e-006, 0.00000000000000000),
    wm2(0.00000000000000000, 0.00000000000000000, 0.00000000000000000);
  Eigen::Vector3d vm1(0.00000000000000000, 0.0048901633857000000, 0.097803267713999997),
    vm2(0.00000000000000000, -0.00000000000000000, 0.097803267713999997);
  std::vector<Eigen::Vector3d> wm, vm;
  wm.push_back(wm1);
  wm.push_back(wm2);
  vm.push_back(vm1);
  vm.push_back(vm2);
  sins.Update(wm, vm, 2, 0.01);
  kffortest.SetFt(sins);
  EXPECT_DOUBLE_EQ(0.0, kffortest.Ft_(0, 0));
  EXPECT_DOUBLE_EQ(4.1036225727798722e-005, kffortest.Ft_(0, 1));
  EXPECT_DOUBLE_EQ(-6.0278706931140336e-005, kffortest.Ft_(0, 2));
  EXPECT_DOUBLE_EQ(0.0, kffortest.Ft_(0, 3));
  EXPECT_DOUBLE_EQ(-1.5733116909832734e-007, kffortest.Ft_(0, 4));
  EXPECT_DOUBLE_EQ(0.0000000000000000, kffortest.Ft_(0, 5));
  EXPECT_DOUBLE_EQ(0.010472604784197204, kffortest.Ft_(0, 9));
  EXPECT_DOUBLE_EQ(-0.99994516076999840, kffortest.Ft_(0, 10));
  EXPECT_DOUBLE_EQ(0.99994516072830197, kffortest.Ft_(1, 9));
  EXPECT_DOUBLE_EQ(0.010472604795754181, kffortest.Ft_(1, 10));
  EXPECT_DOUBLE_EQ(-0.99999999995673117, kffortest.Ft_(2, 11));
  EXPECT_DOUBLE_EQ(-9.7803267713999986, kffortest.Ft_(3, 1));

  //timeUpdate
  kffortest.TimeUpdate(sins.nts_);
  for (int i = 0; i < kffortest.q_; ++i)
  {
    EXPECT_DOUBLE_EQ(0.0, kffortest.Xk_(i));
  }
  kffortest.SetZk(
    0.0048897616006432342,
    -5.2712177674583155e-005,
    -0.00030351203538913295,
    -8.2933659939499194e-014,
    9.2639229620772312e-012,
    -3.0351203577083652e-006);
  kffortest.MeasUpdate();
  //EXPECT_DOUBLE_EQ(0.0, kffortest.Xk_(0));
  //EXPECT_DOUBLE_EQ(0.0, kffortest.Xk_(1));
  //EXPECT_DOUBLE_EQ(0.0, kffortest.Xk_(2));
  EXPECT_DOUBLE_EQ(0.0048413485417853966, kffortest.Xk_(3));
  EXPECT_DOUBLE_EQ(-5.2190279937874239e-005, kffortest.Xk_(4));
  EXPECT_DOUBLE_EQ(-0.00030050697388340298, kffortest.Xk_(5));
  EXPECT_DOUBLE_EQ(-8.3738232832735526e-014, kffortest.Xk_(6));
  EXPECT_DOUBLE_EQ(9.3538273218685337e-012, kffortest.Xk_(7));
  EXPECT_DOUBLE_EQ(-3.0645963878902100e-006, kffortest.Xk_(8));

}