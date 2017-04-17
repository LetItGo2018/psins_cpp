//the IMU class
//Reference:
//       [1]严恭敏, 翁浚. 捷联惯导算法与组合导航原理讲义
//       [2]严恭敏. PSINS Toolbox for Matlab(160731)
//Author:zqlee@whu.edu.cn
//DATE  :2017/04/10

#include "IMU_LIB/imu.h"

namespace IMU_LIB
{
double IMU::gyrbiasRw2_ = 2.0e-5*2.0e-5*0.005*1e3;  //2e-12*1e3
double IMU::accbiasRw2_ = 3.0e-3*3.0e-3*0.005*1e2;  //4.5e-8*1e2

Eigen::Matrix3d IMU::gyrmeascov_ = Eigen::Matrix3d::Identity()*1.7e-4*1.7e-4 / 0.005 * 10;       // sigma_g * sigma_g / dt, ~6e-6*10
Eigen::Matrix3d IMU::accmeascov_ = Eigen::Matrix3d::Identity()*2.0e-3*2.0e-3 / 0.005 * 10;       // sigma_a * sigma_a / dt, ~8e-4*10

// covariance of bias random walk
Eigen::Matrix3d IMU::gyrbiasRWcov_ = Eigen::Matrix3d::Identity()*gyrbiasRw2_;     // sigma_gw * sigma_gw * dt, ~2e-12
Eigen::Matrix3d IMU::accbiasRWcov_ = Eigen::Matrix3d::Identity()*accbiasRw2_;     // sigma_aw * sigma_aw * dt, ~4.5e-8

IMU::IMU() : nSamples_(0), prefirst_(1)
{
  phim_.setZero();
  dvbm_.setZero();
  wm_1_.setZero();
  vm_1_.setZero();
}

//Coning & sculling compensation.
//
void IMU::cnscl(std::vector<Eigen::Vector3d> wm, std::vector<Eigen::Vector3d> vm, int nSamples)
{
  int i = 0;
  if (nSamples < 1)
  {
    return;
  }
  static double conefactors[5][4] = {				            // coning coefficients
    {0.0},                                              //1
    { 2. / 3 },										                      //2
    { 9. / 20, 27. / 20 },							                //3
    { 54. / 105, 92. / 105, 214. / 105 },				        //4
    { 250. / 504, 525. / 504, 650. / 504, 1375. / 504 }	//5
  };
  double *pcf = conefactors[nSamples - 1];
  Eigen::Vector3d cm(0.0, 0.0, 0.0), sm(0.0, 0.0, 0.0), wmm(0.0, 0.0, 0.0), vmm(0.0, 0.0, 0.0);

  nSamples_ = nSamples;
  if (nSamples == 1)  // one-plus-previous sample
  {
    if (prefirst_ == 1) 
    { 
      wm_1_ = wm[0]; 
      vm_1_ = vm[0]; 
      prefirst_ = 0; 
    }
    cm = 1.0 / 12 * wm_1_.cross(wm[0]); wm_1_ = wm[0];
    sm = 1.0 / 12 * (wm_1_.cross(wm[0])+vm_1_.cross(vm[0])); vm_1_ = vm[0];
  }
  if (nSamples > 1) prefirst_ = 1;
  for (i = 0; i < nSamples - 1; i++)
  {
    cm = cm + pcf[i] * wm[i];
    sm = sm + pcf[i] * vm[i];
    wmm = wmm + wm[i];
    vmm = vmm + vm[i];
  }
  wmm = wmm + wm[i];
  vmm = vmm + vm[i];
  phim_ = wmm + cm.cross(wm[i]);
  dvbm_ = vmm + 1.0 / 2 * wmm.cross(vmm) + (cm.cross(vm[i]) + sm.cross(wm[i]));
}
Eigen::Vector3d IMU::Getphim() const
{
  return phim_;
}
void IMU::Setphim(const Eigen::Vector3d &phim)
{
  phim_ = phim;
}
Eigen::Vector3d IMU::Getdvbm() const
{
  return dvbm_;
}
void IMU::Setdvbm(const Eigen::Vector3d &dvbm)
{
  dvbm_ = dvbm;
}
Eigen::Vector3d IMU::Getwm_1() const
{
  return wm_1_;
}
void IMU::Setwm_1(const Eigen::Vector3d &wm)
{
  wm_1_ = wm;
}
Eigen::Vector3d IMU::Getvm_1() const
{
  return vm_1_;
}
void IMU::Setvm_1(const Eigen::Vector3d &vm)
{
  vm_1_ = vm;
}

Eigen::Matrix3d IMU::GetGyrMeasCov()
{
  return gyrmeascov_;
}
void IMU::SetGyrMeasCov(const Eigen::Matrix3d &gyrmeascov)
{
  gyrmeascov_ = gyrmeascov;
}
Eigen::Matrix3d IMU::GetAccMeasCov()
{
  return accmeascov_;
}
void IMU::SetAccMeasCov(const Eigen::Matrix3d &accmeascov)
{
  accmeascov_ = accmeascov;
}
Eigen::Matrix3d IMU::GetGyrBiasRWCov()
{
  return gyrbiasRWcov_;
}
void IMU::SetGyrBiasRWCov(const Eigen::Matrix3d &gyrbiasRWcov)
{
  gyrbiasRWcov_ = gyrbiasRWcov;
}
Eigen::Matrix3d IMU::GetAccBiasRWCov()
{
  return accbiasRWcov_;
}
void IMU::SetAccBiasRWCov(const Eigen::Matrix3d &accbiasRWcov)
{
  accbiasRWcov_ = accbiasRWcov;
}
double IMU::GetGyrBiasRW2()
{
  return gyrbiasRw2_;
}
void IMU::SetGyrBiasRW2(const double &gyrbiasRw2)
{
  gyrbiasRw2_ = gyrbiasRw2;
}
double IMU::GetAccBiasRW2()
{
  return accbiasRw2_;
}
void IMU::SetAccBiasRW2(const double &accbiasRw2)
{
  accbiasRw2_ = accbiasRw2;
}
}