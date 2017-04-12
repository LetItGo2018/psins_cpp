//the IMU class
//Reference:
//       [1]严恭敏, 翁浚. 捷联惯导算法与组合导航原理讲义
//       [2]严恭敏. PSINS Toolbox for Matlab(160731)
//Author:zqlee@whu.edu.cn
//DATE  :2017/04/10

#include "IMU_LIB/imu.h"

namespace IMU_LIB
{
IMU::IMU() :nSamples_(0), prefirst_(1)
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
}