//the PSINS class
//Reference:
//       [1]严恭敏, 翁浚. 捷联惯导算法与组合导航原理讲义
//       [2]严恭敏. PSINS Toolbox for Matlab(160731)
//Author:zqlee@whu.edu.cn
//DATE  :2017/04/11

#ifndef IMU_PSINS_H
#define IMU_PSINS_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "IMU_LIB/imu.h"
#include "IMU_LIB/imuearth.h"
#include "IMU_LIB/imucommon.h"

namespace IMU_LIB
{
class PSINS
{
public:
  PSINS();
  PSINS(Eigen::Quaterniond &qnb0, Eigen::Vector3d &vn0, Eigen::Vector3d &pos0);

  void Update(std::vector<Eigen::Vector3d> wm, std::vector<Eigen::Vector3d> vm, int nSamples, double ts);
  Eigen::Matrix3d AlignCoarse(Eigen::Vector3d wmm, Eigen::Vector3d vmm, double latitude);
  Eigen::Matrix3d AlignWahba(std::vector<Eigen::Vector3d> wm, std::vector<Eigen::Vector3d> vm, const Eigen::Vector3d &pos, const int &nSamples, const double &ts);
  void etm(Eigen::Matrix3d &Maa, Eigen::Matrix3d &Mav, Eigen::Matrix3d &Map,
           Eigen::Matrix3d &Mva, Eigen::Matrix3d &Mvv, Eigen::Matrix3d &Mvp, 
           Eigen::Matrix3d &Mpv, Eigen::Matrix3d &Mpp);

  double nts_;
  IMUEarthPara eth_;
  IMU imu_;
  Eigen::Quaterniond qnb_;
  Eigen::Matrix3d Cnb_;
  Eigen::Matrix3d Cnb0_;
  Eigen::Matrix3d Kg_;
  Eigen::Matrix3d Ka_;
  Eigen::Vector3d wib_;
  Eigen::Vector3d fb_;
  Eigen::Vector3d fn_;
  Eigen::Vector3d an_;
  Eigen::Vector3d web_;
  Eigen::Vector3d wnb_;
  Eigen::Vector3d att_;
  Eigen::Vector3d vn_;
  Eigen::Vector3d pos_;
  Eigen::Vector3d eb_;
  Eigen::Vector3d db_;
  Eigen::Quaterniond qib0b_;
  Eigen::Matrix4d Kforalgin_;
};
}
#endif//IMU_PSINS_H