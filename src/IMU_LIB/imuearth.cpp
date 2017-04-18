//the Earth related parameters.
//Reference:
//       [1]严恭敏, 翁浚. 捷联惯导算法与组合导航原理讲义
//       [2]严恭敏. PSINS Toolbox for Matlab(160731)
//Author:zqlee@whu.edu.cn
//DATE  :2017/04/10

#include "IMU_LIB/imuearth.h"

#include <math.h>
//#include <Eigen/Core>

#include "IMU_LIB/imucommon.h"
namespace IMU_LIB
{
IMUEarthPara::IMUEarthPara() :
    lti_(0.0), hgt_(0.0), sl_(0.0), sl2_(0.0), sl4_(0.0),
    cl_(1.0), tl_(0.0), RMh_(6371137.0), RNh_(6371137.0), clRNh_(cl_*RNh_),
    f_RMh_(0.0), f_RNh_(0.0), f_clRNh_(0.0)
{
  const IMUCommonStruct imucommonparameters;
  a_ = imucommonparameters.Re_;
  f_ = imucommonparameters.f_;
  wie_ = imucommonparameters.wie_;
  b_ = (1 - f_)*a_;
  e_ = sqrt(a_*a_ - b_*b_) / a_;
  e2_ = e_*e_;
  ep_ = sqrt(e2_ / (1 - e2_));
  ep2_ = ep_*ep_;
  gn_ = Eigen::Vector3d(0, 0, -imucommonparameters.g0_);
  wnie_.setZero();
  wnen_.setZero();
  wnin_.setZero();
  gcc_.setZero();
}
IMUEarthPara::IMUEarthPara(const Eigen::Vector3d &pos) :
    lti_(pos(0)), hgt_(pos(2)), f_RMh_(0.0), f_RNh_(0.0), f_clRNh_(0.0)
{
  const IMUCommonStruct imucommonparameters;
  a_ = imucommonparameters.Re_;
  f_ = imucommonparameters.f_;
  wie_ = imucommonparameters.wie_;
  b_ = (1 - f_)*a_;
  e_ = sqrt(a_*a_ - b_*b_) / a_;
  e2_ = e_*e_;
  ep_ = sqrt(e2_ / (1 - e2_));
  ep2_ = ep_*ep_;
  sl_ = sin(pos(0));
  sl2_ = sl_*sl_;
  sl4_ = sl2_*sl2_;  
  cl_ = cos(pos(0));
  tl_ = cl_ == 0.0 ? 1.0e15 : sl_ / cl_;
  double sq = 1 - imucommonparameters.e2_*sl2_;
  double sq2 = sqrt(sq);
  RMh_ = imucommonparameters.Re_*(1 - imucommonparameters.e2_) / sq / sq2 + pos(2);
  RNh_ = imucommonparameters.Re_ / sq2+pos(2);
  clRNh_ = cl_*RNh_;
  gn_ = Eigen::Vector3d(0, 0, -imucommonparameters.g0_);
  wnie_.setZero();
  wnen_.setZero();
  wnin_.setZero();
  gcc_.setZero();
}

IMUEarthPara::IMUEarthPara(double a0, double f0, double g0)
  : a_(a0), f_(f0), lti_(0.0), hgt_(0.0), sl_(0.0), sl2_(0.0), sl4_(0.0),
  cl_(1.0), tl_(0.0), RMh_(6371137.0), RNh_(6371137.0), clRNh_(cl_*RNh_),
  f_RMh_(0.0), f_RNh_(0.0), f_clRNh_(0.0)
{
  const IMUCommonStruct imucommonparameters;
  wie_ = imucommonparameters.wie_;
  b_ = (1 - f_)*a_;
  e_ = sqrt(a_*a_ - b_*b_) / a_;
  e2_ = e_*e_;
  ep_ = sqrt(e2_ / (1 - e2_));
  ep2_ = ep_*ep_;
  gn_ = Eigen::Vector3d(0, 0, -g0);
  wnie_.setZero();
  wnen_.setZero();
  wnin_.setZero();
  gcc_.setZero();
}

//Update the Earth related parameters
//pos     I     const Eigen::Vector3d, (lat, lon, h) 
//vn      I     const Eigen::Vector3d
//
void IMUEarthPara::Update(const Eigen::Vector3d &pos, const Eigen::Vector3d &vn)
{
  const IMUCommonStruct imucommonparameters;
  sl_ = sin(pos(0));
  cl_ = cos(pos(0));
  tl_ = sl_ / cl_;
  double sq = 1 - e2_*sl_*sl_;
  double sq2 = sqrt(sq);
  RMh_ = a_*(1 - e2_) / sq / sq2 + pos(2);
  f_RMh_ = 1.0 / RMh_;
  RNh_ = a_ / sq2 + pos(2);
  clRNh_ = cl_*RNh_;
  f_RNh_ = 1.0 / RNh_;
  f_clRNh_ = 1.0 / clRNh_;
  wnie_(0) = 0, wnie_(1) = wie_*cl_, wnie_(2) = wie_*sl_;
  wnen_(0) = -vn(1) * f_RMh_, wnen_(1) = vn(0) * f_RNh_, wnen_(2) = wnen_(1) * tl_;
  wnin_ = wnie_ + wnen_;
  sl2_ = sl_*sl_, sl4_ = sl2_*sl2_;
  gn_(2) = -(imucommonparameters.g0_*(1 + 5.27094e-3*sl2_ + 2.32718e-5*sl4_) - 3.086e-6*pos(2));
  gcc_ = gn_ - (wnie_ + wnin_).cross(vn);
}

Eigen::Vector3d IMUEarthPara::vn2dpos(const Eigen::Vector3d &vn, double ts)
{
  Eigen::Vector3d ret(vn[1] * f_RMh_, vn[0] * f_clRNh_, vn[2]);
  ret *= ts;
  return ret;
}
}