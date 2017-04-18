//the Earth related parameters.
//Reference:
//       [1]严恭敏, 翁浚. 捷联惯导算法与组合导航原理讲义
//       [2]严恭敏. PSINS Toolbox for Matlab(160731)
//Author:zqlee@whu.edu.cn
//DATE  :2017/04/10

#ifndef IMU_EARTH_H_
#define IMU_EARTH_H_

#include <Eigen/Core>
#include <Eigen/Dense>

#include "IMU_LIB/imucommon.h"

namespace IMU_LIB
{
class IMUEarthPara
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  IMUEarthPara();
  IMUEarthPara(double a0, double f0, double g0);
  IMUEarthPara(const Eigen::Vector3d &pos);

  //Update the Earth related parameters
  void Update(const Eigen::Vector3d &pos, const Eigen::Vector3d &vn);
  //ins.Mpvvn*nts;
  Eigen::Vector3d vn2dpos(const Eigen::Vector3d &vn, double ts = 1.0);

  double a_;                      //long semi-major axis
  double b_;                      //short semi-major axis
  double f_;                      //flattening
  double e_;                      //1st eccentricity
  double e2_;                     //square of e
  double ep_;                     //2nd eccentricity
  double ep2_;                    //square of ep
  double wie_;                    //the Earth's angular rate
  double lti_;
  double hgt_;
  double sl_;                     //sin(lat)
  double sl2_;                    //sl*sl
  double sl4_;                    //sl2*sl2
  double cl_;                     //cons(lat)
  double tl_;                     //sl/cl
  double RMh_;
  double RNh_;
  double clRNh_;                  //cl*RNh
  double f_RMh_;
  double f_RNh_;
  double f_clRNh_;
  Eigen::Vector3d wnie_;           //4.1-3
  Eigen::Vector3d wnen_;           //4.1-4
  Eigen::Vector3d wnin_;           //wnie+wnen
  Eigen::Vector3d gn_;             //
  Eigen::Vector3d gcc_;            //Gravitational/Coriolis/Centripetal acceleration

};
}
#endif//IMU_EARTH_H_