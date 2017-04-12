//the IMU common struct
//Reference:
//       [1]严恭敏, 翁浚. 捷联惯导算法与组合导航原理讲义
//       [2]严恭敏. PSINS Toolbox for Matlab(160731)
//Author:zqlee@whu.edu.cn
//DATE  :2017/04/10

#include "IMU_LIB/imucommon.h"

#include <math.h>

namespace IMU_LIB
{
IMUCommonStruct::IMUCommonStruct()
  :Re_(6378137.0), f_(1.0 / 298.257), wie_(7.2921151467e-5), g0_(9.7803267714)
{  
  e_ = sqrt(2 * f_ - f_*f_);
  e2_ = e_*e_;
  mg_ = 1.0e-3 * g0_;
  ug_ = 1.0e-6*g0_;
  deg_ = PI / 180.0;
  min_ = deg_ / 60.0;
  sec_ = min_ / 60.0;
  ppm_ = 1.0e-6;
  hur_ = 3600.0;
  dph_ = deg_ / hur_;
  dpsh_ = deg_ / sqrt(hur_);
  dphpsh_ = dph_ / sqrt(hur_);
  ugpsHz_ = ug_ / sqrt(1.0);
  ugpsh_ = ug_ / sqrt(hur_);
  mpsh_ = 1 / sqrt(hur_);
  mpspsh_ = 1 / 1 / sqrt(hur_);
  ppmpsh_ = ppm_ / sqrt(hur_);
  secpsh_ = sec_ / sqrt(hur_);
}

////Convert 3x1 vector to 3x3 askew matrix.
//
extern Eigen::Matrix3d askew(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d Omega;
  Omega << 0, -v(2), v(1)
    , v(2), 0, -v(0)
    , -v(1), v(0), 0;
  return Omega;
}

////Convert rotation vector to transformation quaternion.
//
extern Eigen::Quaterniond rv2q(const Eigen::Vector3d &v)
{
#define F1	(   2 * 1)		// define: Fk=2^k*k! 
#define F2	(F1*2 * 2)
#define F3	(F2*2 * 3)
#define F4	(F3*2 * 4)
#define F5	(F3*2 * 5)
  double n2 = v(0)*v(0) + v(1)*v(1) + v(2)*v(2), c, f;
  if (n2 < (PI / 180.0*PI / 180.0))	// 0.017^2 
  {
    double n4 = n2*n2;
    c = 1.0 - n2*(1.0 / F2) + n4*(1.0 / F4);
    f = 0.5 - n2*(1.0 / F3) + n4*(1.0 / F5);
  }
  else
  {
    double n_2 = sqrt(n2) / 2.0;
    c = cos(n_2);
    f = sin(n_2) / n_2*0.5;
  }
  return Eigen::Quaterniond(c, f*v(0), f*v(1), f*v(2));
}

////Convert transformation matrix to rotation vector.
//
extern Eigen::Vector3d mat2rv(const Eigen::Matrix3d &Cnb)
{
  Eigen::Vector3d ret;
  ret(0) = asinEx(Cnb(2,1));
  ret(1) = atan2Ex(-Cnb(2,0), Cnb(2,2));
  ret(2) = atan2Ex(-Cnb(0,1), Cnb(1,1));
  return ret;
}

// determine the sign of 'val' with the sensitivity of 'eps'
int signE(double val, double eps)
{
  int s;

  if (val<-eps)
  {
    s = -1;
  }
  else if (val>eps)
  {
    s = 1;
  }
  else
  {
    s = 0;
  }
  return s;
}
// set double value 'val' between range 'minVal' and 'maxVal'
double range(double val, double minVal, double maxVal)
{
  double res;

  if (val<minVal)
  {
    res = minVal;
  }
  else if (val>maxVal)
  {
    res = maxVal;
  }
  else
  {
    res = val;
  }
  return res;
}
double atan2Ex(double y, double x)
{
  double res;

  if ((sign(y) == 0) && (sign(x) == 0))
  {
    res = 0.0;
  }
  else
  {
    res = atan2(y, x);
  }
  return res;
}
}