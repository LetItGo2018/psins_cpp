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

extern Eigen::Vector3d q2rv(const Eigen::Quaterniond &q)
{ 
  double quasign = 1.0;
  if (q.coeffs()(3)<0.0)
  {
    quasign = -1;
  }
  double n2 = acos(quasign*q.coeffs()(3)), k = 0.0;
  if (n2 > 1e-40)
  {
    k = 2 * n2 / sin(n2);
  }   
  else
  {
    k = 2;
  }  
  Eigen::Vector3d rv(q.coeffs()(0), q.coeffs()(1), q.coeffs()(2));
  return quasign*k*rv;
}

////Convert transformation matrix to att.
//
extern Eigen::Vector3d cnb2att(const Eigen::Matrix3d &Cnb)
{
  Eigen::Vector3d ret;
  ret(0) = asinEx(Cnb(2,1));
  ret(1) = atan2Ex(-Cnb(2,0), Cnb(2,2));
  ret(2) = atan2Ex(-Cnb(0,1), Cnb(1,1));
  //Eigen::Quaterniond qua(Cnb);
  //ret = q2rv(qua);
  return ret;
}

////Convert att to transformation matrix.
//
extern Eigen::Matrix3d att2cnb(const Eigen::Vector3d &att)
{
  Eigen::Matrix3d Cnb;
  double si = sin(att(0)), sj = sin(att(1)), sk = sin(att(2));
  double ci = cos(att(0)), cj = cos(att(1)), ck = cos(att(2));
  Cnb <<cj*ck - si*sj*sk, -ci*sk , sj*ck + si*cj*sk,
        cj*sk + si*sj*ck,  ci*ck , sj*sk - si*cj*ck,
       -ci*sj           ,  si    , ci*cj;
  return Cnb;
}

extern Eigen::Quaterniond cnb2qua(const Eigen::Matrix3d &Cnb)
{
  double w = 0.0, x = 0.0, y = 0.0, z = 0.0;
  w = sqrt(abs(1.0 + Cnb(0, 0) + Cnb(1, 1) + Cnb(2, 2))) / 2.0;
  x = sign(Cnb(2, 1) - Cnb(1, 2)) * sqrt(abs(1.0 + Cnb(0, 0) - Cnb(1, 1) - Cnb(2, 2))) / 2.0;
  y = sign(Cnb(0, 2) - Cnb(2, 0)) * sqrt(abs(1.0 - Cnb(0, 0) + Cnb(1, 1) - Cnb(2, 2))) / 2.0;
  z = sign(Cnb(1, 0) - Cnb(0, 1)) * sqrt(abs(1.0 - Cnb(0, 0) - Cnb(1, 1) + Cnb(2, 2))) / 2.0;
  return Eigen::Quaterniond(w, x, y, z);
}

////Convert transformation matrix to rotation vector.
//
extern Eigen::Vector3d mat2rv(const Eigen::Matrix3d &Cnb)
{
  Eigen::Vector3d ret;
  Eigen::Quaterniond qua(Cnb);
  ret = q2rv(qua);
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
extern Eigen::Vector3d qmulv(const Eigen::Quaterniond &q, const Eigen::Vector3d &v)
{
  //return q._transformVector(v)*q.conjugate();
  return Eigen::Vector3d::Zero();
}

extern Eigen::Matrix4d rq2m(const Eigen::Quaterniond &q)
{
  Eigen::Matrix4d ret;
  Eigen::Vector4d qcoeff=q.coeffs();
  ret << qcoeff(3), -qcoeff(0), -qcoeff(1), -qcoeff(2),
         qcoeff(0), qcoeff(3), qcoeff(2), -qcoeff(1),
         qcoeff(1), -qcoeff(2), qcoeff(3), qcoeff(0),
         qcoeff(2), qcoeff(1), -qcoeff(0), qcoeff(3);
  return ret;
}
extern Eigen::Matrix4d lq2m(const Eigen::Quaterniond &q)
{
  Eigen::Matrix4d ret;
  Eigen::Vector4d qcoeff = q.coeffs();
  ret << qcoeff(3), -qcoeff(0), -qcoeff(1), -qcoeff(2),
         qcoeff(0), qcoeff(3), -qcoeff(2), qcoeff(1),
         qcoeff(1), qcoeff(2), qcoeff(3), -qcoeff(0),
         qcoeff(2), -qcoeff(1), qcoeff(0), qcoeff(3);
  return ret;
}

extern Eigen::Matrix3d p2cne(const Eigen::Vector3d &pos)
{
  double slat = sin(pos(0)), clat = cos(pos(0)),
         slon = sin(pos(1)), clon = cos(pos(1));
  Eigen::Matrix3d ret;
  ret <<     -slon,       clon,    0,
        -slat*clon, -slat*slon, clat,
         clat*clon,  clat*slon, slat;
  return ret;
}
}