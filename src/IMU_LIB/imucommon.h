//the IMU common struct
//Reference:
//       [1]严恭敏, 翁浚. 捷联惯导算法与组合导航原理讲义
//       [2]严恭敏. PSINS Toolbox for Matlab(160731)
//Author:zqlee@whu.edu.cn
//DATE  :2017/04/10

#ifndef IMU_IMUCOMMON_H
#define IMU_IMUCOMMON_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace IMU_LIB
{
const double PI = 3.14159265358979;
#define EPS		2.22044604925031e-16
#define INF		1.0e100

int signE(double val, double eps);
double range(double val, double minVal, double maxVal);
double atan2Ex(double y, double x);
double fastvxv(double *a, double *b, int n);
#define sign(val)		signE(val, EPS)
#define asinEx(x)		asin(range(x, -1.0, 1.0))
#define acosEx(x)		acos(range(x, -1.0, 1.0))

////IMU common parameter struct
//
struct IMUCommonStruct
{
  IMUCommonStruct();
  double Re_;               //the Earth's semi-major axis
  double f_;                //flattening
  double g0_;               //gravitational force
  double wie_;              //the Earth's angular rate
  double e_;                //1st eccentricity
  double e2_;               //square of e_
  double mg_;               //milli g
  double ug_;               //micro g
  double deg_;              //arcdeg
  double min_;              //arcmin
  double sec_;              //arcsec
  double hur_;              //time hour (1hur=3600second)
  double ppm_;              //parts per million
  double ppmpsh_;           //ppm / sqrt(hour)
  double dph_;              //arcdeg / hour
  double dpsh_;             //arcdeg / sqrt(hour)
  double dphpsh_;           //(arcdec/hour) / sqrt(hour)
  double ugpsh_;            // ug / sqrt(hour)
  double ugpsHz_;           //ug / sqrt(Hz)
  double mpsh_;             //m / sqrt(hour)
  double mpspsh_;           //(m/s) / sqrt(hour), 1*mpspsh=1667*ugpsHz
  double secpsh_;           //arcsec / sqrt(hour)
};

////Convert 3x1 vector to 3x3 askew matrix.
//
extern Eigen::Matrix3d askew(const Eigen::Vector3d &);

////Convert rotation vector to transformation quaternion.
//
extern Eigen::Quaterniond rv2q(const Eigen::Vector3d &);

////Convert transformation matrix to rotation vector.
//
extern Eigen::Vector3d mat2rv(const Eigen::Matrix3d &);
}
#endif//IMU_IMUCOMMON_H