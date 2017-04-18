//the KalmanFilter class
//Reference:
//       [1]严恭敏, 翁浚. 捷联惯导算法与组合导航原理讲义
//       [2]严恭敏. PSINS Toolbox for Matlab(160731)
//Author:zqlee@whu.edu.cn
//DATE  :2017/04/15

#ifndef IMU_KALMANFILTER_H
#define IMU_KALMANFILTER_H

#include <stdarg.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include "IMU_LIB/psins.h"

namespace IMU_LIB
{
class KalmanFilterBase
{
public:
  KalmanFilterBase();
  KalmanFilterBase(int q0, int r0);
  void SetPk(double f, ...);
  void SetQt(double f, ...);
  void SetRk(double f, ...);
  void SetZk(double f, ...);
  void SetFt(){};
  void SetHk(){};
  void TimeUpdate(double ts);
  void MeasUpdate(double fading = 1.0);

  int q_;
  int r_;
  Eigen::VectorXd Xk_;
  Eigen::VectorXd Zk_;
  Eigen::MatrixXd Ft_;
  Eigen::MatrixXd Pk_;
  Eigen::MatrixXd Qt_;
  Eigen::MatrixXd Hk_;
  Eigen::MatrixXd Rk_;
};
class KalmanFilter:public KalmanFilterBase
{
public:
  KalmanFilter() :KalmanFilterBase(){}
  KalmanFilter(int q0, int r0) :KalmanFilterBase(q0, r0){}
  void SetFt(PSINS &sins);
  void SetHk(void);
};

class KalmanFilterForAlignfn :public KalmanFilterBase
{
public:
  KalmanFilterForAlignfn() :KalmanFilterBase(5,2){}
  //KalmanFilterForAlignfn(int q0, int r0) :KalmanFilterBase(q0, r0){}
  void SetFt(Eigen::Vector3d &pos);
  void SetHk(Eigen::Vector3d &pos);
};

}

#endif//IMU_KALMANFILTER_H