//the KalmanFilter class
//Reference:
//       [1]严恭敏, 翁浚. 捷联惯导算法与组合导航原理讲义
//       [2]严恭敏. PSINS Toolbox for Matlab(160731)
//Author:zqlee@whu.edu.cn
//DATE  :2017/04/15

#include "IMU_LIB/kalmanfilter.h"

namespace IMU_LIB
{
KalmanFilter::KalmanFilter()
{
  q_ = 15;
  r_ = 6;
  Xk_.resize(q_);
  Zk_.resize(r_);
  Ft_.resize(q_, q_);
  Pk_.resize(q_, q_);
  Qt_.resize(q_, q_);
  Hk_.resize(r_, q_);
  Rk_.resize(r_, r_);
  Xk_.setZero();
  Zk_.setZero();
  Ft_.setZero();
  Pk_.setZero();
  Qt_.setZero();
  Hk_.setZero();
  Rk_.setZero();
}
KalmanFilter::KalmanFilter(int q0, int r0)
{
  q_ = q0;
  r_ = r0;
  Xk_.resize(q_);
  Zk_.resize(r_);
  Ft_.resize(q_, q_);
  Pk_.resize(q_, q_);
  Qt_.resize(q_, q_);
  Hk_.resize(r_, q_);
  Rk_.resize(r_, r_);
  Xk_.setZero();
  Zk_.setZero();
  Ft_.setZero();
  Pk_.setZero();
  Qt_.setZero();
  Hk_.setZero();
  Rk_.setZero();
}
void KalmanFilter::TimeUpdate(double ts)
{
  Eigen::MatrixXd Fk = Ft_*ts;
  for (int i = 0; i < Fk.cols(); ++i)
  {
    Fk(i, i) += 1.0;
  }
  Xk_ = Fk * Xk_;
  Pk_ = Fk*Pk_*Fk.transpose() + Qt_*ts;
}
void KalmanFilter::MeasUpdate(double fading)
{
  Eigen::MatrixXd Pxykk_1 = Pk_*Hk_.transpose();
  Eigen::MatrixXd Py0 = Hk_*Pxykk_1;
  Eigen::VectorXd ykk_1 = Hk_*Xk_;
  Eigen::VectorXd Lk = Zk_ - ykk_1;
  Eigen::MatrixXd Pykk_1 = Py0 + Rk_;
  Eigen::MatrixXd Kk = Pxykk_1*Pykk_1.inverse();
  Xk_ += Kk*Lk;
  Pk_ -= Kk*Pykk_1*Kk.transpose();
  if (fading>1.0) Pk_ = Pk_*fading;
  /*Eigen::VectorXd Pxz, Kk, Hi;
  for (int i = 0; i<r_; i++)
  {
    Hi = Hk_.row(i);
    Pxz = Pk_*Hi.transpose();
    double Pzz = (Hi*Pxz)(0) + Rk_(i, i);
    Kk = Pxz*(1.0 / Pzz);
    Xk_ = Xk_ + Kk*(Zk_(i) - (Hi*Xk_)(0));
    Pk_ = Pk_ - Kk*Pxz.transpose();
  }
  if (fading>1.0) Pk_ = Pk_*fading;*/
}
void KalmanFilter::SetPk(double f, ...)
{
  va_list vl;
  va_start(vl, f);
  for (int i = 0; i < q_; i++)
  {
    Pk_(i, i) = f*f;  f = va_arg(vl, double);
  }
  va_end(vl);
}
void KalmanFilter::SetQt(double f, ...)
{
  va_list vl;
  va_start(vl, f);
  for (int i = 0; i < q_; i++)
  {
    Qt_(i,i) = f*f;  f = va_arg(vl, double);
  }
  va_end(vl);
}
void KalmanFilter::SetRk(double f, ...)
{
  va_list vl;
  va_start(vl, f);
  for (int i = 0; i < r_; i++)
  {
    Rk_(i,i) = f*f;  f = va_arg(vl, double);
  }
  va_end(vl);
}
void KalmanFilter::SetZk(double f, ...)
{
  va_list vl;
  va_start(vl, f);
  for (int i = 0; i < r_; i++)
  {
    Zk_(i) = f;  f = va_arg(vl, double);
  }
  va_end(vl);
}
void KalmanFilter::SetFt(PSINS &sins)
{
  Eigen::Matrix3d Maa, Mav, Map, Mva, Mvv, Mvp, Mpv, Mpp, Cnb;
  sins.etm(Maa, Mav, Map, Mva, Mvv, Mvp, Mpv, Mpp);
  Cnb = sins.Cnb_;
  //	Ft = [ Maa    Mav    Map    -ins.Cnb  O33 
  //         Mva    Mvv    Mvp     O33      ins.Cnb 
  //         O33    Mpv    Mpp     O33      O33
  //         zeros(6,9)  diag(-1./[ins.tauG;ins.tauA]) ];
  // phi


  Ft_.block(0, 0, 3, 3) = Maa;
  Ft_.block(0, 3, 3, 3) = Mav;
  Ft_.block(0, 6, 3, 3) = Map;
  Ft_.block(0, 9, 3, 3) = -sins.Cnb_;
  Ft_.block(0, 12, 3, 3) = Eigen::Matrix3d::Zero();

  Ft_.block(3, 0, 3, 3) = Mva;
  Ft_.block(3, 3, 3, 3) = Mvv;
  Ft_.block(3, 6, 3, 3) = Mvp;
  Ft_.block(3, 9, 3, 3) = Eigen::Matrix3d::Zero();
  Ft_.block(3, 12, 3, 3) = sins.Cnb_;

  Ft_.block(6, 0, 3, 3) = Eigen::Matrix3d::Zero();
  Ft_.block(6, 3, 3, 3) = Mpv;
  Ft_.block(6, 6, 3, 3) = Mpp;
  Ft_.block(6, 9, 3, 3) = Eigen::Matrix3d::Zero();
  Ft_.block(6, 12, 3, 3) = Eigen::Matrix3d::Zero();

  Ft_.block(9, 0, 6, 15) = Eigen::Matrix<double,6,15>::Zero();
  

}

void KalmanFilter::SetHk(void)
{
  //	Hk(0,6) = Hk(1,7) = Hk(2,8) = 1.0;
  Hk_(0, 3) = Hk_(1, 4) = Hk_(2, 5) = 1.0; Hk_(3, 6) = Hk_(4, 7) = Hk_(5, 8) = 1.0;
}
}