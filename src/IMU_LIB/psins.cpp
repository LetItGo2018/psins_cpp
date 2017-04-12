//the PSINS class
//Reference:
//       [1]严恭敏, 翁浚. 捷联惯导算法与组合导航原理讲义
//       [2]严恭敏. PSINS Toolbox for Matlab(160731)
//Author:zqlee@whu.edu.cn
//DATE  :2017/04/11

#include "IMU_LIB/psins.h"

namespace IMU_LIB
{
PSINS::PSINS() :nts_(0.0), eth_(), imu_()
{
  //eth_ = IMUEarthPara();
  //imu_ = IMU();
  qnb_.setIdentity();
  Cnb_.setIdentity();
  Cnb0_.setIdentity();
  Kg_.setIdentity();
  Ka_.setIdentity();
  wib_.setZero();
  fb_.setZero();
  fn_.setZero();
  an_.setZero();
  web_.setZero();
  wnb_.setZero();
  att_.setZero();
  vn_.setZero();
  pos_.setZero();
  eb_.setZero();
  db_.setZero();
}
PSINS::PSINS(Eigen::Quaterniond &qnb0, Eigen::Vector3d &vn0, Eigen::Vector3d &pos0) :
    eth_(), imu_(), nts_(0.0), 
    qnb_(qnb0), vn_(vn0), pos_(pos0)
{
  //eth_ = IMUEarthPara();
  eth_.Update(pos0, vn0);
  //imu_ = IMU();
  Cnb_=qnb_.matrix();
  Cnb0_=Cnb_;
  Kg_.setIdentity();
  Ka_.setIdentity();
  wib_.setZero();
  fb_.setZero();
  fn_.setZero();
  an_.setZero();
  web_.setZero();
  wnb_.setZero();
  att_ = mat2rv(Cnb_);
  eb_.setZero();
  db_.setZero();
}

void PSINS::Update(std::vector<Eigen::Vector3d> wm, std::vector<Eigen::Vector3d> vm, int nSamples, double ts)
{
  //Eigen::Quaterniond quatmp,quatmpanother;
  nts_ = nSamples*ts;
  double nts2 = nts_ / 2;
  imu_.cnscl(wm, vm, nSamples);
  imu_.Setphim(Kg_*imu_.Getphim() - eb_*nts_); 
  imu_.Setdvbm(Ka_*imu_.Getdvbm() - db_*nts_);  // calibration
  Eigen::Vector3d vn01 = vn_ + an_*nts2, pos01 = pos_ + eth_.vn2dpos(vn01, nts2);
  eth_.Update(pos01, vn01);
  wib_ = imu_.Getphim() / nts_; 
  fb_  = imu_.Getdvbm() / nts_;
  web_ = wib_ - Cnb_.transpose()*eth_.wnie_;
  //quatmp = imu_.Getphim() / 2;                  //!!!
  wnb_ = wib_ - (qnb_*rv2q(imu_.Getphim() / 2)).matrix().transpose()*eth_.wnin_;
  fn_ = qnb_*fb_;
  ////quatmp = -eth_.wnin_*nts2;
  an_ = rv2q(-eth_.wnin_*nts2)*fn_ + eth_.gcc_;
  Eigen::Vector3d vn1 = vn_ + an_*nts_;
  pos_ = pos_ + eth_.vn2dpos(vn_ + vn1, nts2);
  vn_ = vn1;
  Cnb0_ = Cnb_;
  //quatmp = -eth_.wnin_*nts_;
  //quatmpanother = imu_.Getphim();
  qnb_ = rv2q(-eth_.wnin_*nts_)*qnb_*rv2q(imu_.Getphim());
  Cnb_ = qnb_.matrix();
  att_ = mat2rv(Cnb_);
}
void PSINS::etm(Eigen::Matrix3d &Maa, Eigen::Matrix3d &Mav, Eigen::Matrix3d &Map,
  Eigen::Matrix3d &Mva, Eigen::Matrix3d &Mvv, Eigen::Matrix3d &Mvp,
  Eigen::Matrix3d &Mpv, Eigen::Matrix3d &Mpp)
{
  IMUCommonStruct imucommonpara;
  double tl = eth_.tl_, secl = 1.0 / eth_.cl_, secl2 = secl*secl,
    wN = eth_.wnie_(1), wU = eth_.wnie_(2), vE = vn_(0), vN = vn_(1);
  double f_RMh = eth_.f_RMh_, f_RNh = eth_.f_RNh_, f_clRNh = eth_.f_clRNh_,
    f_RMh2 = f_RMh*f_RMh, f_RNh2 = f_RNh*f_RNh;
  Eigen::Matrix3d Avn = askew(vn_);
  Eigen::Matrix3d Mp1; Mp1 << 0, 0, 0, -wU, 0, 0, wN, 0, 0;
  Eigen::Matrix3d Mp2; Mp2 << 0, 0, vN*f_RMh2, 0, 0, -vE*f_RNh2, vE*secl2*f_RNh, 0, -vE*tl*f_RNh2;
  Maa = askew(-eth_.wnin_);
  Mav << 0, -f_RMh, 0, f_RNh, 0, 0, tl*f_RNh, 0, 0;
  Map = Mp1 + Mp2;
  Mva = askew(fn_);
  Mvv = Avn*Mav - askew(eth_.wnie_ + eth_.wnin_);
  Mvp = Avn*(Mp1 + Map);
  double scl = eth_.sl_*eth_.cl_;
  Mvp(2, 0) = Mvp(2, 0) - imucommonpara.g0_*(5.27094e-3 * 2 * scl + 2.32718e-5 * 4 * eth_.sl2_*scl); 
  Mvp(2, 2) = Mvp(2, 2) + 3.086e-6;
  Mpv << 0, f_RMh, 0, f_clRNh, 0, 0, 0, 0, 1;
  Mpp << 0, 0, -vN*f_RMh2, vE*tl*f_clRNh, 0, -vE*secl*f_RNh2, 0, 0, 0;
}

}