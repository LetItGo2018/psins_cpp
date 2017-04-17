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
  qib0b_.setIdentity();
  Kforalgin_.setZero();
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
  att_ = cnb2att(Cnb_);
  eb_.setZero();
  db_.setZero();
  qib0b_.setIdentity();
  Kforalgin_.setZero();
}
PSINS::PSINS(Eigen::Vector3d &att0, Eigen::Vector3d &vn0, Eigen::Vector3d &pos0) :
    eth_(), imu_(), nts_(0.0),
    att_(att0), vn_(vn0), pos_(pos0)
{
  Cnb_ = att2cnb(att_);
  qnb_=Eigen::Quaterniond(Cnb_);
  //eth_ = IMUEarthPara();
  eth_.Update(pos0, vn0);
  //imu_ = IMU();
  
  Cnb0_ = Cnb_;
  Kg_.setIdentity();
  Ka_.setIdentity();
  wib_.setZero();
  fb_.setZero();
  fn_.setZero();
  an_.setZero();
  web_.setZero();
  wnb_.setZero();
  att_ = cnb2att(Cnb_);
  eb_.setZero();
  db_.setZero();
  qib0b_.setIdentity();
  Kforalgin_.setZero();
}
PSINS::PSINS(Eigen::Matrix3d &Cnb0, Eigen::Vector3d &vn0, Eigen::Vector3d &pos0) :
    eth_(), imu_(), nts_(0.0),
    Cnb_(Cnb0), vn_(vn0), pos_(pos0)
{
  //eth_ = IMUEarthPara();
  eth_.Update(pos0, vn0);
  //imu_ = IMU();
  //Cnb_ = qnb_.matrix();
  att_ = cnb2att(Cnb_);
  qnb_ = cnb2qua(Cnb_);
  Cnb0_ = Cnb_;
  Kg_.setIdentity();
  Ka_.setIdentity();
  wib_.setZero();
  fb_.setZero();
  fn_.setZero();
  an_.setZero();
  web_.setZero();
  wnb_.setZero();

  eb_.setZero();
  db_.setZero();
  qib0b_.setIdentity();
  Kforalgin_.setZero();
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
  att_ = cnb2att(Cnb_);
}

Eigen::Matrix3d PSINS::AlignCoarse(Eigen::Vector3d wmm, Eigen::Vector3d vmm, double latitude)
{
  double T11, T12, T13, T21, T22, T23, T31, T32, T33;
  double cl = cos(latitude), tl = tan(latitude);
  Eigen::Vector3d wbib = wmm/wmm.norm(), fb = vmm /vmm.norm();
  T31 = fb(0), T32 = fb(1), T33 = fb(2);
  T21 = wbib(0) / cl - T31*tl, T22 = wbib(1) / cl - T32*tl, T23 = wbib(2) / cl - T33*tl;   
  double nn = sqrt(T21*T21 + T22*T22 + T23*T23);
  T11 = T22*T33 - T23*T32, T12 = T23*T31 - T21*T33, T13 = T21*T32 - T22*T31;
  Eigen::Matrix3d ret;
  ret << T11, T12, T13, T21, T22, T23, T31, T32, T33;
  return ret;
}

Eigen::Matrix3d PSINS::AlignWahba(std::vector<Eigen::Vector3d> wm, std::vector<Eigen::Vector3d> vm, const Eigen::Vector3d &pos, const int &nSamples, const double &ts)
{
  IMUCommonStruct imucommonpara;
  IMUEarthPara imuethpara;
  double nts = nSamples*ts;
  imu_.cnscl(wm, vm, nSamples);
  Eigen::Vector3d vib0 = qib0b_.matrix()*imu_.Getdvbm();
  Eigen::Vector3d vi0(imuethpara.cl_*cos(nts*imucommonpara.wie_),
                      imuethpara.cl_*sin(nts*imucommonpara.wie_),
                      imuethpara.sl_);
  vi0 *= imucommonpara.g0_*nts;
  qib0b_ = qib0b_*rv2q(imu_.Getphim());
  Eigen::Quaterniond rightqua(0.0, vib0(0), vib0(1), vib0(2)), 
                     leftqua (0.0, vi0(0) , vi0(1) , vi0(2) );
  Eigen::Matrix4d dM = rq2m(rightqua) - lq2m(leftqua);
  Kforalgin_ = 0.99991*Kforalgin_ + dM.transpose()*dM*nts;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eig(Kforalgin_);
  Eigen::Matrix4d matofv = eig.eigenvectors();
  Eigen::Quaterniond qi0ib0(matofv(0, 0), matofv(1,0), matofv(2,0), matofv(3,0));
  Eigen::Vector3d vforcne(pos(0),nts*imucommonpara.wie_,0.0);
  Eigen::Matrix3d Cni0 = p2cne(vforcne);
  Eigen::Quaterniond qni0(Cni0);
  qnb_ = qni0*qi0ib0*qib0b_;
  return qnb_.matrix();
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
  Eigen::Matrix3d Avn = Sophus::SO3::hat(vn_);
  Eigen::Matrix3d Mp1; Mp1 << 0, 0, 0, -wU, 0, 0, wN, 0, 0;
  Eigen::Matrix3d Mp2; Mp2 << 0, 0, vN*f_RMh2, 0, 0, -vE*f_RNh2, vE*secl2*f_RNh, 0, -vE*tl*f_RNh2;
  Maa = Sophus::SO3::hat(-eth_.wnin_);
  Mav << 0, -f_RMh, 0, f_RNh, 0, 0, tl*f_RNh, 0, 0;
  Map = Mp1 + Mp2;
  Mva = Sophus::SO3::hat(fn_);
  Mvv = Avn*Mav - Sophus::SO3::hat(eth_.wnie_ + eth_.wnin_);
  Mvp = Avn*(Mp1 + Map);
  double scl = eth_.sl_*eth_.cl_;
  Mvp(2, 0) = Mvp(2, 0) - imucommonpara.g0_*(5.27094e-3 * 2 * scl + 2.32718e-5 * 4 * eth_.sl2_*scl); 
  Mvp(2, 2) = Mvp(2, 2) + 3.086e-6;
  Mpv << 0, f_RMh, 0, f_clRNh, 0, 0, 0, 0, 1;
  Mpp << 0, 0, -vN*f_RMh2, vE*tl*f_clRNh, 0, -vE*secl*f_RNh2, 0, 0, 0;
}

}