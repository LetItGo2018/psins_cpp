//the IMU class
//Reference:
//       [1]严恭敏, 翁浚. 捷联惯导算法与组合导航原理讲义
//       [2]严恭敏. PSINS Toolbox for Matlab(160731)
//Author:zqlee@whu.edu.cn
//DATE  :2017/04/10

#ifndef IMU_IMU_H
#define IMU_IMU_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace IMU_LIB
{
class IMU
{
public:
  IMU();
  void cnscl(std::vector<Eigen::Vector3d> wm, std::vector<Eigen::Vector3d> vm, int nSamples);

  Eigen::Vector3d Getphim() const;
  void Setphim(const Eigen::Vector3d &);
  Eigen::Vector3d Getdvbm() const;
  void Setdvbm(const Eigen::Vector3d &);
  Eigen::Vector3d Getwm_1() const;
  void Setwm_1(const Eigen::Vector3d &);
  Eigen::Vector3d Getvm_1() const;
  void Setvm_1(const Eigen::Vector3d &);
  static Eigen::Matrix3d GetGyrMeasCov();
  static void SetGyrMeasCov(const Eigen::Matrix3d &);
  static Eigen::Matrix3d GetAccMeasCov();
  static void SetAccMeasCov(const Eigen::Matrix3d &);
  static Eigen::Matrix3d GetGyrBiasRWCov();
  static void SetGyrBiasRWCov(const Eigen::Matrix3d &);
  static Eigen::Matrix3d GetAccBiasRWCov();
  static void SetAccBiasRWCov(const Eigen::Matrix3d &);
  static double GetGyrBiasRW2();
  static void SetGyrBiasRW2(const double &);
  static double GetAccBiasRW2(); 
  static void SetAccBiasRW2(const double &);

private:
  int nSamples_;
  int prefirst_;
  Eigen::Vector3d phim_;
  Eigen::Vector3d dvbm_;
  Eigen::Vector3d wm_1_; 
  Eigen::Vector3d vm_1_;

  // covariance of measurement
  static Eigen::Matrix3d gyrmeascov_;
  static Eigen::Matrix3d accmeascov_;

  // covariance of bias random walk
  static Eigen::Matrix3d gyrbiasRWcov_;
  static Eigen::Matrix3d accbiasRWcov_;

  static double gyrbiasRw2_;
  static double accbiasRw2_;
};
}
#endif//IMU_IMU_H