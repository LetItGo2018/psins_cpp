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
private:
  int nSamples_;
  int prefirst_;
  Eigen::Vector3d phim_;
  Eigen::Vector3d dvbm_;
  Eigen::Vector3d wm_1_; 
  Eigen::Vector3d vm_1_;
};
}
#endif//IMU_IMU_H