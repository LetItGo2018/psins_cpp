#include "IMU_LIB/psins.h"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigen>

std::vector<std::string> split(std::string str, std::string pattern);

int main(int agrc, char **argv)
{
  Eigen::Vector3d att0(0.0, 0.0,0.0);
  Eigen::Vector3d vn0(0.0, 0.0, 0.0);
  Eigen::Vector3d pos0(0.597706293396019, 1.900832224040738, 380.0);
  IMU_LIB::PSINS psinsfortest(att0, vn0, pos0);

  std::string imufile = "../../data/input/imu.dat";
  std::string posfile = "../../data/output/pos.txt";
  std::ifstream fpimu(imufile);
  std::ofstream fppos(posfile);
  if (!fpimu)
  {
    std::cout << "open file error!" << std::endl;
    return -1;
  }
  std::string line;
  std::vector<Eigen::Vector3d> wmm, vmm;
  while (!fpimu.eof())
  {
    std::vector<std::string> imudata;
    std::getline(fpimu,line);
    imudata = split(line, ",");
    if (imudata.size() != 7)
    {
      break;
    }
    Eigen::Vector3d wm(std::atof(imudata[1].c_str()),
                       std::atof(imudata[2].c_str()),
                       std::atof(imudata[3].c_str()));
    Eigen::Vector3d vm(std::atof(imudata[4].c_str()),
                       std::atof(imudata[5].c_str()),
                       std::atof(imudata[6].c_str()));
    wmm.push_back(wm);
    vmm.push_back(vm);
    if (wmm.size() == 4)
    {
      psinsfortest.Update(wmm, vmm, 4, 0.01);
      char tmp[1024];
      sprintf(tmp, "%.10f %.10f %.10f", psinsfortest.pos_(0), psinsfortest.pos_(1), psinsfortest.pos_(2));
      fppos << tmp << std::endl;
      std::cout << psinsfortest.pos_(0) << " " << psinsfortest.pos_(1) << " " << psinsfortest.pos_(2) << std::endl;
      wmm.clear();
      vmm.clear();
    }
  }
  fppos.close();
  fpimu.close();
  int a = 0;
  return 0;
}

std::vector<std::string> split(std::string str, std::string pattern)
{
  std::string::size_type pos;
  std::vector<std::string> result;
  str += pattern;//扩展字符串以方便操作
  int size = str.size();

  for (int i = 0; i < size; i++)
  {
    pos = str.find(pattern, i);
    if (pos < size)
    {
      std::string s = str.substr(i, pos - i);
      result.push_back(s);
      i = pos + pattern.size() - 1;
    }
  }
  return result;
}