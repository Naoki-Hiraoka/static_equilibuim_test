#include <wrench_distribution_existence/wrench_distribution_existence.h>
#include <iostream>

int main(){
  wrench_distribution_existence::Args args;

  // rleg
  {
    wrench_distribution_existence::Args::EEF eef;
    eef.pose.translation() = Eigen::Vector3d(0,-0.1,0) + Eigen::Vector3d(0.12,0.05,0);
    args.eefs.push_back(eef);
  }
  {
    wrench_distribution_existence::Args::EEF eef;
    eef.pose.translation() = Eigen::Vector3d(0,-0.1,0) + Eigen::Vector3d(0.12,-0.05,0);
    args.eefs.push_back(eef);
  }
  {
    wrench_distribution_existence::Args::EEF eef;
    eef.pose.translation() = Eigen::Vector3d(0,-0.1,0) + Eigen::Vector3d(-0.09,0.05,0);
    args.eefs.push_back(eef);
  }
  {
    wrench_distribution_existence::Args::EEF eef;
    eef.pose.translation() = Eigen::Vector3d(0,-0.1,0) + Eigen::Vector3d(-0.09,-0.05,0);
    args.eefs.push_back(eef);
  }

  // lleg
  {
    wrench_distribution_existence::Args::EEF eef;
    eef.pose.translation() = Eigen::Vector3d(0,0.1,0) + Eigen::Vector3d(0.12,0.05,0);
    args.eefs.push_back(eef);
  }
  {
    wrench_distribution_existence::Args::EEF eef;
    eef.pose.translation() = Eigen::Vector3d(0,0.1,0) + Eigen::Vector3d(0.12,-0.05,0);
    args.eefs.push_back(eef);
  }
  {
    wrench_distribution_existence::Args::EEF eef;
    eef.pose.translation() = Eigen::Vector3d(0,0.1,0) + Eigen::Vector3d(-0.09,0.05,0);
    args.eefs.push_back(eef);
  }
  {
    wrench_distribution_existence::Args::EEF eef;
    eef.pose.translation() = Eigen::Vector3d(0,0.1,0) + Eigen::Vector3d(-0.09,-0.05,0);
    args.eefs.push_back(eef);
  }

  for(int i=0;i<args.eefs.size();i++){
    args.eefs[i].F.resize(6,4);
    args.eefs[i].F.insert(0,0) = 1/sqrt(3); args.eefs[i].F.insert(0,1) =-1/sqrt(3); args.eefs[i].F.insert(0,2) =-1/sqrt(3); args.eefs[i].F.insert(0,3) = 1/sqrt(3);
    args.eefs[i].F.insert(1,0) = 1/sqrt(3); args.eefs[i].F.insert(1,1) = 1/sqrt(3); args.eefs[i].F.insert(1,2) =-1/sqrt(3); args.eefs[i].F.insert(1,3) =-1/sqrt(3);
    args.eefs[i].F.insert(2,0) = 1/sqrt(3); args.eefs[i].F.insert(2,1) = 1/sqrt(3); args.eefs[i].F.insert(2,2) = 1/sqrt(3); args.eefs[i].F.insert(2,3) = 1/sqrt(3);
    args.eefs[i].dl = Eigen::VectorXd::Zero(4); args.eefs[i].C.resize(4,4); args.eefs[i].du = 1e10 * Eigen::VectorXd::Ones(4);
    for(int j=0;j<4;j++) args.eefs[i].C.insert(j,j) = 1.0;
  }

  args.m = 100.0;
  args.c = Eigen::Vector3d(0,0,0);
  args.debugLevel = 2;
  std::cerr << wrench_distribution_existence::check(args) << std::endl;

  std::cerr << std::endl << std::endl;

  args.c = Eigen::Vector3d(0.2,0,0);
  std::cerr << wrench_distribution_existence::check(args) << std::endl;

  return true;
}
