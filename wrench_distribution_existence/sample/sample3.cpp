#include <wrench_distribution_existence/wrench_distribution_existence.h>
#include <iostream>

int main(){
  wrench_distribution_existence::Args2 args;

  {
    // rleg
    wrench_distribution_existence::Args2::EEF eef;
    eef.pose.translation() = Eigen::Vector3d(0,-0.1,0);
    eef.dim = 6;
    eef.F.resize(2);
    eef.F[0].resize(6,6); for(int i=0;i<6;i++) eef.F[0].insert(i,i) = 1.0;
    eef.F[1].resize(6,6);
    eef.dl = Eigen::VectorXd::Zero(11); eef.C.resize(11,6); eef.du = 1e10 * Eigen::VectorXd::Ones(11);
    eef.C.insert(0,2) = 1.0; eef.du[0] = 2000.0;
    eef.C.insert(1,0) = 1.0; eef.C.insert(1,2) = 0.2;
    eef.C.insert(2,0) = -1.0; eef.C.insert(2,2) = 0.2;
    eef.C.insert(3,1) = 1.0; eef.C.insert(3,2) = 0.2;
    eef.C.insert(4,1) = -1.0; eef.C.insert(4,2) = 0.2;
    eef.C.insert(5,2) = 0.05; eef.C.insert(5,3) = 1.0;
    eef.C.insert(6,2) = 0.05; eef.C.insert(6,3) = -1.0;
    eef.C.insert(7,2) = 0.12; eef.C.insert(7,4) = 1.0;
    eef.C.insert(8,2) = 0.09; eef.C.insert(8,4) = -1.0;
    eef.C.insert(9,2) = 0.005; eef.C.insert(9,5) = 1.0;
    eef.C.insert(10,2) = 0.005; eef.C.insert(10,5) = -1.0;
    args.eefs.push_back(eef);
  }
  {
    // lleg
    wrench_distribution_existence::Args2::EEF eef;
    eef.pose.translation() = Eigen::Vector3d(0,+0.1,0);
    eef.dim = 6;
    eef.F.resize(2);
    eef.F[0].resize(6,6); for(int i=0;i<6;i++) eef.F[0].insert(i,i) = 1.0;
    eef.F[1].resize(6,6);
    eef.dl = Eigen::VectorXd::Zero(11); eef.C.resize(11,6); eef.du = 1e10 * Eigen::VectorXd::Ones(11);
    eef.C.insert(0,2) = 1.0; eef.du[0] = 2000.0;
    eef.C.insert(1,0) = 1.0; eef.C.insert(1,2) = 0.2;
    eef.C.insert(2,0) = -1.0; eef.C.insert(2,2) = 0.2;
    eef.C.insert(3,1) = 1.0; eef.C.insert(3,2) = 0.2;
    eef.C.insert(4,1) = -1.0; eef.C.insert(4,2) = 0.2;
    eef.C.insert(5,2) = 0.05; eef.C.insert(5,3) = 1.0;
    eef.C.insert(6,2) = 0.05; eef.C.insert(6,3) = -1.0;
    eef.C.insert(7,2) = 0.12; eef.C.insert(7,4) = 1.0;
    eef.C.insert(8,2) = 0.09; eef.C.insert(8,4) = -1.0;
    eef.C.insert(9,2) = 0.005; eef.C.insert(9,5) = 1.0;
    eef.C.insert(10,2) = 0.005; eef.C.insert(10,5) = -1.0;
    args.eefs.push_back(eef);
  }
  {
    // rarm
    wrench_distribution_existence::Args2::EEF eef;
    eef.pose.translation() = Eigen::Vector3d(0.7,-0.1,1.0);
    eef.pose.linear() = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()).toRotationMatrix();
    eef.dim = 6;
    eef.F.resize(2);
    eef.F[0].resize(6,6); for(int i=0;i<6;i++) eef.F[0].insert(i,i) = 1.0;
    eef.F[1].resize(6,6); for(int i=0;i<6;i++) eef.F[1].insert(i,i) = -1.0;
    eef.dl = Eigen::VectorXd::Zero(11); eef.C.resize(11,6); eef.du = 1e10 * Eigen::VectorXd::Ones(11);
    eef.C.insert(0,2) = 1.0; eef.du[0] = 2000.0;
    eef.C.insert(1,0) = 1.0; eef.C.insert(1,2) = 0.2;
    eef.C.insert(2,0) = -1.0; eef.C.insert(2,2) = 0.2;
    eef.C.insert(3,1) = 1.0; eef.C.insert(3,2) = 0.2;
    eef.C.insert(4,1) = -1.0; eef.C.insert(4,2) = 0.2;
    eef.C.insert(5,2) = 0.05; eef.C.insert(5,3) = 1.0;
    eef.C.insert(6,2) = 0.05; eef.C.insert(6,3) = -1.0;
    eef.C.insert(7,2) = 0.05; eef.C.insert(7,4) = 1.0;
    eef.C.insert(8,2) = 0.05; eef.C.insert(8,4) = -1.0;
    eef.C.insert(9,2) = 0.005; eef.C.insert(9,5) = 1.0;
    eef.C.insert(10,2) = 0.005; eef.C.insert(10,5) = -1.0;
    args.eefs.push_back(eef);
  }
  {
    // larm
    wrench_distribution_existence::Args2::EEF eef;
    eef.pose.translation() = Eigen::Vector3d(0.7,0.1,1.0);
    eef.pose.linear() = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX()).toRotationMatrix();
    eef.dim = 6;
    eef.F.resize(2);
    eef.F[0].resize(6,6); for(int i=0;i<6;i++) eef.F[0].insert(i,i) = 1.0;
    eef.F[1].resize(6,6); for(int i=0;i<6;i++) eef.F[1].insert(i,i) = -1.0;
    eef.dl = Eigen::VectorXd::Zero(11); eef.C.resize(11,6); eef.du = 1e10 * Eigen::VectorXd::Ones(11);
    eef.C.insert(0,2) = 1.0; eef.du[0] = 2000.0;
    eef.C.insert(1,0) = 1.0; eef.C.insert(1,2) = 0.2;
    eef.C.insert(2,0) = -1.0; eef.C.insert(2,2) = 0.2;
    eef.C.insert(3,1) = 1.0; eef.C.insert(3,2) = 0.2;
    eef.C.insert(4,1) = -1.0; eef.C.insert(4,2) = 0.2;
    eef.C.insert(5,2) = 0.05; eef.C.insert(5,3) = 1.0;
    eef.C.insert(6,2) = 0.05; eef.C.insert(6,3) = -1.0;
    eef.C.insert(7,2) = 0.05; eef.C.insert(7,4) = 1.0;
    eef.C.insert(8,2) = 0.05; eef.C.insert(8,4) = -1.0;
    eef.C.insert(9,2) = 0.005; eef.C.insert(9,5) = 1.0;
    eef.C.insert(10,2) = 0.005; eef.C.insert(10,5) = -1.0;
    args.eefs.push_back(eef);
  }


  {
    wrench_distribution_existence::Args2::Object obj;
    obj.m = 100.0;
    obj.c = Eigen::Vector3d(0,0,1.0);
    args.objects.push_back(obj);
  }

  {
    wrench_distribution_existence::Args2::Object obj;
    obj.m = 5.0;
    obj.c = Eigen::Vector3d(0.7,0,1.0);
    args.objects.push_back(obj);
  }

  args.debugLevel = 2;
  std::cerr << wrench_distribution_existence::check2(args) << std::endl;

  std::cerr << std::endl << std::endl;

  args.objects[1].m = 100.0;
  std::cerr << wrench_distribution_existence::check2(args) << std::endl;

  return true;
}
