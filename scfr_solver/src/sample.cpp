#include <scfr_solver/scfr_solver.h>
#include <iostream>

int main(){

  std::vector<Eigen::Isometry3d> poses;
  std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As;
  std::vector<Eigen::VectorXd> bs;
  std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Cs;
  std::vector<Eigen::VectorXd> dls;
  std::vector<Eigen::VectorXd> dus;
  double m = 100.0;

  {
    // rleg
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(0,-0.1,0);
    Eigen::SparseMatrix<double,Eigen::RowMajor> A(0,6);
    Eigen::VectorXd b(0);
    Eigen::SparseMatrix<double,Eigen::RowMajor> C(11,6);
    Eigen::VectorXd dl = Eigen::VectorXd::Zero(11);
    Eigen::VectorXd du = Eigen::VectorXd::Constant(11, 1e10);
    C.insert(0,2) = 1.0; dl[0] = 0.0; du[0] = 2000.0;
    C.insert(1,0) = 1.0; C.insert(1,2) = 0.2;
    C.insert(2,0) = -1.0; C.insert(2,2) = 0.2;
    C.insert(3,1) = 1.0; C.insert(3,2) = 0.2;
    C.insert(4,1) = -1.0; C.insert(4,2) = 0.2;
    C.insert(5,2) = 0.05; C.insert(5,3) = 1.0;
    C.insert(6,2) = 0.05; C.insert(6,3) = -1.0;
    C.insert(7,2) = 0.1; C.insert(7,4) = 1.0;
    C.insert(8,2) = 0.1; C.insert(8,4) = -1.0;
    C.insert(9,2) = 0.01; C.insert(9,5) = 1.0;
    C.insert(10,2) = 0.01; C.insert(10,5) = -1.0;
    poses.push_back(pose);
    As.push_back(A);
    bs.push_back(b);
    Cs.push_back(C);
    dls.push_back(dl);
    dus.push_back(du);
  }
  {
    // lleg
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(0,0.1,0);
    Eigen::SparseMatrix<double,Eigen::RowMajor> A(0,6);
    Eigen::VectorXd b(0);
    Eigen::SparseMatrix<double,Eigen::RowMajor> C(11,6);
    Eigen::VectorXd dl = Eigen::VectorXd::Zero(11);
    Eigen::VectorXd du = Eigen::VectorXd::Constant(11, 1e10);
    C.insert(0,2) = 1.0; dl[0] = 0.0; du[0] = 2000.0;
    C.insert(1,0) = 1.0; C.insert(1,2) = 0.2;
    C.insert(2,0) = -1.0; C.insert(2,2) = 0.2;
    C.insert(3,1) = 1.0; C.insert(3,2) = 0.2;
    C.insert(4,1) = -1.0; C.insert(4,2) = 0.2;
    C.insert(5,2) = 0.05; C.insert(5,3) = 1.0;
    C.insert(6,2) = 0.05; C.insert(6,3) = -1.0;
    C.insert(7,2) = 0.1; C.insert(7,4) = 1.0;
    C.insert(8,2) = 0.1; C.insert(8,4) = -1.0;
    C.insert(9,2) = 0.01; C.insert(9,5) = 1.0;
    C.insert(10,2) = 0.01; C.insert(10,5) = -1.0;
    poses.push_back(pose);
    As.push_back(A);
    bs.push_back(b);
    Cs.push_back(C);
    dls.push_back(dl);
    dus.push_back(du);
  }


  Eigen::SparseMatrix<double,Eigen::RowMajor> M;
  Eigen::VectorXd l;
  Eigen::VectorXd u;
  std::vector<Eigen::Vector2d> vertices;
  bool result = scfr_solver::calcSCFR(poses,
                                      As,
                                      bs,
                                      Cs,
                                      dls,
                                      dus,
                                      m,
                                      M,
                                      l,
                                      u,
                                      vertices);
  std::cout << "result" << std::endl;
  std::cout << result << std::endl;
  std::cout << "M" << std::endl;
  std::cout << M << std::endl;
  std::cout << "l" << std::endl;
  std::cout << l << std::endl;
  std::cout << "u" << std::endl;
  std::cout << u << std::endl;
  std::cout << "vertices" << std::endl;
  for(int i=0;i<vertices.size();i++){
    std::cout << vertices[i] << std::endl;
  }

  return 0;
}
