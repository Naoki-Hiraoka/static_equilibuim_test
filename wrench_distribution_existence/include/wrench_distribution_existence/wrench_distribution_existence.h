#ifndef WRENCH_DISTRIBUTION_EXISTENCE_WRENCH_DISTRIBUTION_EXISTENCE_H
#define WRENCH_DISTRIBUTION_EXISTENCE_WRENCH_DISTRIBUTION_EXISTENCE_H

#include <map>
#include <memory>
#include <Eigen/Eigen>
#include <vector>

namespace wrench_distribution_existence {

  class Args {
  public:
    class EEF {
    public:
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity(); // world系/
      Eigen::SparseMatrix<double,Eigen::RowMajor> F; // 6 x M. 接触力を表す変数(M次元)をpose local frameのロボットが受ける6次元力(F N)に変換する行列. FACE表現なら単位行列であり、SPAN表現なら[n 0]^Tである
      Eigen::SparseMatrix<double,Eigen::RowMajor> A; // 接触力を表す変数(M次元)の制約. ? x M.
      Eigen::VectorXd b;
      Eigen::SparseMatrix<double,Eigen::RowMajor> C; // 接触力を表す変数(M次元)の制約. ?? x N
      Eigen::VectorXd dl;
      Eigen::VectorXd du;
    };
    std::vector<EEF> eefs;
    double m = 1.0; // robotの質量
    Eigen::Vector3d c = Eigen::Vector3d::Zero(); // world系. 重心位置
    Eigen::Matrix<double, 6, 1> extWrench = Eigen::Matrix<double, 6, 1>::Zero(); // world系. 重心周り. F - mgした後残っている力. 基本的には、この値だけ加速することになる.
    double g = 9.80665;
    double maxValue = 1e5; // 接触力を表す変数の最大・最小値. 制約A,Cがゆるい場合、無限に力を発揮可能なためLPの解が定まらないことから、maxValueで制限する.
  public:
    int debugLevel = 0; // 0,1: no output. 2: verbose
  };

  bool check(const Args& args); // 力分配が存在すればtrue


  class Args2 {
  public:
    class EEF {
    public:
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity(); // world系/
      double dim = 0;
      std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > F; // サイズI. 6 x M. 接触力を表す変数(M次元)をpose local frameのi番目のロボットが受ける6次元力(F N)に変換する行列. FACE表現なら単位行列であり、SPAN表現なら[n 0]^Tである
      Eigen::SparseMatrix<double,Eigen::RowMajor> A; // 接触力を表す変数(M次元)の制約. ? x M.
      Eigen::VectorXd b;
      Eigen::SparseMatrix<double,Eigen::RowMajor> C; // 接触力を表す変数(M次元)の制約. ?? x N
      Eigen::VectorXd dl;
      Eigen::VectorXd du;
    };
    std::vector<EEF> eefs;
    class Object {
    public:
      double m = 1.0; // 質量
      Eigen::Vector3d c = Eigen::Vector3d::Zero(); // world系. 重心位置
      Eigen::Matrix<double, 6, 1> extWrench = Eigen::Matrix<double, 6, 1>::Zero(); // world系. 重心周り. F - mgした後残っている力. 基本的には、この値だけ加速することになる.
    };
    std::vector<Object> objects; // サイズI
    double g = 9.80665;
    double maxValue = 1e5; // 接触力を表す変数の最大・最小値. 制約A,Cがゆるい場合、無限に力を発揮可能なためLPの解が定まらないことから、maxValueで制限する.
  public:
    int debugLevel = 0; // 0,1: no output. 2: verbose
  };

  bool check2(const Args2& args); // 力分配が存在すればtrue

};

#endif
