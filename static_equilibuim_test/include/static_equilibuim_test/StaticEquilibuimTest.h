#ifndef STATIC_EQUILIBUIM_TEST_H
#define STATIC_EQUILIBUIM_TEST_H

#include <Eigen/Eigen>

namespace static_equilibuim_test{
  // Testing Static Equilibuium for Legged Robots (2008)の方法
  // Ax = b
  // dl <= Cx <= du
  // x の 0,1番目の要素を投影
  // 返り値
  // l_out <= M_out x[0:1] <= u_out
  bool calcProjection(const Eigen::SparseMatrix<double,Eigen::RowMajor>& A,
                      const Eigen::VectorXd& b,
                      const Eigen::SparseMatrix<double,Eigen::RowMajor>& C,
                      const Eigen::VectorXd& dl,
                      const Eigen::VectorXd& du,
                      Eigen::SparseMatrix<double,Eigen::RowMajor>& M_out,
                      Eigen::VectorXd& l_out,
                      Eigen::VectorXd& u_out,
                      std::vector<Eigen::Vector2d>& vertices,
                      int debuglevel = 0,
                      double eps = 0.05,
                      size_t maxiter = 30,
                      bool revertIfFail = false,
                      double lpTolerance = 1e-12
                      );
}

#endif
