#ifndef SCFR_SOLVER_SCFR_SOLVER_H
#define SCFR_SOLVER_SCFR_SOLVER_H

#include <map>
#include <memory>
#include <Eigen/Eigen>
#include <vector>

namespace scfr_solver {

  class SCFRParam {
  public:
    int debugLevel = 0;
    double eps = 0.05; // 0.05より小さいと時間がかかりすぎる.
    size_t maxIter = 30;
    bool revertIfFail = false;
    double lpTolerance = 1e-15; // 1e-12だと不正確. 1e-15くらい小さくないと，vertexを見逃す
  };

  bool calcSCFR(const std::vector<Eigen::Transform<double, 3, Eigen::AffineCompact> >& poses,
                const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& As, // pose local frame. 列は6(F N)
                const std::vector<Eigen::VectorXd>& bs,
                const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Cs, // pose local frame. 列は6(F N)
                const std::vector<Eigen::VectorXd>& dls,
                const std::vector<Eigen::VectorXd>& dus,
                const double& m, // robotの質量
                Eigen::SparseMatrix<double,Eigen::RowMajor>& M, // world frame. 列は2(XY)
                Eigen::VectorXd& l,
                Eigen::VectorXd& u,
                std::vector<Eigen::Vector2d>& vertices,
                const SCFRParam& param = SCFRParam()
                );

};

#endif
