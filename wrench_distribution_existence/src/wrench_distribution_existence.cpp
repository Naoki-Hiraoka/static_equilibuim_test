#include <wrench_distribution_existence/wrench_distribution_existence.h>

#include <iostream>
#include <clpeigen/clpeigen.h>

namespace wrench_distribution_existence {
  inline Eigen::Matrix3d hat(const Eigen::Vector3d& x) {
    Eigen::Matrix3d M;
    M <<  0.0, -x(2),   x(1),
        x(2),   0.0,  -x(0),
        -x(1),  x(0),   0.0;
    return M;
  }

  bool check(const Args& args){
    int F_cols = 0;
    int A_rows = 0;
    int C_rows = 0;
    for(int i=0;i<args.eefs.size();i++){
      if(args.eefs[i].F.rows() != 6 ||
         (args.eefs[i].A.rows() != 0 && args.eefs[i].A.cols() != args.eefs[i].F.cols()) ||
         (args.eefs[i].C.rows() != 0 && args.eefs[i].C.cols() != args.eefs[i].F.cols())
         ) {
        std::cerr << "[" <<__FUNCTION__ << "] dimension mismatch:" << std::endl;
        return false;
      }
      F_cols += args.eefs[i].F.cols();
      A_rows += args.eefs[i].A.rows();
      C_rows += args.eefs[i].C.rows();
    }

    /*
      objective:  mazimize ox
      subject to: lbA <= Ax <= ubA
                  lb <= x <= ub
     */
    Eigen::VectorXd o = - Eigen::VectorXd::Ones(F_cols);
    Eigen::SparseMatrix<double,Eigen::RowMajor> A(6+A_rows+C_rows, F_cols);
    Eigen::VectorXd lbA = Eigen::VectorXd::Zero(6+A_rows+C_rows);
    Eigen::VectorXd ubA = Eigen::VectorXd::Zero(6+A_rows+C_rows);
    Eigen::VectorXd lb(A.cols()); for(size_t i=0;i<lb.rows();i++) lb[i] = -args.maxValue;
    Eigen::VectorXd ub(A.cols()); for(size_t i=0;i<ub.rows();i++) ub[i] = args.maxValue;

    // 重心周りの力のつりあい. (0-6)
    {
      Eigen::SparseMatrix<double,Eigen::ColMajor> G(6,F_cols);
      int col = 0;
      for(int i=0;i<args.eefs.size();i++){
        Eigen::SparseMatrix<double,Eigen::RowMajor> GraspMatrix(6,6);
        {
          const Eigen::Matrix3d& R = args.eefs[i].pose.linear();
          const Eigen::Matrix3d& p_x_R = wrench_distribution_existence::hat(args.eefs[i].pose.translation() - args.c) * R;
          /*
            |R   0|
            |pxR R|
           */
          for(int k=0;k<3;k++){
            for(int j=0;j<3;j++) GraspMatrix.insert(j,k) = R(j,k);
            for(int j=0;j<3;j++) GraspMatrix.insert(3+j,k) = p_x_R(j,k);
          }
          for(int k=0;k<3;k++){
            for(int j=0;j<3;j++) GraspMatrix.insert(3+j,3+k) = R(j,k);
          }
        }
        Eigen::SparseMatrix<double,Eigen::ColMajor> Gi = GraspMatrix * args.eefs[i].F;
        G.middleCols(col,args.eefs[i].F.cols()) = Gi;
        col += args.eefs[i].F.cols();
      }
      A.topRows<6>() = G;
    }
    {
      // 重力
      ubA(2) = lbA(2) = args.m * args.g;
      // extWrench
      ubA.head<6>() += args.extWrench;
      lbA.head<6>() += args.extWrench;
    }

    // 接触力
    {
      int row = 6;
      int col = 0;
      for(int i=0;i<args.eefs.size();i++){
        if(args.eefs[i].A.rows() > 0){
          Eigen::SparseMatrix<double, Eigen::ColMajor> Ai(args.eefs[i].A.rows(),F_cols);
          Ai.middleCols(col,args.eefs[i].A.cols()) = args.eefs[i].A;
          A.middleRows(row,args.eefs[i].A.rows()) = Ai;
          lbA.segment(row,args.eefs[i].A.rows()) = args.eefs[i].b;
          ubA.segment(row,args.eefs[i].A.rows()) = args.eefs[i].b;
          row += args.eefs[i].A.rows();
        }
        if(args.eefs[i].C.rows() > 0){
          Eigen::SparseMatrix<double, Eigen::ColMajor> Ci(args.eefs[i].C.rows(),F_cols);
          Ci.middleCols(col,args.eefs[i].C.cols()) = args.eefs[i].C;
          A.middleRows(row,args.eefs[i].C.rows()) = Ci;
          lbA.segment(row,args.eefs[i].C.rows()) = args.eefs[i].dl;
          ubA.segment(row,args.eefs[i].C.rows()) = args.eefs[i].du;
          row += args.eefs[i].C.rows();
        }
        col += args.eefs[i].F.cols();
      }
    }

    clpeigen::solver solver;
    // solver.model().setPrimalTolerance(1e-12);//default 1e-7. vertexを見逃さないようにする
    // solver.model().setDualTolerance(1e-12);
    solver.initialize(o,A,lbA,ubA,lb,ub,args.debugLevel);
    bool solved =  solver.solve();

    if(args.debugLevel >= 2){
      std::cerr << "[" <<__FUNCTION__ << "] check:" << std::endl;
      std::cerr << "A" << std::endl;
      std::cerr << A << std::endl;
      std::cerr << "ubA" << std::endl;
      std::cerr << ubA << std::endl;
      std::cerr << "lbA" << std::endl;
      std::cerr << lbA << std::endl;
      if(solved){
        Eigen::VectorXd solution;
        solver.getSolution(solution);
        std::cerr << "[" <<__FUNCTION__ << "] solved. result:" << std::endl;
        std::cerr << solution.transpose() << std::endl;
      }else{
        std::cerr << "[" <<__FUNCTION__ << "] !solved." << std::endl;
      }
    }

    return solved;
  }



  bool check2(const Args2& args){
    int F_cols = 0;
    int A_rows = 0;
    int C_rows = 0;
    for(int i=0;i<args.eefs.size();i++){
      if(args.eefs[i].F.size() != args.objects.size() ||
         (args.eefs[i].A.rows() != 0 && args.eefs[i].A.cols() != args.eefs[i].dim) ||
         (args.eefs[i].C.rows() != 0 && args.eefs[i].C.cols() != args.eefs[i].dim)){
        std::cerr << "[" <<__FUNCTION__ << "] dimension mismatch:" << std::endl;
        return false;
      }
      for(int j=0;j<args.eefs[i].F.size();j++){
        if((args.eefs[i].F[j].rows() != 0 && args.eefs[i].F[j].rows() != 6) ||
           (args.eefs[i].F[j].cols() != 0 && args.eefs[i].F[j].cols() != args.eefs[i].dim)){
          std::cerr << "[" <<__FUNCTION__ << "] dimension mismatch:" << std::endl;
          return false;
        }
      }
      F_cols += args.eefs[i].dim;
      A_rows += args.eefs[i].A.rows();
      C_rows += args.eefs[i].C.rows();
    }

    /*
      objective:  mazimize ox
      subject to: lbA <= Ax <= ubA
                  lb <= x <= ub
     */
    int num_balances = 6 * args.objects.size();
    Eigen::VectorXd o = - Eigen::VectorXd::Ones(F_cols);
    Eigen::SparseMatrix<double,Eigen::RowMajor> A(num_balances+A_rows+C_rows, F_cols);
    Eigen::VectorXd lbA = Eigen::VectorXd::Zero(num_balances+A_rows+C_rows);
    Eigen::VectorXd ubA = Eigen::VectorXd::Zero(num_balances+A_rows+C_rows);
    Eigen::VectorXd lb(A.cols()); for(size_t i=0;i<lb.rows();i++) lb[i] = -args.maxValue;
    Eigen::VectorXd ub(A.cols()); for(size_t i=0;i<ub.rows();i++) ub[i] = args.maxValue;

    // 重心周りの力のつりあい.
    for(int o=0;o<args.objects.size();o++){
      Eigen::SparseMatrix<double,Eigen::ColMajor> G(6,F_cols);
      int col = 0;
      for(int i=0;i<args.eefs.size();i++){
        if(args.eefs[i].F[o].rows() != 0 && args.eefs[i].F[o].cols() != 0) {
          Eigen::SparseMatrix<double,Eigen::RowMajor> GraspMatrix(6,6);
          {
            const Eigen::Matrix3d& R = args.eefs[i].pose.linear();
            const Eigen::Matrix3d& p_x_R = wrench_distribution_existence::hat(args.eefs[i].pose.translation() - args.objects[o].c) * R;
            /*
              |R   0|
              |pxR R|
            */
            for(int k=0;k<3;k++){
              for(int j=0;j<3;j++) GraspMatrix.insert(j,k) = R(j,k);
              for(int j=0;j<3;j++) GraspMatrix.insert(3+j,k) = p_x_R(j,k);
            }
            for(int k=0;k<3;k++){
              for(int j=0;j<3;j++) GraspMatrix.insert(3+j,3+k) = R(j,k);
            }
          }
          Eigen::SparseMatrix<double,Eigen::ColMajor> Gi = GraspMatrix * args.eefs[i].F[o];
          G.middleCols(col,args.eefs[i].dim) = Gi;
        }
        col += args.eefs[i].dim;
      }
      A.middleRows<6>(6*o) = G;
    }
    for(int o=0;o<args.objects.size();o++){
      // 重力
      ubA(6*o+2) = lbA(6*o+2) = args.objects[o].m * args.g;
      // extWrench
      ubA.segment<6>(6*o) += args.objects[o].extWrench;
      lbA.segment<6>(6*o) += args.objects[o].extWrench;
    }

    // 接触力
    {
      int row = num_balances;
      int col = 0;
      for(int i=0;i<args.eefs.size();i++){
        if(args.eefs[i].A.rows() > 0){
          Eigen::SparseMatrix<double, Eigen::ColMajor> Ai(args.eefs[i].A.rows(),F_cols);
          Ai.middleCols(col,args.eefs[i].A.cols()) = args.eefs[i].A;
          A.middleRows(row,args.eefs[i].A.rows()) = Ai;
          lbA.segment(row,args.eefs[i].A.rows()) = args.eefs[i].b;
          ubA.segment(row,args.eefs[i].A.rows()) = args.eefs[i].b;
          row += args.eefs[i].A.rows();
        }
        if(args.eefs[i].C.rows() > 0){
          Eigen::SparseMatrix<double, Eigen::ColMajor> Ci(args.eefs[i].C.rows(),F_cols);
          Ci.middleCols(col,args.eefs[i].C.cols()) = args.eefs[i].C;
          A.middleRows(row,args.eefs[i].C.rows()) = Ci;
          lbA.segment(row,args.eefs[i].C.rows()) = args.eefs[i].dl;
          ubA.segment(row,args.eefs[i].C.rows()) = args.eefs[i].du;
          row += args.eefs[i].C.rows();
        }
        col += args.eefs[i].dim;
      }
    }

    clpeigen::solver solver;
    // solver.model().setPrimalTolerance(1e-12);//default 1e-7. vertexを見逃さないようにする
    // solver.model().setDualTolerance(1e-12);
    solver.initialize(o,A,lbA,ubA,lb,ub,args.debugLevel);
    bool solved =  solver.solve();

    if(args.debugLevel >= 2){
      std::cerr << "[" <<__FUNCTION__ << "] check:" << std::endl;
      std::cerr << "A" << std::endl;
      std::cerr << A << std::endl;
      std::cerr << "ubA" << std::endl;
      std::cerr << ubA << std::endl;
      std::cerr << "lbA" << std::endl;
      std::cerr << lbA << std::endl;
      if(solved){
        Eigen::VectorXd solution;
        solver.getSolution(solution);
        std::cerr << "[" <<__FUNCTION__ << "] solved. result:" << std::endl;
        std::cerr << solution.transpose() << std::endl;
      }else{
        std::cerr << "[" <<__FUNCTION__ << "] !solved." << std::endl;
      }
    }

    return solved;
  }


}
