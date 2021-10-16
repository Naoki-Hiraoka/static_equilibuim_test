#include <static_equilibuim_test/StaticEquilibuimTest.h>

#include <clpeigen/clpeigen.h>

namespace static_equilibuim_test{
  // 3点a, b, cからなる三角形の面積
  double calcArea(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& c){
    Eigen::Vector2d v1 = b - a;
    Eigen::Vector2d v2 = c - a;
    return (v1[0]*v2[1]-v1[1]*v2[0])/2;
  }

  // v1, v2 の間の三角形の面積
  double calcMidArea(const std::tuple<Eigen::Vector2d,Eigen::Vector2d,double>& v1, const std::tuple<Eigen::Vector2d,Eigen::Vector2d,double>& v2){
    const Eigen::Vector2d p1 = std::get<0>(v1);
    const Eigen::Vector2d p2 = std::get<0>(v2);

    // 2点が一致している
    if((p1 - p2).norm() < 1e-6) return 0;

    // 2点が無限遠にある
    if(p1[0] > std::numeric_limits<double>::max()/2 && p2[0] > std::numeric_limits<double>::max()/2) return 0;
    if(p1[0] < -std::numeric_limits<double>::max()/2 && p2[0] < -std::numeric_limits<double>::max()/2) return 0;
    if(p1[1] > std::numeric_limits<double>::max()/2 && p2[1] > std::numeric_limits<double>::max()/2) return 0;
    if(p1[1] < -std::numeric_limits<double>::max()/2 && p2[1] < -std::numeric_limits<double>::max()/2) return 0;

    const Eigen::Vector2d n1 = std::get<1>(v1);
    const Eigen::Vector2d n2 = std::get<1>(v2);

    // pX: outerの交点を求める
    Eigen::Matrix<double,2,2> tmp;
    tmp.row(0) = n1.transpose(); tmp.row(1) = n2.transpose();
    Eigen::Vector2d pX = tmp.inverse() * Eigen::Vector2d(p1.dot(n1),p2.dot(n2));

    return calcArea(p1,pX,p2);
  }

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
                      int debuglevel,
                      double eps,
                      size_t maxiter
                      ){
    if(A.cols() != C.cols()){
      std::cerr << "[static_equilibuim_test::calcProjection] A.cols() != C.cols()" << std::endl;
      return false;
    }
    if(A.cols() < 2){
      std::cerr << "[static_equilibuim_test::calcProjection] A.cols() < 2" << std::endl;
      return false;
    }
    if(A.rows() != b.size()){
      std::cerr << "[static_equilibuim_test::calcProjection] A.rows() != b.size()" << std::endl;
      return false;
    }
    if(C.rows() != dl.size() || C.rows() != du.size()){
      std::cerr << "[static_equilibuim_test::calcProjection] C.rows() != dl.size() || C.rows() != du.size()" << std::endl;
      return false;
    }

    if(debuglevel){
      std::cerr << "before projection" << std::endl;
      std::cerr << "A" << std::endl << A << std::endl;
      std::cerr << "b" << std::endl << b << std::endl;
      std::cerr << "C" << std::endl << C << std::endl;
      std::cerr << "dl" << std::endl << dl << std::endl;
      std::cerr << "du" << std::endl << du << std::endl;
    }

    // initialize solver
    // lbM <= Mx <= ubM
    // lb <= x <= ub
    Eigen::SparseMatrix<double,Eigen::RowMajor> M(A.rows()+C.rows(),A.cols());
    M.topRows(A.rows()) = A;
    M.bottomRows(C.rows()) = C;
    Eigen::VectorXd lbM(A.rows()+C.rows());
    lbM.head(b.rows()) = b;
    lbM.tail(dl.rows()) = dl;
    Eigen::VectorXd ubM(A.rows()+C.rows());
    ubM.head(b.rows()) = b;
    ubM.tail(du.rows()) = du;
    Eigen::VectorXd lb(A.cols());
    for(size_t i=0;i<lb.rows();i++) lb[i] = -std::numeric_limits<double>::max();
    Eigen::VectorXd ub(A.cols());
    for(size_t i=0;i<ub.rows();i++) ub[i] = std::numeric_limits<double>::max();

    Eigen::VectorXd o = Eigen::VectorXd::Zero(M.cols());
    clpeigen::solver solver;
    solver.initialize(o,M,lbM,ubM,lb,ub,debuglevel);

    Eigen::VectorXd solution;
    std::list<std::tuple<Eigen::Vector2d,Eigen::Vector2d,double> > Y; //半時計回り. first: innervertex, second: outervector, third: この点と次の点で構成させるOuterとInnerの間の三角系の面積

    // first 4 solve
    std::vector<Eigen::Vector2d> outer(4);
    outer[0] = Eigen::Vector2d(1,0);
    outer[1] = Eigen::Vector2d(0,1);
    outer[2] = Eigen::Vector2d(-1,0);
    outer[3] = Eigen::Vector2d(0,-1);
    for(size_t i=0;i<outer.size();i++){
      o.head<2>() = outer[i];
      solver.updateObjective(o);
      if(!solver.solve()){
        return false; // solution is infeasible
      }
      solver.getSolution(solution);
      Y.push_back(std::tuple<Eigen::Vector2d,Eigen::Vector2d,double>(solution.head<2>(),outer[i],0));
    }

    //calc initial area_Y_inner area_Y_mid(=area_Y_outer - area_Y_inner)
    double area_Y_inner = 0;
    double area_Y_mid = 0;
    area_Y_inner += calcArea(std::get<0>(*std::next(Y.begin(),0)),
                             std::get<0>(*std::next(Y.begin(),1)),
                             std::get<0>(*std::next(Y.begin(),2)));
    area_Y_inner += calcArea(std::get<0>(*std::next(Y.begin(),2)),
                             std::get<0>(*std::next(Y.begin(),3)),
                             std::get<0>(*std::next(Y.begin(),0)));
    for(std::list<std::tuple<Eigen::Vector2d,Eigen::Vector2d,double> >::iterator it=Y.begin();it!=Y.end();it++){
      std::list<std::tuple<Eigen::Vector2d,Eigen::Vector2d,double> >::iterator nextit = std::next(it);
      if(nextit==Y.end()) nextit = Y.begin();
      double area = calcMidArea(*it,*nextit);
      std::get<2>(*it) = area;
      area_Y_mid += area;

    }

    // loop until conversion
    for(size_t i=0;i<maxiter;i++){
      if(area_Y_inner / (area_Y_inner+area_Y_mid) > 1 - eps) break; //近似が概ね収束

      // 最大値を探す
      double maxvalue = -1;
      std::list<std::tuple<Eigen::Vector2d,Eigen::Vector2d,double> >::iterator maxit;
      for(std::list<std::tuple<Eigen::Vector2d,Eigen::Vector2d,double> >::iterator it=Y.begin();it!=Y.end();it++){
        if(std::get<2>(*it) > maxvalue){
          maxit = it;
          maxvalue = std::get<2>(*it);
        }
      }
      std::list<std::tuple<Eigen::Vector2d,Eigen::Vector2d,double> >::iterator nextit = std::next(maxit);
      if (nextit == Y.end()) nextit = Y.begin();

      Eigen::Vector2d p1 = std::get<0>(*maxit);
      Eigen::Vector2d p2 = std::get<0>(*nextit);
      Eigen::Vector2d n = Eigen::Vector2d((p2-p1)[1],-(p2-p1)[0]).normalized();

      // solve LP
      o.head<2>() = n;
      solver.updateObjective(o);
      if(!solver.solve()){
        return false; // solution is infeasible
      }
      solver.getSolution(solution);
      Eigen::Vector2d pX = solution.head<2>();

      area_Y_inner += calcArea(p1,pX,p2);
      area_Y_mid -= std::get<2>(*maxit);
      std::tuple<Eigen::Vector2d,Eigen::Vector2d,double> vX(pX,n,0);
      std::get<2>(*maxit) = calcMidArea(*maxit,vX);
      area_Y_mid += std::get<2>(*maxit);
      std::get<2>(vX) = calcMidArea(vX,*nextit);
      area_Y_mid += std::get<2>(vX);
      Y.insert(std::next(maxit),vX);
    }

    //return value
    M_out = Eigen::SparseMatrix<double,Eigen::RowMajor>(Y.size(),2);
    u_out = Eigen::VectorXd(Y.size());
    l_out = Eigen::VectorXd(Y.size());
    vertices.resize(Y.size());
    size_t i=0;
    for(std::list<std::tuple<Eigen::Vector2d,Eigen::Vector2d,double> >::iterator it=Y.begin();it!=Y.end();it++){
      M_out.insert(i,0) = std::get<1>(*it)[0];
      M_out.insert(i,1) = std::get<1>(*it)[1];
      u_out[i] = std::get<1>(*it).dot(std::get<0>(*it));
      l_out[i] = -std::numeric_limits<double>::max();
      vertices[i] = std::get<0>(*it);
      i++;
    }

    if(debuglevel){
      std::cerr << "M_out" << std::endl << M_out << std::endl;
      std::cerr << "l_out" << std::endl << l_out << std::endl;
      std::cerr << "u_out" << std::endl << u_out << std::endl;
      std::cerr << "vertices" << std::endl;
      for(size_t i=0;i<vertices.size();i++){
        std::cerr << vertices[i] << std::endl;
      }
    }

    return true;
  }
};
