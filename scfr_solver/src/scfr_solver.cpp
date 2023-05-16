#include <scfr_solver/scfr_solver.h>

#include <iostream>
#include <static_equilibuim_test/StaticEquilibuimTest.h>

namespace scfr_solver {
  inline Eigen::Matrix3d hat(const Eigen::Vector3d& x) {
    Eigen::Matrix3d M;
    M <<  0.0, -x(2),   x(1),
        x(2),   0.0,  -x(0),
        -x(1),  x(0),   0.0;
    return M;
  }

  inline bool appendRow(const std::vector<Eigen::VectorXd>& vs, Eigen::VectorXd& vout){
    size_t rows = 0;
    for(size_t i=0;i<vs.size();i++){
      rows += vs[i].size();
    }
    vout.resize(rows);
    size_t idx = 0;
    for(size_t i=0;i<vs.size();i++){
      vout.segment(idx,vs[i].size()) = vs[i];
      idx += vs[i].size();
    }

    return true;
  }

  inline bool appendCol(const std::vector<Eigen::SparseMatrix<double, Eigen::ColMajor> >& Ms, Eigen::SparseMatrix<double, Eigen::ColMajor >& Mout){
    if(Ms.size() == 0) {
      Mout.resize(Mout.rows(),0);//もとのMoutのrowを用いる
      return true;
    }
    size_t rows = Ms[0].rows();
    size_t cols = 0;
    for(size_t i=0;i<Ms.size();i++){
      cols += Ms[i].cols();
      if(Ms[i].rows() != rows){
        std::cerr << "[appendCol] Ms[i].rows() " << Ms[i].rows() << " != rows " << rows << std::endl;
        return false;
      }
    }
    Mout.resize(rows,cols);
    size_t idx = 0;
    for(size_t i=0;i<Ms.size();i++){
      Mout.middleCols(idx,Ms[i].cols()) = Ms[i];
      idx += Ms[i].cols();
    }

    return true;
  }

  inline bool appendDiag(const std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> >& Ms, Eigen::SparseMatrix<double, Eigen::RowMajor >& Mout){
    if(Ms.size() == 0) {
      Mout.resize(0,0);
      return true;
    }
    size_t cols = 0;
    size_t rows = 0;
    for(size_t i=0;i<Ms.size();i++){
      rows += Ms[i].rows();
      cols += Ms[i].cols();
    }
    Mout.resize(rows,cols);
    size_t idx_row = 0;
    size_t idx_col = 0;
    for(size_t i=0;i<Ms.size();i++){
      Eigen::SparseMatrix<double, Eigen::ColMajor> M_ColMajor(Ms[i].rows(),cols);
      M_ColMajor.middleCols(idx_col,Ms[i].cols()) = Ms[i];
      Mout.middleRows(idx_row,M_ColMajor.rows()) = M_ColMajor;
      idx_row += Ms[i].rows();
      idx_col += Ms[i].cols();
    }

    return true;
  }

  bool calcSCFR(const std::vector<Eigen::Transform<double, 3, Eigen::AffineCompact>>& poses,
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
                const SCFRParam& param
                ) {

    if(poses.size() != As.size() ||
       poses.size() != bs.size() ||
       poses.size() != Cs.size() ||
       poses.size() != dls.size() ||
       poses.size() != dus.size()){
      std::cerr << "[" <<__FUNCTION__ << "] dimension mismatch" << std::endl;
      return false;
    }

    if(poses.size() == 0) {
      M.resize(0,2);
      l.resize(0);
      u.resize(0);
      vertices.clear();
      return true;
    }

    Eigen::SparseMatrix<double,Eigen::RowMajor> A;
    Eigen::VectorXd b;
    Eigen::SparseMatrix<double,Eigen::RowMajor> C;
    Eigen::VectorXd dl;
    Eigen::VectorXd du;
    {
      // compute A, b, C, dl, du
      // x = [dpx dpy dw1 dw2 ...]^T
      // wはpose座標系．poseまわり.サイズは6

      // Grasp Matrix Gx = h
      // 原点まわりのつりあい. 座標軸はworld系の向き
      Eigen::SparseMatrix<double,Eigen::RowMajor> G;
      Eigen::VectorXd h = Eigen::VectorXd::Zero(6);
      {
        h[2] = m*9.80665;

        std::vector<Eigen::SparseMatrix<double,Eigen::ColMajor> > Gs;
        {
          Eigen::SparseMatrix<double,Eigen::ColMajor> G01(6,2);
          G01.insert(3,1) = -m*9.80665;
          G01.insert(4,0) = m*9.80665;
          Gs.push_back(G01);
        }

        for(size_t i=0;i<poses.size();i++){
          Eigen::SparseMatrix<double,Eigen::ColMajor> GraspMatrix(6,6);
          {
            const Eigen::Transform<double, 3, Eigen::AffineCompact>& pos = poses[i];
            const Eigen::Matrix3d& R = pos.linear();
            const Eigen::Matrix3d& p_x_R = scfr_solver::hat(pos.translation()) * R;

            std::vector<Eigen::Triplet<double> > G_tripletList;
            for(size_t row=0;row<3;row++){
              for(size_t col=0;col<3;col++){
                G_tripletList.push_back(Eigen::Triplet<double>(row,col,R(row,col)));
              }
            }
            for(size_t row=0;row<3;row++){
              for(size_t col=0;col<3;col++){
                G_tripletList.push_back(Eigen::Triplet<double>(3+row,col,p_x_R(row,col)));
              }
            }
            for(size_t row=0;row<3;row++){
              for(size_t col=0;col<3;col++){
                  G_tripletList.push_back(Eigen::Triplet<double>(3+row,3+col,R(row,col)));
              }
            }
            GraspMatrix.setFromTriplets(G_tripletList.begin(), G_tripletList.end());
          }
          Gs.push_back(GraspMatrix);
        }

        Eigen::SparseMatrix<double,Eigen::ColMajor> G_ColMajor;
        scfr_solver::appendCol(Gs,G_ColMajor);
        G = G_ColMajor;
      }

      //接触力制約
      Eigen::SparseMatrix<double,Eigen::RowMajor> A_contact;
      Eigen::VectorXd b_contact;
      Eigen::SparseMatrix<double,Eigen::RowMajor> C_contact;
      Eigen::VectorXd dl_contact;
      Eigen::VectorXd du_contact;
      {
        std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > A_contacts;
        std::vector<Eigen::VectorXd> b_contacts;
        std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > C_contacts;
        std::vector<Eigen::VectorXd> dl_contacts;
        std::vector<Eigen::VectorXd> du_contacts;

        {
          Eigen::SparseMatrix<double,Eigen::RowMajor> A01(0,2);
          Eigen::VectorXd b01(0);
          Eigen::SparseMatrix<double,Eigen::RowMajor> C01(0,2);
          Eigen::VectorXd dl01(0);
          Eigen::VectorXd du01(0);
          A_contacts.push_back(A01);
          b_contacts.push_back(b01);
          C_contacts.push_back(C01);
          dl_contacts.push_back(dl01);
          du_contacts.push_back(du01);
        }

        for (size_t i=0;i<poses.size();i++){
          Eigen::SparseMatrix<double,Eigen::RowMajor> A = As[i];
          Eigen::VectorXd b = bs[i];
          Eigen::SparseMatrix<double,Eigen::RowMajor> C = Cs[i];
          Eigen::VectorXd dl = dls[i];
          Eigen::VectorXd du = dus[i];
          if(A.cols() != 6 ||
             C.cols() != 6 ||
             A.rows() != b.size() ||
             C.rows() != dl.size() ||
             C.rows() != du.size()){
            std::cerr << "[" <<__FUNCTION__ << "] dimension mismatch" << std::endl;
            return false;
          }
          A_contacts.push_back(A);
          b_contacts.push_back(b);
          C_contacts.push_back(C);
          dl_contacts.push_back(dl);
          du_contacts.push_back(du);
        }
        scfr_solver::appendDiag(A_contacts,A_contact);
        scfr_solver::appendRow(b_contacts,b_contact);
        scfr_solver::appendDiag(C_contacts,C_contact);
        scfr_solver::appendRow(dl_contacts,dl_contact);
        scfr_solver::appendRow(du_contacts,du_contact);
      }

      A = Eigen::SparseMatrix<double,Eigen::RowMajor>(G.rows()+A_contact.rows(),G.cols());
      A.topRows(G.rows()) = G;
      A.bottomRows(A_contact.rows()) = A_contact;
      b = Eigen::VectorXd(h.size()+b_contact.size());
      b.head(h.rows()) = h;
      b.tail(b_contact.rows()) = b_contact;
      C = C_contact;
      dl = dl_contact;
      du = du_contact;
    }

    // SCFRの各要素はx[0:1]の次元の大きさに揃っている
    if(!static_equilibuim_test::calcProjection(A,b,C,dl,du,M,l,u,vertices,param.debugLevel, param.eps,param.maxIter,param.revertIfFail,param.lpTolerance)){
      std::cerr << "[" <<__FUNCTION__ << "] projection failed" << std::endl;
      M.resize(0,2);
      l.resize(0);
      u.resize(0);
      vertices.clear();
      return false;
    }

    return true;
  }
}
