#include "ICP.h"
#include <algorithm>
///////////////////////////////////////////////////////////////////////////////
/// Compute the rigid motion for point-to-point and point-to-plane distances
/*
/// @param Source (one 3D point per column)
/// @param Target (one 3D point per column)
/// @param Confidence weights
template <typename Derived1, typename Derived2, typename Derived3>
Eigen::Affine3d RigidMotionEstimator::point_to_point(Eigen::MatrixBase<Derived1>& X,
                               Eigen::MatrixBase<Derived2>& Y,
                               const Eigen::MatrixBase<Derived3>& w) {
    /// Normalize weight vector
    Eigen::VectorXd w_normalized = w/w.sum();
    /// De-mean
    Eigen::Vector3d X_mean, Y_mean;
    for(int i=0; i<3; ++i) {
        X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();
        Y_mean(i) = (Y.row(i).array()*w_normalized.transpose().array()).sum();
    }
    X.colwise() -= X_mean;
    Y.colwise() -= Y_mean;
    /// Compute transformation
    Eigen::Affine3d transformation;
    Eigen::Matrix3d sigma = X * w_normalized.asDiagonal() * Y.transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
    if(svd.matrixU().determinant()*svd.matrixV().determinant() < 0.0) {
        Eigen::Vector3d S = Eigen::Vector3d::Ones(); S(2) = -1.0;
        transformation.linear().noalias() = svd.matrixV()*S.asDiagonal()*svd.matrixU().transpose();
    } else {
        transformation.linear().noalias() = svd.matrixV()*svd.matrixU().transpose();
    }
    transformation.translation().noalias() = Y_mean - transformation.linear()*X_mean;
    /// Apply transformation
    X = transformation*X;
    /// Re-apply mean
    X.colwise() += X_mean;
    Y.colwise() += Y_mean;
    return transformation;
}


/// @param Source (one 3D point per column)
/// @param Target (one 3D point per column)
/// @param Target normals (one 3D normal per column)
/// @param Confidence weights
/// @param Right hand side
template <typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5>
Eigen::Affine3d RigidMotionEstimator::point_to_plane(Eigen::MatrixBase<Derived1>& X,
                               Eigen::MatrixBase<Derived2>& Y,
                               Eigen::MatrixBase<Derived3>& N,
                               const Eigen::MatrixBase<Derived4>& w,
                               const Eigen::MatrixBase<Derived5>& u) {
    typedef Eigen::Matrix<double, 6, 6> Matrix66;
    typedef Eigen::Matrix<double, 6, 1> Vector6;
    typedef Eigen::Block<Matrix66, 3, 3> Block33;
    /// Normalize weight vector
    Eigen::VectorXd w_normalized = w/w.sum();
    /// De-mean
    Eigen::Vector3d X_mean;
    for(int i=0; i<3; ++i)
        X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();
   // X.colwise() -= X_mean;
   // Y.colwise() -= X_mean;
    /// Prepare LHS and RHS
    Matrix66 LHS = Matrix66::Zero();
    Vector6 RHS = Vector6::Zero();
    Block33 TL = LHS.topLeftCorner<3,3>();
    Block33 TR = LHS.topRightCorner<3,3>();
    Block33 BR = LHS.bottomRightCorner<3,3>();
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3,X.cols());
    #pragma omp parallel
    {
        #pragma omp for
        for(int i=0; i<X.cols(); i++) {
            //Eigen::Vector3d x = X.col(i);
            //Vector3d n = N.cols(i);
            C.col(i) = X.col(i).cross(N.col(i));
        }
        #pragma omp sections nowait
        {
            #pragma omp section
            for(int i=0; i<X.cols(); i++) TL.selfadjointView<Eigen::Upper>().rankUpdate(C.col(i), w(i));
            #pragma omp section
            for(int i=0; i<X.cols(); i++) TR += (C.col(i)*N.col(i).transpose())*w(i);
            #pragma omp section
            for(int i=0; i<X.cols(); i++) BR.selfadjointView<Eigen::Upper>().rankUpdate(N.col(i), w(i));
            #pragma omp section
            for(int i=0; i<C.cols(); i++) {
                double dist_to_plane = -((X.col(i) - Y.col(i)).dot(N.col(i)) - u(i))*w(i);
                RHS.head<3>() += C.col(i)*dist_to_plane;
                RHS.tail<3>() += N.col(i)*dist_to_plane;
            }
        }
    }
    LHS = LHS.selfadjointView<Eigen::Upper>();
    /// Compute transformation
    Eigen::Affine3d transformation;
    Eigen::LDLT<Matrix66> ldlt(LHS);
    RHS = ldlt.solve(RHS);
    transformation  = Eigen::AngleAxisd(RHS(0), Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(RHS(1), Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(RHS(2), Eigen::Vector3d::UnitZ());
    transformation.translation() = RHS.tail<3>();
    /// Apply transformation
    X = transformation*X;
    /// Re-apply mean
    //X.colwise() += X_mean;
    //Y.colwise() += X_mean;
    /// Return transformation
    return transformation;
}
*/


///////////////////////////////////////////////////////////////////////////////
/// ICP implementation using iterative reweighting
ICP::Parameters::Parameters() : f(TRIMMED),
                       p(0.85),
                       max_icp(5),
                       max_outer(1),
                       stop(1e-4) {}


void ICP::less_weight(Eigen::VectorXd& r, double p) {
    for(int i=0; i<r.rows(); ++i) {
        if(r(i)<p && r(i)>=0) r(i) = 1.0;
        else r(i) = 0.0;
    }
}

void ICP::median_weight(Eigen::VectorXd &r, double p) {
    std::vector<double> sortedDist(r.rows());
    for (int i = 0; i < r.rows(); ++i) {
        sortedDist[i] = r(i);
    }
    int n = r.rows()/2;
    std::nth_element(sortedDist.begin(), sortedDist.begin() + n, sortedDist.end());
    double median = sortedDist[n];
    less_weight(r, p*median);
}

/// Weight functions
/// @param Residuals
/// @param Parameter
void ICP::uniform_weight(Eigen::VectorXd& r) {
    r = Eigen::VectorXd::Ones(r.rows());
}
/// @param Residuals
/// @param Parameter
void ICP::pnorm_weight(Eigen::VectorXd& r, double p, double reg) {
    for(int i=0; i<r.rows(); ++i) {
        r(i) = p/(std::pow(r(i),2-p) + reg);
    }
}
/// @param Residuals
/// @param Parameter
void ICP::tukey_weight(Eigen::VectorXd& r, double p) {
    for(int i=0; i<r.rows(); ++i) {
        if(r(i) > p) r(i) = 0.0;
        else r(i) = std::pow((1.0 - std::pow(r(i)/p,2.0)), 2.0);
    }
}
/// @param Residuals
/// @param Parameter
void ICP::fair_weight(Eigen::VectorXd& r, double p) {
    for(int i=0; i<r.rows(); ++i) {
        r(i) = 1.0/(1.0 + r(i)/p);
    }
}
/// @param Residuals
/// @param Parameter
void ICP::logistic_weight(Eigen::VectorXd& r, double p) {
    for(int i=0; i<r.rows(); ++i) {
        r(i) = (p/r(i))*std::tanh(r(i)/p);
    }
}

bool ICP::sort_pred::operator()(const std::pair<int,double> &left,
                const std::pair<int,double> &right) {
    return left.second < right.second;
}

/// @param Residuals
/// @param Parameter
void ICP::trimmed_weight(Eigen::VectorXd& r, double p) {
    std::vector<std::pair<int, double> > sortedDist(r.rows());
    for(int i=0; i<r.rows(); ++i) {
        sortedDist[i] = std::pair<int, double>(i,r(i));
    }
    std::sort(sortedDist.begin(), sortedDist.end(), sort_pred());
    r.setZero();
    int nbV = r.rows()*p;
    for(int i=0; i<nbV; ++i) {
        r(sortedDist[i].first) = 1.0;
    }
}

/// @param Function type
/// @param Residuals
/// @param Parameter
void ICP::robust_weight(Function f, Eigen::VectorXd& r, double p) {
    switch(f) {
        case PNORM: pnorm_weight(r,p); break;
        case TUKEY: tukey_weight(r,p); break;
        case FAIR: fair_weight(r,p); break;
        case LOGISTIC: logistic_weight(r,p); break;
        case TRIMMED: trimmed_weight(r,p); break;
        case LESS: less_weight(r,p);break;
        case MEDIAN: median_weight(r, p);break;
        case NONE: uniform_weight(r); break;
        default: uniform_weight(r); break;
    }
}
/*
/// Reweighted ICP with point to point
/// @param Source (one 3D point per column)
/// @param Target (one 3D point per column)
/// @param Parameters
template <typename Derived1, typename Derived2>
void ICP::point_to_point(Eigen::MatrixBase<Derived1>& X,
                    Eigen::MatrixBase<Derived2>& Y,
                    Parameters par) {
    /// Build kd-tree
    nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);
    /// Buffers
    Eigen::Matrix3Xd Q = Eigen::Matrix3Xd::Zero(3, X.cols());
    Eigen::VectorXd W = Eigen::VectorXd::Zero(X.cols());
    Eigen::Matrix3Xd Xo1 = X;
    Eigen::Matrix3Xd Xo2 = X;
    /// ICP
    for(int icp=0; icp<par.max_icp; ++icp) {
        /// Find closest point
        #pragma omp parallel for
        for(int i=0; i<X.cols(); ++i) {
            Q.col(i) = Y.col(kdtree.closest(X.col(i).data()));
        }
        /// Computer rotation and translation
        for(int outer=0; outer<par.max_outer; ++outer) {
            /// Compute weights
            W = (X-Q).colwise().norm();
            robust_weight(par.f, W, par.p);
            /// Rotation and translation update
            RigidMotionEstimator::point_to_point(X, Q, W);
            /// Stopping criteria
            double stop1 = (X-Xo1).colwise().norm().maxCoeff();
            Xo1 = X;
            if(stop1 < par.stop) break;
        }
        /// Stopping criteria
        double stop2 = (X-Xo2).colwise().norm().maxCoeff();
        Xo2 = X;
        if(stop2 < par.stop) break;
    }
}
/// Reweighted ICP with point to plane
/// @param Source (one 3D point per column)
/// @param Target (one 3D point per column)
/// @param Target normals (one 3D normal per column)
/// @param Parameters
template <typename Derived1, typename Derived2, typename Derived3>
Eigen::Affine3d ICP::point_to_plane(
                    Eigen::MatrixBase<Derived1>& X,
                    Eigen::MatrixBase<Derived2>& Y,
                    Eigen::MatrixBase<Derived3>& N,
                    Parameters par) {

    /// Build kd-tree
    nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);
    //end  = clock();
    //of<<"build kdtree cost: "<<end-start<<std::endl;
    //start = clock();
    /// Buffers
    Eigen::Matrix3Xd Qp = Eigen::Matrix3Xd::Zero(3, X.cols());
    Eigen::Matrix3Xd Qn = Eigen::Matrix3Xd::Zero(3, X.cols());
    Eigen::VectorXd W = Eigen::VectorXd::Zero(X.cols());
    Eigen::Matrix3Xd Xo1 = X;
    Eigen::Matrix3Xd Xo2 = X;
    /// ICP
    Eigen::Affine3d Rt;
    Rt.setIdentity();
    //end = clock();
    //of<<"init cost: "<<end-start<<std::endl;
    //of<<"Y size: "<<Y.rows()<<' '<<Y.cols()<<std::endl;
    //of<<"X size: "<<X.rows()<<' '<<X.cols()<<std::endl;
    //start = clock();
    for(int icp=0; icp<par.max_icp; ++icp)
    {
        /// Find closest point
        #pragma omp parallel for
        for(int i=0; i<X.cols(); ++i)
        {
            int id = kdtree.closest(X.col(i).data());
            Qp.col(i) = Y.col(id);
            Qn.col(i) = N.col(id);
        }
        /// Computer rotation and translation
        for(int outer=0; outer<par.max_outer; ++outer)
        {
            /// Compute weights
            //start1 = clock();
            W = (Qn.array()*(X-Qp).array()).colwise().sum().abs().transpose();
            robust_weight(par.f, W, par.p);
            //end1 = clock();
            //cost1 += (end1-start1);
            //start2 = clock();
            /// Rotation and translation update
            Rt = RigidMotionEstimator::point_to_plane(X, Qp, Qn, W) * Rt;
            //end2 = clock();
            //cost2 += (end2-start2);
            /// Stopping criteria
            double stop1 = (X-Xo1).colwise().norm().maxCoeff();
            Xo1 = X;
            //of<<"innner: " <<outer<<' '<<stop1<<std::endl;
            if(stop1 < par.stop) break;
        }
        /// Stopping criteria
        double stop2 = (X-Xo2).colwise().norm().maxCoeff() ;
        Xo2 = X;
        //of<<"outer: " <<icp<<' '<<stop2<<std::endl;
        if(stop2 < par.stop) break;
    }
    //end = clock();
    //of<<"compute RT cost: "<<end-start<<std::endl;
    //of<<"cost1: "<<cost1<<std::endl;
    //of<<"cost2: "<<cost2<<std::endl;
    //of<<std::endl;
    return Rt;
}

template <typename Derived1>
Eigen::Vector2i ICP::ConvertRealWorldToProjective(const Eigen::MatrixBase<Derived1>& realWorldPoint, const cv::Mat& colorImage, double fx, double fy)
{
    Eigen::Vector2i coor;
    coor(0) = (realWorldPoint(0,0)/fx/(-realWorldPoint(2,0))+0.5)*colorImage.cols + 0.5;
    coor(1) = (0.5-realWorldPoint(1,0)/fy/(-realWorldPoint(2,0)))*colorImage.rows + 0.5;
    if(coor(0) < 0) coor(0) = 0;
    else if(coor(0) >= colorImage.cols) coor(0) = colorImage.cols - 1;
    if(coor(1) < 0) coor(1) = 0;
    else if(coor(1) >= colorImage.rows) coor(1) = colorImage.rows - 1;
    return coor;
}
/*
/// Reweighted ICP with point to plane projection
/// @param Source (one 3D point per column)
/// @param Target (one 3D point per column)
/// @param Target normals (one 3D normal per column)
/// @param Parameters
template <typename Derived1, typename Derived2, typename Derived3>//, typename Derived4>
Eigen::Affine3d ICP::point_to_plane(
                    Eigen::MatrixBase<Derived1>& X,
                    Eigen::MatrixBase<Derived2>& Y,
                    Eigen::MatrixBase<Derived3>& N,
                    //Eigen::MatrixBase<Derived4>& KeyPoint,
                    cv::Mat& colorImage,
                    Eigen::MatrixXi* inDepthImage,
                    double fx,
                    double fy,
                    //double& max_keypoint_dist,
                    Parameters par) {


    /// Buffers
    Eigen::Matrix3Xd Qp = Eigen::Matrix3Xd::Zero(3, X.cols());
    Eigen::Matrix3Xd Qn = Eigen::Matrix3Xd::Zero(3, X.cols());
    Eigen::VectorXd W = Eigen::VectorXd::Zero(X.cols());
    Eigen::Matrix3Xd Xo1 = X;
    Eigen::Matrix3Xd Xo2 = X;
    /// ICP
    Eigen::Affine3d Rt;
    Rt.setIdentity();
    //end = clock();
    //of<<"init cost: "<<end-start<<std::endl;
    //of<<"Y size: "<<Y.rows()<<' '<<Y.cols()<<std::endl;
    //of<<"X size: "<<X.rows()<<' '<<X.cols()<<std::endl;
    //start = clock();

    int icp=0;
    for(; icp<par.max_icp; ++icp)
    {
        /// Find closest point
        #pragma omp parallel for
        for(int i=0; i<X.cols(); ++i)
        {
            Eigen::Vector2i coor = ConvertRealWorldToProjective(X.col(i), colorImage, fx, fy);
            int id = inDepthImage->coeff(coor(1), coor(0));
            if(id != -1)
            {
                Qp.col(i) = Y.col(id);
                Qn.col(i) = N.col(id);
            }
            else
            {
                Qp.col(i) = X.col(i);
                Qn.col(i).setZero();
            }
        }
        /// Computer rotation and translation
        for(int outer=0; outer<par.max_outer; ++outer)
        {
            /// Compute weights
            //start1 = clock();
            W = (Qn.array()*(X-Qp).array()).colwise().sum().abs().transpose();
            robust_weight(par.f, W, par.p);
            //corresponded_rate = W.sum()/W.size();
            //of<<icp<<' '<<outer<<' '<<W.sum()/W.size()<<std::endl;
            //end1 = clock();
            //cost1 += (end1-start1);
            //start2 = clock();
            /// Rotation and translation update
            Rt = RigidMotionEstimator::point_to_plane(X, Qp, Qn, W) * Rt;
            //end2 = clock();
            //cost2 += (end2-start2);
            /// Stopping criteria
            double stop1 = (X-Xo1).colwise().norm().maxCoeff();
            Xo1 = X;
            //of<<"innner: " <<outer<<' '<<stop1<<std::endl;
            if(stop1 < par.stop) break;
        }
        /// Stopping criteria
        double stop2 = (X-Xo2).colwise().norm().maxCoeff() ;
        Xo2 = X;
        //of<<"outer: " <<icp<<' '<<stop2<<std::endl;
        if(stop2 < par.stop) break;
    }
    return Rt;
}*/

