///////////////////////////////////////////////////////////////////////////////
///   "Sparse Iterative Closest Point"
///   by Sofien Bouaziz, Andrea Tagliasacchi, Mark Pauly
///   Copyright (C) 2013  LGG, EPFL
///////////////////////////////////////////////////////////////////////////////
///   1) This file contains different implementations of the ICP algorithm.
///   2) This code requires EIGEN and NANOFLANN.
///   3) If OPENMP is activated some part of the code will be parallelized.
///   4) This code is for now designed for 3D registration
///   5) Two main input types are Eigen::Matrix3Xd or Eigen::Map<Eigen::Matrix3Xd>
///////////////////////////////////////////////////////////////////////////////
///   namespace nanoflann: NANOFLANN KD-tree adaptor for EIGEN
///   namespace RigidMotionEstimator: functions to compute the rigid motion
///   namespace SICP: sparse ICP implementation
///   namespace ICP: reweighted ICP implementation
///////////////////////////////////////////////////////////////////////////////
#ifndef SPARSEICP_H
#define SPARSEICP_H
//#include <nanoflann.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include "KDTreeAdaptor.h"
#include "ICP.h"
#include "FacePerformance/FacePerformanceFunction.h"
#include "FacePerformance/FacePerformanceParameter.h"
#include <Eigen/CholmodSupport>

///////////////////////////////////////////////////////////////////////////////
/// Compute the rigid motion for point-to-point and point-to-plane distances
namespace RigidMotionEstimator {

/// @param Source (one 3D point per column)
/// @param Target (one 3D point per column)
/// @param Target normals (one 3D normal per column)
/// @param Confidence weights
/// @param Right hand side
template <typename Derived1, typename Derived2, typename Derived3,
          typename Derived4, typename Derived5, typename Derived6,
          typename Derived7, typename Derived8>
Eigen::Affine3d point2plane_and_point2point(Eigen::MatrixBase<Derived1>& X,
                               Eigen::MatrixBase<Derived2>& Y,
                               Eigen::MatrixBase<Derived3>& N,
                               const Eigen::MatrixBase<Derived4>& w,
                               const Eigen::MatrixBase<Derived5>& u,
                               Eigen::MatrixBase<Derived6>& X2,
                               Eigen::MatrixBase<Derived7>& Y2,
                               const Eigen::MatrixBase<Derived8>& w2,
                               double fx, double fy, double rows, double cols) {
    typedef Eigen::Matrix<double, 6, 6> Matrix66;
    typedef Eigen::Matrix<double, 6, 1> Vector6;
    typedef Eigen::Block<Matrix66, 3, 3> Block33;
    typedef Eigen::Matrix<double, 2, 6> Matrix26;

    /// compute point-to-plane part
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

    ///compute the point-to-point part
    Matrix26 lhs;
    Eigen::Vector2d rhs;
    for(int i=0; i<X2.cols(); i++)
    {
        double k1 = -(Y2(0,i)/cols-0.5)*fx;
        double k2 = -(0.5-Y2(1,i)/rows)*fy;
        double x1 = X2(0, i);
        double x2 = X2(1, i);
        double x3 = X2(2, i);
        lhs(0,0) = -k1*x2;      lhs(1,0) = -x3 - k2*x2;
        lhs(0,1) = x3 + k1*x1;  lhs(1,1) = k2*x1;
        lhs(0,2) = -x2;         lhs(1,2) = x1;
        lhs(0,3) = 1.0;         lhs(1,3) = 0.0;
        lhs(0,4) = 0.0;         lhs(1,4) = 1.0;
        lhs(0,5) = -k1;         lhs(1,5) = -k2;
        rhs(0) = -x1 + k1*x3;   rhs(1) = -x2 + k2*x3;

        LHS.selfadjointView<Eigen::Upper>().rankUpdate(lhs.transpose(), w2(i));
        RHS += lhs.transpose() * rhs * w2(i);
    }
    LHS = LHS.selfadjointView<Eigen::Upper>();
    /// Compute transformation
    Eigen::Affine3d transformation;
    Eigen::LDLT<Matrix66> ldlt(LHS);
    //CholmodSimplicialLDLT<Matrix66> ldlt(LHS);
    RHS = ldlt.solve(RHS);
    transformation  = Eigen::AngleAxisd(RHS(0), Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(RHS(1), Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(RHS(2), Eigen::Vector3d::UnitZ());
    transformation.translation() = RHS.tail<3>();
    /// Apply transformation
    X = transformation*X;
    X2 = transformation*X2;

    return transformation;
}


    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Confidence weights
    template <typename Derived1, typename Derived2, typename Derived3>
    Eigen::Affine3d point_to_point(Eigen::MatrixBase<Derived1>& X,
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
    template <typename Derived1, typename Derived2>
    inline Eigen::Affine3d point_to_point(Eigen::MatrixBase<Derived1>& X,
                                          Eigen::MatrixBase<Derived2>& Y) {
        return point_to_point(X, Y, Eigen::VectorXd::Ones(X.cols()));
    }
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Confidence weights
    /// @param Right hand side
    template <typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5>
    Eigen::Affine3d point_to_plane(Eigen::MatrixBase<Derived1>& X,
                                   Eigen::MatrixBase<Derived2>& Y,
                                   Eigen::MatrixBase<Derived3>& N,
                                   const Eigen::MatrixBase<Derived4>& w,
                                   const Eigen::MatrixBase<Derived5>& u) {
        typedef Eigen::Matrix<double, 6, 6> Matrix66;
        typedef Eigen::Matrix<double, 6, 1> Vector6;
        typedef Eigen::Block<Matrix66, 3, 3> Block33;
        /// Normalize weight vector
        /// De-mean
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
        //CholmodSimplicialLDLT<Matrix66> ldlt(LHS);
        RHS = ldlt.solve(RHS);
        transformation  = Eigen::AngleAxisd(RHS(0), Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(RHS(1), Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(RHS(2), Eigen::Vector3d::UnitZ());
        transformation.translation() = RHS.tail<3>();
        /// Apply transformation
        X = transformation*X;
        /// Re-apply mean

        /// Return transformation
        return transformation;
    }

    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Confidence weights
    /// @param Right hand side
    template <typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5>
    Eigen::Affine3d point_to_plane_demean(Eigen::MatrixBase<Derived1>& X,
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
        X.colwise() -= X_mean;
        Y.colwise() -= X_mean;
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
        //CholmodSimplicialLDLT<Matrix66> ldlt(LHS);
        RHS = ldlt.solve(RHS);
        transformation  = Eigen::AngleAxisd(RHS(0), Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(RHS(1), Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(RHS(2), Eigen::Vector3d::UnitZ());
        transformation.translation() = RHS.tail<3>();
        /// Apply transformation
        X = transformation*X;
        /// Re-apply mean
        X.colwise() += X_mean;
        Y.colwise() += X_mean;
        /// Return transformation
        transformation.translation() += (X_mean - transformation.rotation()*X_mean);

        return transformation;

//        Eigen::Affine3d inv_transformation;
//        inv_transformation = Eigen::AngleAxisd(-RHS(2), Eigen::Vector3d::UnitZ()) *
//                Eigen::AngleAxisd(-RHS(1), Eigen::Vector3d::UnitY()) *
//                Eigen::AngleAxisd(-RHS(0), Eigen::Vector3d::UnitX());
//        inv_transformation.translation() = -inv_transformation.rotation()*transformation.translation();
//        return inv_transformation;

    }

    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Confidence weights
    /// @param Right hand side
    template <typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5>
    Eigen::Affine3d point_to_plane_demean_inv(Eigen::MatrixBase<Derived1>& X,
                                   Eigen::MatrixBase<Derived2>& Y,
                                   Eigen::MatrixBase<Derived3>& N,
                                   const Eigen::MatrixBase<Derived4>& w,
                                   const Eigen::MatrixBase<Derived5>& u,
                                              Eigen::Affine3d& affineTransform) {
        typedef Eigen::Matrix<double, 6, 6> Matrix66;
        typedef Eigen::Matrix<double, 6, 1> Vector6;
        typedef Eigen::Block<Matrix66, 3, 3> Block33;
        /// Normalize weight vector
        Eigen::VectorXd w_normalized = w/w.sum();
        /// De-mean
        Eigen::Vector3d X_mean;
        for(int i=0; i<3; ++i)
            X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();
        X.colwise() -= X_mean;
        Y.colwise() -= X_mean;
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
        //CholmodSimplicialLDLT<Matrix66> ldlt(LHS);
        RHS = ldlt.solve(RHS);
        transformation  = Eigen::AngleAxisd(RHS(0), Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(RHS(1), Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(RHS(2), Eigen::Vector3d::UnitZ());
        transformation.translation() = RHS.tail<3>();
        /// Apply transformation
        X = transformation*X;
        /// Re-apply mean
        X.colwise() += X_mean;
        Y.colwise() += X_mean;
        /// Return transformation
        transformation.translation() += (X_mean - transformation.rotation()*X_mean);

        affineTransform = transformation;

        Eigen::Affine3d inv_transformation;
        inv_transformation = Eigen::AngleAxisd(-RHS(2), Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(-RHS(1), Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(-RHS(0), Eigen::Vector3d::UnitX());
        inv_transformation.translation() = -inv_transformation.rotation() * transformation.translation();
        return inv_transformation;

//        Eigen::Affine3d inv_transformation;
//        inv_transformation = Eigen::AngleAxisd(-RHS(2), Eigen::Vector3d::UnitZ()) *
//                Eigen::AngleAxisd(-RHS(1), Eigen::Vector3d::UnitY()) *
//                Eigen::AngleAxisd(-RHS(0), Eigen::Vector3d::UnitX());
//        inv_transformation.translation() = -inv_transformation.rotation()*transformation.translation();
//        return inv_transformation;

    }

    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Confidence weights
    template <typename Derived1, typename Derived2, typename Derived3, typename Derived4>
    inline Eigen::Affine3d point_to_plane(Eigen::MatrixBase<Derived1>& X,
                                          Eigen::MatrixBase<Derived2>& Yp,
                                          Eigen::MatrixBase<Derived3>& Yn,
                                          const Eigen::MatrixBase<Derived4>& w) {
        return point_to_plane(X, Yp, Yn, w, Eigen::VectorXd::Zero(X.cols()));
    }
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Confidence weights
    template <typename Derived1, typename Derived2, typename Derived3, typename Derived4>
    inline Eigen::Affine3d point_to_plane_demean(Eigen::MatrixBase<Derived1>& X,
                                          Eigen::MatrixBase<Derived2>& Yp,
                                          Eigen::MatrixBase<Derived3>& Yn,
                                          const Eigen::MatrixBase<Derived4>& w) {
        return point_to_plane_demean(X, Yp, Yn, w, Eigen::VectorXd::Zero(X.cols()));
    }

}

///////////////////////////////////////////////////////////////////////////////
/// ICP implementation using iterative reweighting
namespace ICP {
    /// Reweighted ICP with point to point
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Parameters
    template <typename Derived1, typename Derived2>
    void point_to_point(Eigen::MatrixBase<Derived1>& X,
                        Eigen::MatrixBase<Derived2>& Y,
                        Parameters par = Parameters()) {
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
    Eigen::Affine3d point_to_plane(
                        Eigen::MatrixBase<Derived1>& X,
                        Eigen::MatrixBase<Derived2>& Y,
                        Eigen::MatrixBase<Derived3>& N,
                        Parameters par = Parameters()) {
        /// Build kd-tree
        nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);
        /// Buffers
        Eigen::Matrix3Xd Qp = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::Matrix3Xd Qn = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::VectorXd W = Eigen::VectorXd::Zero(X.cols());
        Eigen::Matrix3Xd XLast = X;
        /// ICP
        Eigen::Affine3d Rt;
        Rt.setIdentity();
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
            W = (Qn.array()*(X-Qp).array()).colwise().sum().abs().transpose();
            robust_weight(par.f, W, par.p);
            /// Rotation and translation update
            Eigen::Affine3d Rt1 = RigidMotionEstimator::point_to_plane(X, Qp, Qn, W);
            Rt = Rt1 * Rt;

            /// Stopping criteria
            double stop2 = (X-XLast).colwise().norm().maxCoeff() ;
            XLast = X;
            if(stop2 < par.stop) break;
        }
        return Rt;
    }

    template <typename Derived1>
    inline static Eigen::Vector2i ConvertRealWorldToProjective(const Eigen::MatrixBase<Derived1>& realWorldPoint, const cv::Mat& colorImage, double fx, double fy)
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
    /// Reweighted ICP with point to plane projection
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Parameters
    template <typename Derived1, typename Derived2, typename Derived3>//, typename Derived4>
    Eigen::Affine3d point_to_plane(
                        Eigen::MatrixBase<Derived1>& X,
                        Eigen::MatrixBase<Derived2>& Y,
                        Eigen::MatrixBase<Derived3>& N,
                        cv::Mat& colorImage,
                        Eigen::MatrixXi* inDepthImage,
                        double fx,
                        double fy,
                        Parameters par = Parameters()) {
        /// Buffers
        Eigen::Matrix3Xd Qp(3, X.cols());
        Eigen::Matrix3Xd Qn(3, X.cols());
        Eigen::VectorXd W(X.cols());

        /// ICP
        Eigen::Affine3d Rt;
        Rt.setIdentity();

        for(int icpIter = 0; icpIter < par.max_icp; ++icpIter)
        {
            /// Find closest point
            Eigen::Matrix2Xd coors = FacePerformanceFunction::ConvertRealWorldToProjective_Matrix(X, colorImage, fx, fy);
            #pragma omp parallel for
            for(int i=0; i<X.cols(); ++i)
            {
                int id = inDepthImage->coeff((int)coors(1,i), (int)coors(0,i));
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
                W = (Qn.array()*(X-Qp).array()).colwise().sum().abs().transpose();
                robust_weight(par.f, W, par.p);

                /// Rotation and translation update
                Rt = RigidMotionEstimator::point_to_plane(X, Qp, Qn, W) * Rt;
            }

        }
        return Rt;
    }    /// Reweighted ICP with point to plane projection
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Parameters
    template <typename Derived1, typename Derived2, typename Derived3, typename Derived4>
    Eigen::Affine3d point_to_plane(
                        Eigen::MatrixBase<Derived1>& X,
                        Eigen::MatrixBase<Derived2>& Xn,
                        Eigen::MatrixBase<Derived3>& Y,
                        Eigen::MatrixBase<Derived4>& N,
                        cv::Mat& colorImage,
                        Eigen::MatrixXi* inDepthImage,
                        double fx,
                        double fy,
                        Parameters par = Parameters()) {
        // Buffers
        Eigen::Matrix3Xd Qp(3, X.cols());
        Eigen::Matrix3Xd Qn(3, X.cols());
        Eigen::VectorXd W(X.cols());
        // ICP
        Eigen::Affine3d Rt;
        Rt.setIdentity();

        for (int icpIter = 0; icpIter < par.max_icp; ++icpIter)
        {
            // Find closest point
            Eigen::Matrix2Xd coors = FacePerformanceFunction::ConvertRealWorldToProjective_Matrix(X, colorImage, fx, fy);
            #pragma omp parallel for
            for(int i=0; i<X.cols(); ++i)
            {
                int id = inDepthImage->coeff((int)coors(1,i), (int)coors(0,i));
                if (id != -1)
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
            // Computer rotation and translation
            for (int outer=0; outer<par.max_outer; ++outer)
            {
                // Compute weights
                W = (Qn.array()*(X-Qp).array()).colwise().sum().abs().transpose();
                robust_weight(par.f, W, par.p);
                for (int i = 0; i < X.cols(); ++i)
                {
                    if (std::fabs(Qn.col(i).dot(Xn.col(i))) < 0.2)
                    {
                        W[i] = 0;
                    }
                }
                // Rotation and translation update
                Eigen::Affine3d tmp = RigidMotionEstimator::point_to_plane(X, Qp, Qn, W);
                Xn = tmp.rotation()*Xn;
                Rt = tmp*Rt;
            }
        }
        return Rt;
    }

    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Parameters
    template <typename Derived1, typename Derived2, typename Derived3, typename Derived4>
    Eigen::Affine3d point_to_plane(
                        Eigen::MatrixBase<Derived1>& X,
                        Eigen::MatrixBase<Derived2>& Xn,
                        Eigen::MatrixBase<Derived3>& Y,
                        Eigen::MatrixBase<Derived4>& N,
                        std::vector<int>& init_face_index,
                        cv::Mat& colorImage,
                        Eigen::MatrixXi* inDepthImage,
                        double fx,
                        double fy,
                        Parameters par = Parameters()) {
        // Buffers
        Eigen::Matrix3Xd Qp(3, X.cols());
        Eigen::Matrix3Xd Qn(3, X.cols());
        Eigen::VectorXd W(X.cols());
        // ICP
        Eigen::Affine3d Rt;
        Rt.setIdentity();

        for (int icpIter = 0; icpIter < par.max_icp; ++icpIter)
        {
            /// Find closest point
            Eigen::Matrix2Xd coors = FacePerformanceFunction::ConvertRealWorldToProjective_Matrix(X, colorImage, fx, fy);
            #pragma omp parallel for
            for(int i=0; i<X.cols(); ++i)
            {
                int id = inDepthImage->coeff((int)coors(1,i), (int)coors(0,i));
                if (id != -1)
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
            /// Compute weights
            W = (Qn.array()*(X-Qp).array()).colwise().sum().abs().transpose();
            robust_weight(par.f, W, par.p);

            for (int i = 0; i < X.cols(); ++i)
            {
                if (std::fabs(Qn.col(i).dot(Xn.col(i))) < 0.866)
                {
                    W[i] = 0;
                }
            }
            //reject the inv face vertices
            Eigen::MatrixXd M33(3,3);
            for (int i = 0; i < init_face_index.size(); i += 4)
            {
                int i0 = init_face_index[i];
                int i1 = init_face_index[i+1];
                int i2 = init_face_index[i+2];
                int i3 = init_face_index[i+3];
                M33.col(0) = X.col(i0);
                M33.col(1) = X.col(i1);
                M33.col(2) = X.col(i2);
                if (M33.determinant() >= 0)
                {
                    W[i0] = 0.0;
                    W[i1] = 0.0;
                    W[i2] = 0.0;
                    W[i3] = 0.0;
                }
            }
            if (W.sum() < 100)
                break;

            /// Rotation and translation update
            Eigen::Affine3d Rt1 = RigidMotionEstimator::point_to_plane_demean(X, Qp, Qn, W);
            Xn = Rt1.rotation()*Xn;
            Rt = Rt1 * Rt;
        }
        return Rt;
    }

    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Parameters
    template <typename Derived1, typename Derived2, typename Derived3,
              typename Derived4, typename Derived5, typename Derived6>
    Eigen::Affine3d point2plane_and_point2point(
                        Eigen::MatrixBase<Derived1>& X,
                        Eigen::MatrixBase<Derived2>& Xn,
                        Eigen::MatrixBase<Derived3>& Y,
                        Eigen::MatrixBase<Derived4>& N,
                        Eigen::MatrixBase<Derived5>& X2,
                        Eigen::MatrixBase<Derived6>& Y2,
                        double weight_marker,
                        std::vector<int>& init_face_index,
                        cv::Mat& colorImage,
                        Eigen::MatrixXi* inDepthImage,
                        double fx,
                        double fy,
                        Parameters par = Parameters()) {
        // Buffers
        Eigen::Matrix3Xd Qp(3, X.cols());
        Eigen::Matrix3Xd Qn(3, X.cols());
        Eigen::VectorXd W(X.cols());
        // ICP
        Eigen::Affine3d Rt;
        Rt.setIdentity();

        for (int icpIter = 0; icpIter < par.max_icp; ++icpIter)
        {
            /// Find closest point
            Eigen::Matrix2Xd coors = FacePerformanceFunction::ConvertRealWorldToProjective_Matrix(X, colorImage, fx, fy);
            #pragma omp parallel for
            for (int i = 0; i < X.cols(); ++i)
            {
                int id = inDepthImage->coeff((int)coors(1,i), (int)coors(0,i));
                if (id != -1)
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
            /// Compute weights
            W = (Qn.array()*(X-Qp).array()).colwise().sum().abs().transpose();
            robust_weight(par.f, W, par.p);

            #pragma omp parallel for
            for (int i = 0; i < X.cols(); ++i)
            {
                if (std::fabs(Qn.col(i).dot(Xn.col(i))) < 0.866)
                {
                    W[i] = 0;
                }
            }
            //reject the inv face vertices
            Eigen::MatrixXd M33(3,3);
            for (int i = 0; i < init_face_index.size(); i += 4)
            {
                int i0 = init_face_index[i];
                int i1 = init_face_index[i+1];
                int i2 = init_face_index[i+2];
                int i3 = init_face_index[i+3];
                M33.col(0) = X.col(i0);
                M33.col(1) = X.col(i1);
                M33.col(2) = X.col(i2);
                if (M33.determinant() >= 0)
                {
                    W[i0] = 0.0;
                    W[i1] = 0.0;
                    W[i2] = 0.0;
                    W[i3] = 0.0;
                }
            }
            if (W.sum() < 100)
                break;

            /// Rotation and translation update
            Eigen::Affine3d Rt1 = RigidMotionEstimator::point2plane_and_point2point(X, Qp, Qn, W, Eigen::VectorXd::Zero(X.cols()),
                                                                                    X2, Y2, Eigen::VectorXd::Ones(X2.cols())*weight_marker,
                                                                                    fx, fy, (double)colorImage.rows, (double)colorImage.cols);
            Xn = Rt1.rotation()*Xn;
            Rt = Rt1 * Rt;
        }
        return Rt;
    }

}

///////////////////////////////////////////////////////////////////////////////
#endif







/*

*/
