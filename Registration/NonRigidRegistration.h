#ifndef NONRIGIDREGISTER_H_INCLUDED
#define NONRIGIDREGISTER_H_INCLUDED
#include <vector>
#include <string>
#include <map>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <algorithm>
#include <math.h>
#include <Eigen/Dense>
#include <fstream>
#include <QString>
#include "qdebug.h"
#include "ICP/ICP.h"
#include "ICP/KDTreeAdaptor.h"
#include "FacePerformance/FacePerformanceFunction.h"
#include "FacePerformance/FacePerformanceParameter.h"
class NonRigidRegister
{
public:
    NonRigidRegister(FacePerformanceParameter* face_paras);
    ~NonRigidRegister();
    void FitMeshLasso();
    void AfterFit();
    void Compute_A_b_to_refinement();

private:
    void Init();
    void UpdateRtExpression();
    void UpdateAll();
    void FindNearestKdtree();
    void FindNearestProjection();
    void Compute_A_b();
#ifdef EYETRACKER
    void TrackEye();
#endif
    void Lasso(Eigen::MatrixXd& A, Eigen::VectorXd & b, double beta, Eigen::VectorXd& x, double tolerance = 1.0e-3);
    void Shotgun(Eigen::MatrixXd& A, Eigen::VectorXd & b, double beta, Eigen::VectorXd& x, double tolerance = 1.0e-3);
private:
    FacePerformanceParameter* face_paras;
    NonRigidRegisterParameter* paras;
#ifdef EYETRACKER
    EyeParameter* eye_paras;
#endif
    std::ofstream of;
    double update_expression_cost;
    double computer_A_b_cost;
    double find_nearest_cost;
    double find_nearest_precompute_cost;
    double find_nearest_project_cost;
    double lasso_cost;

    double _3d_cost;
    double _3d_basis_;
    double _2d_cost;
    double _2d_basis_;
    double smooth_cost;
    double marker_cost;

};
#endif
