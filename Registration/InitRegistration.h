#ifndef INITREGISTER_HH_INCLUDED
#define INITREGISTER_HH_INCLUDED
#include <vector>
#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <algorithm>
#include <math.h>
#include <Eigen/Dense>
#include <fstream>
#include <limits>
#include "FacePerformance/FacePerformanceFunction.h"
#include "ICP/ICP.h"
#include "ICP/KDTreeAdaptor.h"
#include "FacePerformance/FacePerformanceFunction.h"
#include "FaceProcessor/CRF/PoseEstimation.h"
#include "ICP/SparseICP.h"
#include "Util/ConfigParameter.h"

class InitRegister
{
public:
    InitRegister(FacePerformanceParameter* paras);
    ~InitRegister();
    void FitMesh();
    void Blendshape2Trimesh(TriMesh_Eigen& tmesh);
    static void LoadEigenMatrix(Eigen::MatrixXd& mat, std::string filename);
    static void LoadEigenVector(Eigen::VectorXd& vec, std::string filename);
    static void LoadEigenMatrix_gl(Eigen::MatrixXd& mat, std::string filename);
    static void LoadEigenVector_gl(Eigen::VectorXd& vec, std::string filename);

private:
    void InitRt_yz0();
    void UpdateRtNeutral();
    void UpdateRegisteredRtNeutral();
    void Update_yz0();
    void FindNearestKdtree();
    void FindNearestProjection();

private:
    FacePerformanceParameter* face_paras;
    InitRegisterParameter* paras;
    Eigen::VectorXd yz0;
    std::ofstream of;
};
#endif
