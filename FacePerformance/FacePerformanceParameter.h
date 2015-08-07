#ifndef FACEPERFORMANCEPARAMETER_H
#define FACEPERFORMANCEPARAMETER_H
#include <QtGui>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/CholmodSupport>
#include <ObjectTypes/PolyMesh/PolyMesh.hh>
#include "Util/TriMesh_Eigen.h"
#include "FaceProcessor/CRF/PoseEstimation.h"


typedef Eigen::SparseMatrix<double> SparseMatrixXd;
typedef Eigen::SparseVector<double> SparseVectorXd;
typedef Eigen::SparseMatrix<double, Eigen::RowMajor> SparseMatrixXd_RowMajor;

class InitRegisterParameter
{
public:
    //configure paras
    int maxInIters;
    int outIters;
    double max_accept_closet_dist;
    double registerRate;
    double markerWeight_RT;
    double markerWeight_yz0;
    double beta1;
    double beta2;
    double beta3;
    int num_P;
    int num_E;
    int radius_bilateral;
    double max_facerecognition_error;

    int num_r;
    int num_t;
    int num_y;
    int num_z0;
    int num_Rt;
    int num_yz0;
    int num_all;
    int start_r;
    int start_t;
    int start_y;
    int start_z0;

    std::vector<int> init_registerIndex;
    int init_registerIndex_size;
    std::vector<int> init_face_index;
    std::vector<int> fixed_registerIndex;
    Eigen::MatrixXd V_fixed;
    std::vector<int> jaw_registeIndex;

    Eigen::VectorXd weight_correspond;
    Eigen::MatrixXd temp_init_neutral;
    Eigen::MatrixXd init_mean_neutral;
    Eigen::MatrixXd init_P_eigenvectors_T;
    Eigen::MatrixXd init_E_eigenvectors_T;
    Eigen::MatrixXd Qp;
    Eigen::MatrixXd Qn;
    Eigen::MatrixXd V;
    Eigen::MatrixXd V_dary;
    Eigen::Matrix3Xd V_marker;

    //parameters for solving y, z0
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    Eigen::MatrixXd ATA;
    Eigen::MatrixXd nt_R_t;
    Eigen::MatrixXd R_m_t_c;
    Eigen::VectorXd diagonal;


#ifdef USE_KEYPOINT_INIT
    Eigen::Matrix3Xd source_kp;
    Eigen::Matrix3Xd target_kp;
#endif
};

class RigidRegisterParameter
{
public:
    //configure paras
    int maxInIters;
    int icpIters;
    double max_accept_closet_dist;
    double registerRate;
    double markerWeight;
    double max_nose_error;

    std::vector<int> init_registerIndex;
    int init_registerIndex_size;

    std::vector<int> init_face_index;
    std::vector<int> nose_face_index;

    //judge tracking succeed or not
    bool tracking_succeed;
    std::vector<int> nose_regionIndex;
    int nose_regionIndex_size;
    bool isNonrigidUseMarker;
    
    Eigen::Matrix3Xd Vp;
};

class NonRigidRegisterParameter
{
public:
    //configure paras
    int outIters;
    double max_accept_closet_dist;
    double registerRate;
    double colorWeight;
    double markerWeight;
    double alpha;
    double beta;

    std::vector<int> init_registerIndex;
    int init_registerIndex_size;
    std::vector<int> init_face_index;
    std::vector<int> fixed_registerIndex;
    bool * isOpticalIndex;

    int num_x;
    int num_frame;
    Eigen::VectorXd pre_frame_x;
    Eigen::VectorXd ppre_frame_x;

    Eigen::MatrixXd A; //lasso
    Eigen::VectorXd unchanged_b; //lasso

    //equation 2 E_fit
    Eigen::MatrixXd A_3d;
    Eigen::VectorXd b_3d;
    Eigen::MatrixXd A_2d;
    Eigen::VectorXd b_2d;

    Eigen::MatrixXi mapPoint2Image;

    Eigen::MatrixXd V; //register point in template
    Eigen::MatrixXd Qp; //nearest point in point cloud
    Eigen::MatrixXd Qn; //nearest point normal
    Eigen::VectorXd weight_coorespond;

    //Eigen::MatrixXd V_upsampling;
    SparseMatrixXd sp_upsampling;
    int init_registerIndex_upsampled_size;

    //Eigen::MatrixXd sR_init_deltaB_x;
    Eigen::MatrixXd init_upsampled_deltaB;
    Eigen::MatrixXd R_init_b0_Qp_T;
    Eigen::MatrixXd R_init_b0_V_T;

    Eigen::Matrix3d R;
    Eigen::MatrixXd R_init_b0;
    Eigen::MatrixXd nt_R;
    Eigen::MatrixXd nt_derive_Jf;
    Eigen::MatrixXd init_expression;

    Eigen::MatrixXd J;
    Eigen::MatrixXd derive_Jf;
    int m_channels;
};

class RefineMentParameter
{
public:
    ///configure paras
    double gamma;
    int MaxIter;
    double beta1;
    double beta2;
    double beta3;
    double sigma;
    Eigen::VectorXd sigma_blendshape;
    double sigma_b0;

    std::vector<int> init_registerIndex;
    int init_registerIndex_size;

    Eigen::VectorXd c;
    Eigen::MatrixXd Zi;

    std::vector<Eigen::MatrixXd > Ti_Star_P;
    std::vector<Eigen::MatrixXd > Ti_Star_E;
    std::vector<Eigen::MatrixXd > Ti_Star_m;
    Eigen::VectorXd diag;
    std::vector<int> update_blendshape_index;
    int update_blendshape_index_size;

    //different blendshape has different part of changing
    std::vector<Eigen::MatrixXd > part_Ti_Star_P_T;
    std::vector<Eigen::MatrixXd > part_Ti_Star_E_T;
    std::vector<Eigen::MatrixXd > part_Ti_Star_m;
    Eigen::MatrixXd init_Ti_Star_PE;

    //bool isInitRefinement;
    Eigen::MatrixXd update_Tib0;
    Eigen::MatrixXd update_Ezi;

    //different expressiong with different E
    std::vector<Eigen::MatrixXd > E_eigenvectors_T_gl;// each col a vector
    std::vector<Eigen::VectorXd > E_eigenvalues_gl;
    std::vector<int> start_index_E;
    std::vector<int> num_index_E;
    int num_all_E;
    std::vector<Eigen::MatrixXd > init_E_eigenvectors_T_gl; //after select
    std::vector<Eigen::MatrixXd > choose_E_eigenvectors_T_gl;

};
#ifdef EYETRACKER
class EyeParameter
{
public:
    const static int num_blendshape = 14;//14 eye blendshapes
    std::vector<std::vector<int> > part_T_Star_index;
    std::vector<SparseMatrixXd> GTGF;
    std::vector<SparseMatrixXd> GTHGF;
    Eigen::CholmodSimplicialLDLT<SparseMatrixXd> LDLT_solver[num_blendshape];
    Eigen::VectorXd x; //eye blenshapes weight, 14

    std::vector<Eigen::VectorXd> temp_x;

    std::vector<int> eye_keypoint_index_3d;
    std::vector< cv::Point2d > eye_keypoint_2d;
    Eigen::MatrixXd delta_B;
    Eigen::MatrixXd R_delta_B_eye_kepoint_part;
    Eigen::MatrixXd RT_expression_eye_keypoit_part;
};
#endif

class FacePerformanceParameter
{
public:

    ///common paras
    //mesh resolution: vertex(11510)
    const static int num_blendshape = 33;//47;
    const static int num_expression_blendshape = 33; //1 neutral, 32 other expressions
    const static int num_eye_blendshape = 14;
    Eigen::VectorXd mean_neutral;      //34530 * 1
    Eigen::MatrixXd P_eigenvectors_all;//34530 * 50
    Eigen::MatrixXd E_eigenvectors_all;//34530 * 49
    Eigen::VectorXd P_eigenvalues_all; //50
    Eigen::VectorXd E_eigenvalues_all; //49
    Eigen::MatrixXd P_eigenvectors;
    Eigen::MatrixXd E_eigenvectors;
    Eigen::MatrixXd P_eigenvectors_T;
    Eigen::MatrixXd E_eigenvectors_T;
    Eigen::VectorXd P_eigenvalues;
    Eigen::VectorXd E_eigenvalues;
    //Eigen::VectorXd srt;              //initregister
    Eigen::VectorXd y;
    Eigen::VectorXd z0;
    Eigen::VectorXd x;              //expression coeff, size 46
    SparseMatrixXd_RowMajor ExpressionTransform; // pre compute
    Eigen::VectorXd ExpressionNeutral;   // pre compute
    Eigen::VectorXd AllBlendshapes;
    Eigen::VectorXd Last_u;      //store last y,z0,z1,...,zn
    Eigen::VectorXd u;      //store y,z0,z1,...,zn

    ///double s;                       //scaling
    Eigen::Affine3d RT;             //Rotation and Translation
    Eigen::Affine3d RT_smooth;      //smooth the RT
    Eigen::MatrixXd expression;
    Eigen::MatrixXd RT_expression;     //3 * 11510
    Eigen::MatrixXd neutral_expression;
    Eigen::MatrixXd delta_B;

    //paras from nonrigid to refinement
    Eigen::MatrixXd A_3d_to_refinement;
    Eigen::MatrixXd A_2d_to_refinement;
    Eigen::VectorXd b_3d_to_refinement;
    Eigen::VectorXd b_2d_to_refinement;
    Eigen::VectorXd c_dary_to_refinement;

    std::vector<std::vector<int> > part_T_Star_index;


    //image and pointcloud
    double fx, fy;
    cv::Mat colorImage;
    cv::Mat grayImage;
    cv::Mat depthImage;
    Eigen::MatrixXi inDepthImage;
    cv::Mat colorImage240X320;

    //int depthImage_width, depthImage_height;
    //double maxDepth;
    TriMesh_Eigen* pointCloudMesh;
    std::vector<int> meshMarkerPointIndex;
    std::vector<int> marker_registerIndex;
    std::vector<int> marker_nearestIndex;
    std::vector<double> marker_Coordinates;
    Eigen::Matrix2Xd marker_Coordinates_all;

#ifdef FEATURE_WITH_EYE_CENTER
    Eigen::Matrix3Xd mesh_eyes_center;
    Eigen::Matrix3Xd pointcloud_eyes_center;
#endif

    cv::Rect depthImage_chosedRegion;
    Eigen::VectorXi isBackIndex;

    std::vector<SparseMatrixXd> GTGF;
    std::vector<SparseMatrixXd> GTHGF;
    Eigen::CholmodSimplicialLDLT<SparseMatrixXd> LDLT_solver[num_blendshape-1];
    Eigen::MatrixXd preframe_expression_blendshape_color;

    ///different paras
    InitRegisterParameter * init_paras;
    RigidRegisterParameter * rigid_paras;
    NonRigidRegisterParameter * nonrigid_paras;
    RefineMentParameter * refine_paras;
#ifdef EYETRACKER
    EyeParameter * eye_paras;
#endif

    PoseEstimation * poseEstimation;
    PolyMesh* expression_mesh;
    Eigen::VectorXd c_boundary_to_refinement;

    //smooth rotation
    std::vector<Eigen::Quaterniond > temp_rotation;
    std::vector<Eigen::VectorXd > temp_translation;
    double H_rotation;
    double H_translation;

    //save to database
    std::vector<cv::Mat> allColorImage;
    std::vector<cv::Mat> allDepthImage;
};

#endif // FACEPERFORMANCEPARAMETER_H
