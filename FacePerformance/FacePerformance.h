#ifndef FACEPERFORMANCE_HH_INCLUDED
#define FACEPERFORMANCE_HH_INCLUDED

#include <QtGui>
#include <QLabel>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <ObjectTypes/PolyMesh/PolyMesh.hh>
#include <ObjectTypes/TriangleMesh/TriangleMesh.hh>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/CholmodSupport>
#include <fstream>
#include <vector>
#include <string>
#include "FaceProcessor/FeatureExtraction.h"
#include "Registration/InitRegistration.h"
#include "Registration/RigidRegistration.h"
#include "Registration/NonRigidRegistration.h"
#include <time.h>
#include "Registration/RefineMent.h"
#include "UI/userviewmesh.h"
#include "UI/avatarviewmesh.h"
#ifdef USE_OPENNI1
    #include <XnCppWrapper.h> //use openni1
#else
    #include <OpenNI.h> //use openni2
#endif
#include "Util/ShowParameter.h"
#include "Util/TriMesh_Eigen.h"
#include "FacePerformanceParameter.h"
#include "FacePerformanceFunction.h"
#include "FaceProcessor/CRF/PoseEstimation.h"
#include <omp.h>
#include "Util/ShowThread.h"
#include "qdebug.h"
#include "QSettings"
#include "Util/ConfigParameter.h"

typedef Eigen::SparseMatrix<double> SparseMatrixXd;
typedef Eigen::Triplet<double> T;

class FacePerformance : public QObject
{
    Q_OBJECT

signals:
    void UpdateView();
    void UpdateAvatar();
    void ShowColorImage(uchar *, int ,int );
public :
    FacePerformance();
    QString Name() { return QString("FacePerformance"); }
    ~FacePerformance(){}

    void LoadFaceParameters();
    void LoadInitRegisterParameters();
    void LoadRigidRegisterParameters();
    void LoadNonrigidRegisterParameters();
    void LoadRefinementParameters();
    void LoadAllParameters();
    void LoadTStar();
    void ComputeDeltaBByNeutral();
    void ComputeBlendShapeByNeutral(MyTriMesh* neutral_mesh, int id_mesh,
                                    Eigen::CholmodSimplicialLDLT<SparseMatrixXd>& LDLT_solver,
                                    const SparseMatrixXd& GTHGF);
public:
    unsigned short maxDepth_openni;

#ifdef USE_OPENNI1
    //openni1
    XnStatus result_openni;
    xn::DepthMetaData depthMD;
    xn::ImageMetaData imageMD;
    xn::Context context;
    xn::DepthGenerator depthGenerator;
    xn::ImageGenerator imageGenerator;
    XnMapOutputMode mapMode;
#else
    //openni2
    openni::Status result_openni2;
    openni::VideoFrameRef oniDepthImg;
    openni::VideoFrameRef oniColorImg;
    openni::Device oniDevice;
    openni::VideoStream oniDepthStream;
    openni::VideoStream oniColorStream;
    openni::VideoMode oniDepthMode;
    openni::VideoMode oniColorMode;
#endif
    //face processor
    FeatureExtraction* featureExtraction;
    PoseEstimation * poseEstimation;

    FacePerformanceParameter faceParas;
    InitRegisterParameter initParas;
    RigidRegisterParameter rigidParas;
    NonRigidRegisterParameter nonrigidParas;
    RefineMentParameter refineParas;
    InitRegister* initRegister;
    RigidRegister* rigidRegister;
    NonRigidRegister* nonrigidRegister;
    RefineMent* refineMent;
#ifdef EYETRACKER
    EyeParameter eyeParas;
#endif
    std::ofstream of;

private slots:

public:
    void InitializePlugin();
public slots:
    void OpenSensor();
    bool ReadFrame();
    void StartTrackThread();
    void TrackFace();
    void InitSystem();
    void UpdateParas();

public:
    QMutex mutex_isInitFrame;
    bool isInitFrame;
    QMutex mutex_isTracking;
    bool isTracking;

    //for showing
    QLabel* userImage;
    UserViewMesh* userViewMesh;
    AvatarViewMesh* avatarViewMesh;
    QLabel* recognizedImage;

    QMutex mutex_pointCloud;
    QMutex mutex_x;

    QMutex mutex_colorImage;
    QMutex mutex_paras;
    QMutex mutex_recognition;
    ShowConfig showParas;
    QMutex mutex_show;

    TriMesh_Eigen depthMesh;
    PolyMesh * expression_mesh;
    Eigen::MatrixXd mesh_color;

    QMutex mutex_allImage;

};
#endif
