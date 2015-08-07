#ifndef AVATARVIEWMESH_H
#define AVATARVIEWMESH_H

#include <QGLViewer/qglviewer.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <ObjectTypes/TriangleMesh/TriangleMesh.hh>
#include <ObjectTypes/PolyMesh/PolyMesh.hh>
#include <vector>
#include <Eigen/Dense>
#include "qimage.h"
#include <QMutex>
#include <fstream>

class AvatarViewMesh : public QGLViewer
{
    Q_OBJECT
public:
    AvatarViewMesh();
    ~AvatarViewMesh();
    virtual void draw();
    virtual void init();

    void getBaseModel( std::string pathOfObj );
    void loadBaseModel(const std::string& pathOfObj, PolyMesh& mesh, Eigen::MatrixXd& modelOfFaceShift);
    void loadAllAvatars(const std::string& all_avatars_model_dir);


    std::vector<Eigen::MatrixXd > avatar_models;
    PolyMesh avatar_meshes[3];
    int cur_mesh_index;

    Eigen::MatrixXd modelOfFaceShift;
private:
    PolyMesh mesh;
    //std::string pathOfObj;
    std::vector<QImage> textures;
    std::vector<GLuint> TextureIDs;

public:
    QMutex * mutex_x;
    QMutex * mutex_isTracking;
    bool* isTracking;

    Eigen::VectorXd x;
    Eigen::MatrixXd R;
    Eigen::VectorXd T;
    Eigen::Vector3d translation_show;

signals:

public slots:
    void updateX(Eigen::VectorXd *x);
    void updateMesh();
};

#endif // AVATARVIEWMESH_H
