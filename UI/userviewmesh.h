#ifndef USERVIEWMESH_H
#define USERVIEWMESH_H

#include <QGLViewer/qglviewer.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <ObjectTypes/TriangleMesh/TriangleMesh.hh>
#include <ObjectTypes/PolyMesh/PolyMesh.hh>
#include <vector>
#include <Eigen/Dense>
#include <QMutex>
#include "Util/ShowParameter.h"
#include "Util/TriMesh_Eigen.h"

enum DrawStyle{
    DrawVectices,
    DrawFaces,
    DrawEdges
};

class UserViewMesh : public QGLViewer
{
    Q_OBJECT
public:
    void setMutex_pointCloud(QMutex* mutex_pointCloud);
signals:

public slots:
    void getPointCloud(TriMesh_Eigen* depthMesh);//get point cloud from face

public:
    virtual void draw();
    virtual void init();

    int setMesh(std::string filename);
    PolyMesh mesh;
    TriMesh_Eigen* pointCloud;

private:
    DrawStyle drawStyle;
    QMutex* mutex_pointCloud;
public:
    Eigen::Vector3d translation_show;
    ShowConfig *showParas;
    QMutex* mutex_showParas;
    Eigen::MatrixXd *mesh_color;
    
    QMutex* mutex_isTracking;
    bool* isTracking;
};

#endif // USERVIEWMESH_H
