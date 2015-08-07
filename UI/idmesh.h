#ifndef IDMESH_H
#define IDMESH_H

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
#include "TemplateWidget.h"
#include "AvatarviewMesh.h"
#include "FacePerformance/FacePerformance.h"
#include "Util/ConfigParameter.h"

class idmesh : public QGLViewer
{
    Q_OBJECT
public:
    idmesh(QWidget* parent=NULL);
    ~idmesh();

protected:
    virtual void draw();
    virtual void init();

public:
    PolyMesh  mesh;
    //double  scale;
    //int     v_size;
    //bool    is_draw_point_;
    //bool    is_draw_edge_;
    //bool    is_draw_face_;

signals:
    //void    operatorInfo(QString);
    //void    meshInfo(int ,int ,int);



public:
    void    ReadMesh(std::string filename);
    void    changeid(int id);
    double  scale;
    void    changescale(double dd);
    //void    WriteMesh();
    //void    CheckDrawPoint(bool bv);
    //void    CheckDrawEdge(bool bv);
    //void    CheckDrawFace(bool bv);

};

#endif
