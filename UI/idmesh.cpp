#include "idmesh.h"
#include "qdebug.h"

idmesh::idmesh(QWidget* parent): QGLViewer(parent)
{
}

void idmesh::changeid(int id)
{
    std::string dir_face_database = ConfigParameter::facedatabaseDir;
    PolyMesh tmpmesh;
    OpenMesh::IO::read_mesh(tmpmesh,dir_face_database + QString::number(id).toStdString() + "_neutral.obj");

    auto vv_it=tmpmesh.vertices_begin();
    for(auto v_it=mesh.vertices_begin();v_it!=mesh.vertices_end();v_it++)
    {
        mesh.point(v_it.handle()).data()[0]=tmpmesh.point(vv_it.handle()).data()[0];
        mesh.point(v_it.handle()).data()[1]=tmpmesh.point(vv_it.handle()).data()[1];
        mesh.point(v_it.handle()).data()[2]=tmpmesh.point(vv_it.handle()).data()[2];
        vv_it++;
    }
}


idmesh::~idmesh()
{
}

/*
double calscale(PolyMesh mesh)
{
    double min;
    double max;
    double tmp;
    auto v_it=mesh.vertices_begin();
    min=mesh.point(v_it.handle()).data()[0];
    max=mesh.point(v_it.handle()).data()[0];

    for(v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it)
    {
        for(int i=0;i<2;i++)
        {
            tmp=mesh.point(v_it.handle()).data()[i];
            if(min>tmp)
            {
                min=tmp;
            }
            if(max<tmp)
            {
                max=tmp;
            }
        }
    }



    if(fabs(min)>fabs(max))
    {
        return fabs(min);
    }
    else
    {
        return fabs(max);
    }

}
*/

void idmesh::draw()
{
    //const double scale=150;
    auto v_it=mesh.vertices_begin();
    glEnable(GL_LIGHTING);
    glDisable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,GL_TRUE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_NORMALIZE);
    glColor3f(0.9f,0.9f,1.f);

    mesh.request_face_normals();
    mesh.request_vertex_normals();
    mesh.update_normals();

    for(auto it = mesh.faces_begin(); it != mesh.faces_end(); it++)
    {
        auto face =it.handle();
        glBegin(GL_POLYGON);
        for(auto v_it =mesh.fv_begin(face);v_it != mesh.fv_end(face); v_it++)
        {
            glNormal3d(mesh.normal(v_it.handle()).data()[0],mesh.normal(v_it.handle()).data()[1],mesh.normal(v_it.handle()).data()[2]);
            glVertex3d(mesh.point(v_it.handle()).data()[0]/scale,mesh.point(v_it.handle()).data()[1]/scale,mesh.point(v_it.handle()).data()[2]/scale);

        }
           glEnd();
    }
}

void idmesh::init()
{
    scale=150;

    //is_draw_edge_=true;
    //is_draw_face_=true;
    //is_draw_point_=true;
    this->setBackgroundColor(QColor(96,98,90));
    float defaultPosition[4] = {0.0,0.0,0.0,1.0};
    float defaultSpotDirection[3] = {0.0,0.0,1.0};

    float defaultAmbientColor[4] = {0.005f,0.005f,0.005f,1.f};
    float defaultDiffuseColor[4] = {0.9f,0.9f,0.9f,1.f};
    float defaultSpecularColor[4] = {0.9f,0.9f,0.9f,1.f};

    glShadeModel(GL_SMOOTH);
    glEnable(GL_LIGHTING);

    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,defaultAmbientColor);
    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,defaultDiffuseColor);
    glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,defaultSpecularColor);
    glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,200.f);

    glLightfv(GL_LIGHT0, GL_POSITION, defaultPosition);
    glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, defaultSpotDirection);

    glLightfv(GL_LIGHT0, GL_AMBIENT, defaultAmbientColor);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, defaultDiffuseColor);
    glLightfv(GL_LIGHT0, GL_SPECULAR, defaultSpecularColor);

    glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, 0.f);
    glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 120.f);
    GLfloat mat_specular[4] = { 0.18f, 0.18f, 0.18f, 1.f };
    GLfloat mat_shininess[] = { 64.f };
    GLfloat global_ambient[] = { 0.05f, 0.05f, 0.05f, 1.f };
    GLfloat light1_ambient[] = { 0.0f, 0.0f, 0.0f, 1.f };
    GLfloat light1_diffuse[] = { 0.9f, 0.9f, 0.9f, 1.f };
    GLfloat light1_specular[] = { 0.85f, 0.85f, 0.85f, 1.f };

    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
    glLightfv(GL_LIGHT1, GL_AMBIENT, light1_ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, light1_specular);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_ambient);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
    glLightf(GL_LIGHT1, GL_SPOT_EXPONENT, 0.0);
}

void idmesh::ReadMesh(std::string filename)
{
    OpenMesh::IO::read_mesh(mesh,filename);
    //scale=calscale(mesh);
    //v_size=mesh.n_vertices();
}

void    idmesh::changescale(double dd)
{
    scale=dd;
}

