#include "UserviewMesh.h"
#include "qdebug.h"

// Draws seem like the paintEvent in QWidget.
void UserViewMesh::draw()
{
    mutex_isTracking->lock();
    if(!(*isTracking))
    {
        mutex_isTracking->unlock();
        return;
    }
    mutex_isTracking->unlock();

    const double mesh_scale = 175.0;
    mutex_pointCloud->lock();


    mutex_showParas->lock();
    int i=0;
    int j=0;

    switch(showParas->faceMeshFormat)
    {
    case FACES_FACEMESH:
        //draw faces
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glDisable(GL_LIGHT1);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glEnable(GL_POLYGON_OFFSET_FILL);

        for (PolyMesh::FaceIter faceIt = mesh.faces_begin(); faceIt != mesh.faces_end(); ++faceIt)
        {
            glBegin(GL_POLYGON);
            glColor3d(240.0/255.0, 240.0/255.0, 240.0/255.0);
            //Get the vertices of the point
            for (PolyMesh::FaceVertexIter vertexIt = mesh.fv_begin(faceIt); vertexIt != mesh.fv_end(faceIt); ++vertexIt)
            {
                PolyMesh::Normal normal = mesh.normal(vertexIt.handle());
                PolyMesh::Point point = mesh.point(vertexIt.handle());
                point[0] += translation_show(0);
                point[0] /= mesh_scale;
                point[1] += translation_show(1);
                point[1] /= mesh_scale;
                point[2] += translation_show(2);
                point[2] /= mesh_scale;
                glNormal3dv(&normal[0]);
                glVertex3dv(&point[0]);
            }
            glEnd();
        }

        glDisable(GL_LIGHTING);  // Draw polygon outlines with lighting disabled.
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);  // Draw polygon outlines.

        glColor3f(0.1,0.1,0.1);  // Draw the outlines in black.
        glLineWidth(1.0f);
        glPolygonOffset(1.0, 1.0);
        for (PolyMesh::FaceIter faceIt = mesh.faces_begin(); faceIt != mesh.faces_end(); ++faceIt)
        {
            glBegin(GL_POLYGON);
            //Get the vertices of the point
            for (PolyMesh::FaceVertexIter vertexIt = mesh.fv_begin(faceIt); vertexIt != mesh.fv_end(faceIt); ++vertexIt)
            {
                PolyMesh::Normal normal = mesh.normal(vertexIt.handle());
                PolyMesh::Point point = mesh.point(vertexIt.handle());
                point[0] += translation_show(0);
                point[0] /= mesh_scale;
                point[1] += translation_show(1);
                point[1] /= mesh_scale;
                point[2] += translation_show(2);
                point[2] /= mesh_scale;
                glNormal3dv(&normal[0]);
                glVertex3dv(&point[0]);
            }

            glEnd();
        }
        glEnable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        break;
    case TEXTURE_FACEMESH:

        for (PolyMesh::FaceIter faceIt = mesh.faces_begin(); faceIt != mesh.faces_end(); ++faceIt)
        {
            glBegin(GL_POLYGON);
            //Get the vertices of the point
            for (PolyMesh::FaceVertexIter vertexIt = mesh.fv_begin(faceIt); vertexIt != mesh.fv_end(faceIt); ++vertexIt)
            {
                PolyMesh::Normal normal = mesh.normal(vertexIt.handle());
                PolyMesh::Point point = mesh.point(vertexIt.handle());
                int idx = vertexIt.handle().idx();
                point[0] += translation_show(0);
                point[0] /= mesh_scale;
                point[1] += translation_show(1);
                point[1] /= mesh_scale;
                point[2] += translation_show(2);
                point[2] /= mesh_scale;

                glNormal3dv(&normal[0]);
                glVertex3dv(&point[0]);
                if(mesh_color->cols() != 0)
                    glColor3d(mesh_color->coeff(0, idx), mesh_color->coeff(1, idx), mesh_color->coeff(2, idx));
            }
            glEnd();
        }
        break;
    }

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    switch(showParas->pointCloudFormat)
    {
    case FACES_POINTCLOUD:
        if(pointCloud != NULL)
        {
            glEnable(GL_LIGHTING);
            glDisable(GL_LIGHT0);
            glEnable(GL_LIGHT1);
            glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,GL_TRUE);
            glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);// todo include this into spotlight node

            glEnable(GL_COLOR_MATERIAL);
            glEnable(GL_NORMALIZE);

            Eigen::Matrix3Xd show_vertices = pointCloud->vertices.colwise() + translation_show;
            show_vertices /= mesh_scale;
            glColor3f(0.9f,0.9f,1.f);
            glBegin(GL_TRIANGLES);
            for(int i=0; i<pointCloud->faces.cols(); i++)
            {
                for(int j=pointCloud->faces.rows()-1; j>=0; j--)
                {
                    int index = pointCloud->faces(j,i);
                    glVertex3dv(show_vertices.col(index).data());
                    glNormal3dv(pointCloud->normals.col(index).data());
                }
            }
            glEnd();
            glColor3d(1.0,1.0,1.0);
        }
        break;

    case VERTICES_POINTCLOUD:
        if(pointCloud != NULL)
        {
            glEnable(GL_LIGHTING);
            glDisable(GL_LIGHT0);
            glEnable(GL_LIGHT1);

            glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,GL_TRUE);
            glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);// todo include this into spotlight node

            glEnable(GL_COLOR_MATERIAL);
            glEnable(GL_NORMALIZE);

            Eigen::Matrix3Xd show_vertices = pointCloud->vertices.colwise() + translation_show;
            show_vertices /= mesh_scale;
            glBegin(GL_POINTS);
            for(int i=0; i<pointCloud->vertices.cols(); i++)
            {
                glColor3f(0.9f,0.9f,1.f);
                glVertex3dv(show_vertices.col(i).data());
                Eigen::Vector3d n = -pointCloud->normals.col(i);
                glNormal3dv(n.data());
            }
            glEnd();
        }

        break;
    }
    mutex_showParas->unlock();
    mutex_pointCloud->unlock();
}

void UserViewMesh::init()
{
    translation_show.setZero();
    this->setBackgroundColor(QColor(96,98,90));

    //set up light0 for face mesh
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

    //set up light1 for pointcloud
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

void UserViewMesh::setMutex_pointCloud(QMutex* mutex)
{
    mutex_pointCloud = mutex;
}

int UserViewMesh::setMesh(std::string filename)
{
    pointCloud = NULL;
    mesh.request_vertex_normals();
    mesh.request_face_normals();

    if (!OpenMesh::IO::read_mesh(mesh, filename))
    {
        std::cerr<< "Cannot Open mesh in file ' "<< filename<<" '" << std::endl;
        return 0;
    }

    PolyMesh::VertexIter v_it, v_end = mesh.vertices_end();

    for(v_it = mesh.vertices_begin(); v_it != v_end; ++v_it)
    {
        mesh.point(v_it) = PolyMesh::Point(0,0,0);
    }

    return 1;
}

void UserViewMesh::getPointCloud(TriMesh_Eigen* depthMesh)//get point cloud from face
{
    pointCloud = depthMesh;
}
