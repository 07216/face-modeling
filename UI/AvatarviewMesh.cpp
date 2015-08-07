#include "AvatarviewMesh.h"
#include "qdebug.h"
#include <iostream>
#include <time.h>

void ReadMatrixBinary(Eigen::MatrixXd &mat, FILE *input)
{
    int row = 0;
    int col = 0;
    size_t rc = 0;
    if ((rc = fread(&row,sizeof(int), 1,input)) == 0 )
    {
        std::cerr<<"Read error!"<<std::endl;
        std::exit(6);
    }
    if ((rc = fread(&col,sizeof(int), 1,input)) == 0 )
    {
        std::cerr<<"Read error!"<<std::endl;
        std::exit(6);
    }
    if (row * col == 0)
    {
        std::cerr<<"The data is wrong!"<<std::endl;
        std::exit(6);
    }

    mat.resize(row,col);
    if ((rc = fread(&mat(0,0), sizeof(double), row * col, input)) == 0 )
    {
        std::cerr<<"Read error!"<<std::endl;
        std::exit(6);
    }

}

AvatarViewMesh::AvatarViewMesh()
{
}

AvatarViewMesh::~AvatarViewMesh()
{
    while(!TextureIDs.empty()){
        GLuint textureID = TextureIDs[TextureIDs.size()];
        TextureIDs.pop_back();
        if (glIsTexture(textureID))
        {
            glDeleteTextures( 1, &textureID);
        }
    }
}

void AvatarViewMesh::draw()
{

    //wheather to draw or not
    mutex_isTracking->lock();
    if(!(*isTracking))
    {
        mutex_isTracking->unlock();
        return;
    }
    mutex_isTracking->unlock();

    mutex_x->lock();
    PolyMesh& mesh = avatar_meshes[cur_mesh_index];
    glEnable( GL_TEXTURE_2D );
    glEnable(GL_BLEND);// you enable blending function
    glBlendFunc(GL_ONE, GL_ZERO);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
    int Texture_index_old = 0;
    for (PolyMesh::FaceIter faceIt = mesh.faces_begin(); faceIt != mesh.faces_end(); ++faceIt)
    {
        int Texture_index_new = mesh.texture_index(faceIt.handle());
        if (Texture_index_new != Texture_index_old)
        {
            glBindTexture(GL_TEXTURE_2D, TextureIDs[Texture_index_new - 1]);
            Texture_index_old = Texture_index_new;
        }
        PolyMesh::Color f_color = mesh.color(faceIt.handle());
        glBegin(GL_POLYGON);
        glColor3fv(&f_color[0]);
        PolyMesh::FaceHalfedgeIter heIt = mesh.fh_begin(faceIt);
        for (PolyMesh::FaceVertexIter vertexIt = mesh.fv_begin(faceIt); vertexIt != mesh.fv_end(faceIt); ++vertexIt)
        {
            PolyMesh::Normal normal = mesh.normal(vertexIt.handle());
            PolyMesh::Point point = mesh.point(vertexIt.handle());
            PolyMesh::TexCoord2D tex = mesh.texcoord2D(heIt.handle());
            heIt++;
            glNormal3dv(&normal[0]);
            glTexCoord2fv(&tex[0]);
            glVertex3dv(&point[0]);
        }
        glEnd();
    }
    glDisable( GL_TEXTURE_2D );
    for (PolyMesh::EdgeIter edgeIt = mesh.edges_begin(); edgeIt != mesh.edges_end(); ++edgeIt)
    {
        PolyMesh::HalfedgeHandle helfedge_hd = mesh.halfedge_handle(edgeIt.handle(),0);
        PolyMesh::Point Vertex1 = mesh.point( mesh.from_vertex_handle(helfedge_hd) );
        PolyMesh::Point Vertex2 = mesh.point( mesh.to_vertex_handle(helfedge_hd) );
        glBegin(GL_LINE);
        glColor3f(1,1,1);
        glVertex3dv(&Vertex1[0]);
        glVertex3dv(&Vertex2[0]);
        glEnd();
    }
    mutex_x->unlock();
    glDisable(GL_BLEND);// you enable blending function
}

void AvatarViewMesh::init()
{
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 20.0);
    GLfloat specular_color[4] = { 0.5f, 0.5f, 0.5f, 1.0 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  specular_color);

    PolyMesh& mesh = avatar_meshes[cur_mesh_index];
    R.resize(3,3);
    R.setIdentity();
    translation_show << 10000.0, 100000.0, 100000.0;
    T.resize(3);
    T<<100000.0,100000.0,100000.0;
    this->setBackgroundColor(QColor(96,98,90));
    int texture_size = textures.size();
    TextureIDs.resize(texture_size);
    for (int i = 0; i < texture_size; i++){
        //GLuint TextureID;
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glGenTextures(texture_size, &TextureIDs[i]);
        glBindTexture(GL_TEXTURE_2D, TextureIDs[i]);
        // copy texture to GL
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, textures[i].depth()/8, textures[i].width(),
             textures[i].height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, textures[i].bits());
    }
    std::cout<<"after avatar init"<<std::endl;
}

void AvatarViewMesh::loadBaseModel(const std::string& pathOfObj, PolyMesh& mesh, Eigen::MatrixXd& modelOfFaceShift)
{
    clock_t start = clock();
    std::string objFilename = pathOfObj + "mtl_list.txt";
    std::ifstream if_mtl(objFilename.c_str(), std::ifstream::in);
    std::string pic;
    while(getline(if_mtl, pic)){
        QImage a(std::string("").append(pathOfObj).append(pic).c_str());
        textures.push_back(QGLViewer::convertToGLFormat(a));
    }
    if_mtl.close();

    mesh.request_vertex_normals();
    mesh.request_face_normals();
    mesh.request_face_colors();
    mesh.request_vertex_texcoords2D();
    mesh.request_face_texture_index();
    mesh.request_halfedge_texcoords2D();

    std::string fileName = pathOfObj + "0_Neutral.obj";
    OpenMesh::IO::Options io_op;
    io_op += OpenMesh::IO::Options::VertexNormal;
    io_op += OpenMesh::IO::Options::VertexTexCoord;
    io_op += OpenMesh::IO::Options::FaceColor;
    io_op += OpenMesh::IO::Options::FaceTexCoord;
    if (!OpenMesh::IO::read_mesh(mesh, fileName, io_op))
    {
        std::cout<<"Can't open the file "<<fileName<<std::endl;
        exit(8);
    }
    mesh.update_face_normals();
    mesh.update_normals();
    PolyMesh::VertexIter v_it, v_end = mesh.vertices_end();
    for(v_it = mesh.vertices_begin(); v_it != v_end; ++v_it)
    {
        mesh.point(v_it) = PolyMesh::Point(0,0,0);
    }
    std::string model_mat_filename = pathOfObj + "model_matrix.dat";
    FILE * f_model_mat;
    f_model_mat = fopen(model_mat_filename.c_str(), "rb");
    ReadMatrixBinary(modelOfFaceShift, f_model_mat);
    fclose(f_model_mat);
}

void AvatarViewMesh::updateMesh()
{
    const static double mesh_scale = 300.0;
    mutex_x->lock();
    PolyMesh& mesh = avatar_meshes[cur_mesh_index];
    Eigen::MatrixXd& modelOfFaceShift = avatar_models[cur_mesh_index];
    static int count_function = 0;
    count_function++;
    double xbar = 1 - x.sum();

    int iii=0;
    PolyMesh::VertexIter v_it, v_end = mesh.vertices_end();

    for(v_it = mesh.vertices_begin(); v_it != v_end; ++v_it,iii++)
    {
        Eigen::Vector3d new_v = modelOfFaceShift.block(3*iii, 0, 3, 1) * xbar;
        for (int i = 1; i < 47; i++)
        {
            new_v += modelOfFaceShift.block(iii*3, i, 3, 1) * x[i-1];
        }
        new_v = (R*new_v/20 *100 + T + translation_show)/mesh_scale;

        mesh.point(v_it) = PolyMesh::Point(&new_v(0));
    }
    mutex_x->unlock();
}

void AvatarViewMesh::updateX( Eigen::VectorXd *x )
{
    this->x = *x;
    updateMesh();
    update();
}

void AvatarViewMesh::loadAllAvatars(const std::string& all_avatars_model_dir)
{
    cur_mesh_index = 0;
    std::string objListFilename = all_avatars_model_dir + "avatar_obj_list.txt";
    std::ifstream fin(objListFilename.c_str());
    int num_avatars;
    fin >> num_avatars;
    avatar_models.resize(num_avatars);
    for(int i=0; i<num_avatars; i++)
    {
        std::string obj_dir;
        fin>>obj_dir;
        obj_dir = all_avatars_model_dir + obj_dir + "/";
        loadBaseModel(obj_dir, avatar_meshes[i], avatar_models[i]);
        std::cout<<obj_dir<<std::endl;
    }
}

void AvatarViewMesh::getBaseModel( std::string pathOfObj )
{
    const char* version = (const char*)glGetString(GL_VERSION);
    clock_t start = clock();

    std::string objFilename = pathOfObj + "mtl_list.txt";
    std::ifstream if_mtl(objFilename.c_str(), std::ifstream::in);
    std::string pic;
    while(getline(if_mtl, pic)){
        QImage a(std::string("").append(pathOfObj).append(pic).c_str());
        textures.push_back(QGLViewer::convertToGLFormat(a));
    }
    if_mtl.close();

    mesh.request_vertex_normals();
    mesh.request_face_normals();
    mesh.request_face_colors();
    mesh.request_vertex_texcoords2D();
    mesh.request_face_texture_index();
    mesh.request_halfedge_texcoords2D();

    std::string fileName = pathOfObj + "0_Neutral.obj";
    OpenMesh::IO::Options io_op;
    io_op += OpenMesh::IO::Options::VertexNormal;
    io_op += OpenMesh::IO::Options::VertexTexCoord;
    io_op += OpenMesh::IO::Options::FaceColor;
    io_op += OpenMesh::IO::Options::FaceTexCoord;
    io_op += OpenMesh::IO::Options::FaceNormal;
    if (!OpenMesh::IO::read_mesh(mesh, fileName, io_op))
    {
        std::cout<<"Can't open the file "<<fileName<<std::endl;
        exit(8);
    }
    mesh.update_normals();
    MyTriMesh::VertexIter v_it, v_end = mesh.vertices_end();
    for(v_it = mesh.vertices_begin(); v_it != v_end; ++v_it)
    {
        mesh.point(v_it) = MyTriMesh::Point(0,0,0);
    }
    std::string model_mat_filename = pathOfObj + "model_matrix.dat";
    FILE * f_model_mat;
    f_model_mat = fopen(model_mat_filename.c_str(), "rb");
    ReadMatrixBinary(modelOfFaceShift, f_model_mat);
    fclose(f_model_mat);
}

