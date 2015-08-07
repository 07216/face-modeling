#include "FacePerformanceFunction.h"

void FacePerformanceFunction::AddFace2Off(const std::string &filename)
{
    std::ofstream of(filename.c_str(), std::ios::app);
    std::ifstream in(ConfigParameter::mesh_face_filename.c_str());
    std::string str;
    int idx;
    while(in>>str)
    {
        of<<4<<' ';
        in>>idx;
        of<<idx-1<<' ';
        in>>idx;
        of<<idx-1<<' ';
        in>>idx;
        of<<idx-1<<' ';
        in>>idx;
        of<<idx-1<<std::endl;
    }
    in.close();
    of.close();
}
void FacePerformanceFunction::AddFace2Obj(const std::string &filename)
{
    std::ofstream of(filename.c_str(), std::ios::app);
    std::ifstream in(ConfigParameter::mesh_face_filename.c_str());
    std::string str;
    int idx;
    while(in>>str)
    {
        of<<str<<' ';
        in>>idx;
        of<<idx<<'/'<<idx<<' ';
        in>>idx;
        of<<idx<<'/'<<idx<<' ';
        in>>idx;
        of<<idx<<'/'<<idx<<' ';
        in>>idx;
        of<<idx<<'/'<<idx<<std::endl;
    }
    in.close();
    of.close();
}
void FacePerformanceFunction::PrintSparseMatrix(const SparseMatrixXd& matrix, const std::string & filename)
{
    std::ofstream of(filename.c_str());
    for(int i=0; i<100; i++)
    {
        for(int j=0; j<100; j++)
        {
            of<<matrix.coeff(i,j)<<' ';
        }
        of<<std::endl;
    }
    of.close();
}
void FacePerformanceFunction::ReadMatrixBinary(Eigen::MatrixXd &mat, FILE *input)
{
    int row = 0;
    int col = 0;
    size_t rc = 0;
    if ((rc = fread(&row,sizeof(int), 1,input)) == 0 )
    {
        std::exit(6);
    }
    if ((rc = fread(&col,sizeof(int), 1,input)) == 0 )
    {
        std::exit(6);
    }
    if (row * col == 0)
    {
        std::exit(6);
    }

    mat.resize(row,col);
    if ((rc = fread(&mat(0,0), sizeof(double), row * col, input)) == 0 )
    {
        std::exit(6);
    }

}
void FacePerformanceFunction::LoadSelectedIdx2SparseMatrix(const std::string& filename, SparseMatrixXd & spm)
{
    FILE * fin = fopen(filename.c_str(),"r");
    int idx;
    std::vector<T> triplets_T;
    while(fscanf(fin,"%d;",&idx) != EOF)
    {
        for(int i=0; i<3; i++)
        {
            triplets_T.push_back(T(idx*3+i, idx*3+i, 1.0));
        }
    }
    spm.setFromTriplets(triplets_T.begin(), triplets_T.end());
}
void FacePerformanceFunction::SaveSparseMatrix(SparseMatrixXd& matrix, const std::string & filename)
{
    std::ofstream of(filename.c_str());
    int nonZeros = matrix.nonZeros();
    //matrix.
    double* value = matrix.valuePtr();
    int* row = matrix.innerIndexPtr();
    int* col = matrix.outerIndexPtr();
    of<<matrix.rows()<<' '<<matrix.cols()<<std::endl;
    int j=-1;
    for(int i=0; i<nonZeros; i++)
    {
        while(i>=col[j+1])j++;
        if(abs(value[i])>1.0e-3)
        of<<row[i]<<' '<<j<<' '<<value[i]<<std::endl;
    }
    of.close();
}
void FacePerformanceFunction::LoadSparseMatrix(SparseMatrixXd& matrix, const std::string & filename)
{
    int rows,cols;
    int row,col;
    double val;
    std::ifstream in(filename.c_str());
    FILE * fin = fopen(filename.c_str(),"r");
    fscanf(fin, "%d %d", &rows, &cols);
    matrix.resize(rows, cols);
    std::vector<T> triplets;
    while(!feof(fin))
    {
        fscanf(fin, "%d %d %lf\n", &row, &col, &val);
        triplets.push_back(T(row, col, val));
    }
    matrix.setFromTriplets(triplets.begin(), triplets.end());
    fclose(fin);
}
Eigen::MatrixXd FacePerformanceFunction::Compute_Derive_Color(const Eigen::Vector2i& coor, cv::Mat& colorImage)
{
    Eigen::MatrixXd derive_color(1,2);
    int n = coor(0);
    int m = coor(1);

    if(n+1<colorImage.cols && n>=0)
      derive_color(0,0) = (double)colorImage.at<unsigned char >(m, n+1) - (double)colorImage.at<unsigned char>(m, n);
    else derive_color(0,0) = (double)colorImage.at<unsigned char >(m, n) - (double)colorImage.at<unsigned char>(m, n-1);
    if(m+1<colorImage.rows && m>=0)
       derive_color(0,1) = (double)colorImage.at<unsigned char>(m+1, n)-(double)colorImage.at<unsigned char>(m,n);
    else derive_color(0,1) = (double)colorImage.at<unsigned char>(m, n) - (double)colorImage.at<unsigned char>(m-1, n);
    derive_color /= 255.0;
    return derive_color;
}

Eigen::VectorXd FacePerformanceFunction::Compute_f_color(const Eigen::Vector2i& coor, const cv::Mat& colorImage)
{
    Eigen::VectorXd color(1);
    color(0) = colorImage.at<unsigned char>(coor(1), coor(0));
    color /= 255.0;
    return color;
}

void FacePerformanceFunction::LoadRegisterIndexFromFile(std::vector<int>& index, std::string filename)
{
    index.clear();
    FILE * fin = fopen(filename.c_str(), "r");//ifstream in(filename);
    int idx;
    while(fscanf(fin, "%d", &idx) != EOF)
    {
        index.push_back(idx);
    }
    fclose(fin);//in.close();
}

void FacePerformanceFunction::ComputeNewDepthImageRegion(cv::Rect& face_rect, cv::Mat& img, Eigen::Matrix2Xi& key_point_3d_to_2d)
{
    Eigen::Vector2i center = key_point_3d_to_2d.rowwise().mean();
    int half_width = 100;
    int ul_x = center(0) - half_width;
    int ul_y = center(1) - half_width;
    int br_x = center(0) + half_width;
    int br_y = center(1) + half_width;

    ul_x = ul_x >= 6 ? ul_x : 6;
    ul_y = ul_y >= 6 ? ul_y : 6;
    br_x = br_x <= img.cols-7 ? br_x : img.cols-7;
    br_y = br_y <= img.rows-7 ? br_y : img.rows-7;

    face_rect.width = br_x - ul_x;
    face_rect.height = br_y - ul_y;
    face_rect.x = ul_x;
    face_rect.y = ul_y;

}

void FacePerformanceFunction::PolyMesh2GLMat(std::string mesh_filename)
{
    PolyMesh mesh;
    if(OpenMesh::IO::read_mesh(mesh, mesh_filename)) std::cout<<"read succeed"<<std::endl;
    PolyMesh::VertexIter v_it, v_end = mesh.vertices_end();
    std::cout<<mesh.n_vertices()<<' '<<mesh.n_faces()<<std::endl;
    int num_vertices = mesh.n_vertices();
    cv::Mat mat = cv::Mat::zeros(num_vertices*1, num_vertices*1, CV_32FC1);
    for(v_it = mesh.vertices_begin(); v_it != v_end; ++v_it)
    {
        PolyMesh::VertexOHalfedgeIter voh(mesh, v_it);
        float valence = mesh.valence(v_it);
        int row_idx = v_it->idx();
        for(int i=0; i<1; i++)
        {
            mat.at<float>(row_idx*1+i, row_idx*1+i) = valence;
        }
        for( ; voh; ++voh)
        {
            for(int i=0; i<1; i++)
            {
                mat.at<float>(row_idx*1+i, (mesh.to_vertex_handle(voh)).idx()) = -1.0;
            }
        }
    }
    FILE * of = fopen("mat.txt","w");
    for(int i=0; i<mat.rows; i++)
    {
        for(int j=0; j<mat.cols; j++)
            fprintf(of,"%f ",mat.at<float>(i,j));
        fprintf(of,"\n");
    }
    fflush(of);
    fclose(of);
}

void FacePerformanceFunction::PolyMesh2GLMat_FrontPart(std::string mesh_filename)
{
    //read mesh
    PolyMesh mesh;
    if(OpenMesh::IO::read_mesh(mesh, mesh_filename)) std::cout<<"read succeed"<<std::endl;
    PolyMesh::VertexIter v_it, v_end = mesh.vertices_end();
    std::cout<<mesh.n_vertices()<<' '<<mesh.n_faces()<<std::endl;
    int num_vertices = mesh.n_vertices();

    Eigen::VectorXi mapIndex(num_vertices);
    for(int i=0; i<mapIndex.size(); i++)
        mapIndex[i] = -1;
    //read frontal index
    std::ifstream fin("update_selection_index.txt");
    int val;
    std::vector<int> frontIndex;
    while(fin>>val)
    {
        mapIndex[val] = frontIndex.size();
        frontIndex.push_back(val);
    }
    fin.close();

    cv::Mat mat = cv::Mat::zeros(frontIndex.size(), frontIndex.size(), CV_64FC1);
    for(v_it = mesh.vertices_begin(); v_it != v_end; ++v_it)
    {
        PolyMesh::VertexOHalfedgeIter voh(mesh, v_it);

        int row_idx = mapIndex[v_it->idx()];
        if(row_idx==-1)continue;
        double valence = 0;
        for( ; voh; ++voh)
        {
            int to_idx =  mapIndex[(mesh.to_vertex_handle(voh)).idx()];
            if(to_idx != -1)
            {
                mat.at<double>(row_idx, to_idx) = -1.0;
                valence += 1.0;
            }
        }
        mat.at<double>(row_idx, row_idx) = valence;
    }
    FILE * of = fopen("mat.txt","w");
    //fprintf(of,"0\n");
    for(int i=0; i<mat.rows; i++)
    {
        for(int j=0; j<mat.cols; j++)
            fprintf(of,"%f ",mat.at<double>(i,j));
        fprintf(of,"\n");
    }
    //fprintf(of,"1\n");
    fflush(of);
    fclose(of);
}

double FacePerformanceFunction::BilateralFilter(cv::Mat& depthImage, int y, int x, double sigma_color2_inv_half, double sigma_space2_inv_half, int R)
{
    int sx = x-R;
    int sy = y-R;
    int ex = x+R;
    int ey = y+R;
    double sum1=0, sum2 = 0;
    double value = depthImage.at<unsigned short>(y,x);
    for(int cy=sy; cy<=ey; cy++)
    {
        for(int cx=sx; cx<=ex; cx++)
        {
            double tmp = depthImage.at<unsigned short>(cy, cx);
            if(tmp != 0)
            {
                double space2 = (x - cx) * (x - cx) + (y - cy) * (y - cy);
                double color2 = (value - tmp) * (value - tmp);
                double weight = std::exp(-(space2 * sigma_space2_inv_half + color2 * sigma_color2_inv_half));
                sum1 += tmp * weight;
                sum2 += weight;
            }
        }
    }
    double res = (sum1 / sum2);
    return res;
}
/*
//openni generate point cloud
void FacePerformanceFunction::GeneratePointCloud( TriMesh_Eigen& depthMesh_Eigen, double fx, double fy, cv::Rect& face_rect, cv::Mat& colorImage, cv::Mat& depthImage, unsigned short maxDepth, Eigen::MatrixXi& inDepthImage, int radius_bilateral)
{
    //const double PI = 3.14159165359;
    //double max_theta = cos(78.0/180*PI);
    const static float sigma_color = 30;     //in mm
    const static float sigma_space = 4.5;     // in pixels
    const static double sigma_color2_inv_half = 0.5 / (sigma_color*sigma_color);
    const static double sigma_space2_inv_half = 0.5 / (sigma_space*sigma_space);
    std::cout<<"general Pointcloud"<<std::endl;

    memset(inDepthImage.data(), -1, sizeof(int)*inDepthImage.rows()*inDepthImage.cols());
    int pointcloud_idx = 0;
    std::vector<double> temp_vertices;
    std::vector<double> temp_colors;
    //get pointcloud from facerect only
    //std::cout<<"fx fy: "<<fx<<' '<<fy<<std::endl;
    for(int i=0; i<face_rect.height; i++)
    {
        int y = i + face_rect.y;
        for(int j=0; j<face_rect.width; j++)
        {
            int x = j + face_rect.x;
            unsigned short depth = depthImage.at<unsigned short>(y,x);
            if(depth==0|| depth>maxDepth)continue;
            //vertex

            double real_z = BilateralFilter(depthImage, y, x, sigma_color2_inv_half, sigma_space2_inv_half, radius_bilateral);//depth;
            double real_x = (x*1.0/depthImage.cols - 0.5) * real_z * fx;
            double real_y = (0.5 - y*1.0/depthImage.rows) * real_z * fy;
            temp_vertices.push_back(real_x);
            temp_vertices.push_back(real_y);
            temp_vertices.push_back(-real_z);

            for(int k=0; k<3; k++)
                temp_colors.push_back( (colorImage.at<cv::Vec<unsigned char,3> >(y, x)[k])/255.0 );
            inDepthImage(y,x) = pointcloud_idx ++;
        }
    }
    depthMesh_Eigen.vertices.resize(3, temp_vertices.size()/3);
    depthMesh_Eigen.colors.resize(3, temp_colors.size()/3);
    memcpy(depthMesh_Eigen.vertices.data(), &temp_vertices[0], sizeof(double)*temp_vertices.size());
    memcpy(depthMesh_Eigen.colors.data(), &temp_colors[0], sizeof(double)*temp_colors.size());

    //compute normal
    depthMesh_Eigen.normals.resize(3, depthMesh_Eigen.vertices.cols());
    depthMesh_Eigen.normals.setZero();
    for(int i=0; i<face_rect.height; i++)
    {
        int y = i + face_rect.y;
        for(int j=0; j<face_rect.width; j++)
        {
            int x = j + face_rect.x;
            int index_o = inDepthImage(y,x);
            int index_x = inDepthImage(y,x+1);
            int index_y = inDepthImage(y+1,x);
            if(index_o == -1 || index_x == -1 || index_y == -1)
            {
                continue;
            }
            Eigen::Vector3d gradient_x = depthMesh_Eigen.vertices.col(index_x) - depthMesh_Eigen.vertices.col(index_o);
            Eigen::Vector3d gradient_y = depthMesh_Eigen.vertices.col(index_y) - depthMesh_Eigen.vertices.col(index_o);
            depthMesh_Eigen.normals.col(index_o) = gradient_x.cross(gradient_y).normalized();
        }
    }
    std::cout<<"after general Pointcloud"<<std::endl;


    //std::cout<<"general Pointcloud normal finish： "<<face_rect.x<<' '<<face_rect.y<<' '<<face_rect.width<<' '<<face_rect.height<<std::endl;
    //compute face
    std::vector<int> temp_faces;
    for(int i=0; i<face_rect.height; i++)
    {
        int y = i + face_rect.y;
        for(int j=0; j<face_rect.width; j++)
        {
            int x = j + face_rect.x;
            if(inDepthImage(y+1,x) != -1
                    && inDepthImage(y,x+1) != -1)
            {
                if(inDepthImage(y, x) != -1)
                {
                    temp_faces.push_back(inDepthImage(y,x));
                    temp_faces.push_back(inDepthImage(y+1,x));
                    temp_faces.push_back(inDepthImage(y, x+1));
                    //depthMesh->faces.push_back(TriMesh::Face(inDepthImage(y,x), inDepthImage(y+1,x), inDepthImage(y, x+1)));
                }
                if(inDepthImage(y+1, x+1) != -1)
                {
                    temp_faces.push_back(inDepthImage(y, x+1));
                    temp_faces.push_back(inDepthImage(y+1, x));
                    temp_faces.push_back(inDepthImage(y+1, x+1));
                }
            }

        }
    }
    depthMesh_Eigen.faces.resize(3, temp_faces.size()/3);
    memcpy(depthMesh_Eigen.faces.data(), &temp_faces[0], sizeof(int)*temp_faces.size());

    //std::cout<<"general Pointcloud finish"<<std::endl;
}*/
//openni generate point cloud
void FacePerformanceFunction::GeneratePointCloud( TriMesh_Eigen& depthMesh_Eigen, double fx, double fy, cv::Rect& face_rect, cv::Mat& colorImage, cv::Mat& depthImage, unsigned short maxDepth, Eigen::MatrixXi& inDepthImage, int radius_bilateral)
{
    clock_t start;
    //const double PI = 3.14159165359;
    //double max_theta = cos(78.0/180*PI);
    const static double sigma_color = 30;     //in mm
    const static double sigma_space = 4.5;     // in pixels

    memset(inDepthImage.data(), -1, sizeof(int)*inDepthImage.rows()*inDepthImage.cols());
    int pointcloud_idx = 0;
    std::vector<double> temp_vertices(face_rect.height*face_rect.width*3);
    std::vector<double> temp_colors(face_rect.height*face_rect.width*3);

    cv::Mat faceImg(face_rect.height, face_rect.width, CV_32FC1);

    //#pragma omp parallel for
    for (int i = 0; i < face_rect.height; ++i)
    {
        int y = i + face_rect.y;
        for (int j = 0; j < face_rect.width; ++j)
        {
            int x = j + face_rect.x;
            faceImg.at<float>(i, j) = depthImage.at<unsigned short>(y, x);
        }
    }
    cv::Mat faceImg_smooth;
    start = clock();
    if (radius_bilateral != 0)
        cv::bilateralFilter(faceImg, faceImg_smooth, radius_bilateral, sigma_color, sigma_space);
    else faceImg_smooth = faceImg.clone();
    std::cout<<"cost bilateral: "<<(clock()-start)*1.0/CLOCKS_PER_SEC<<std::endl;

    //store the face part for pose estimation
    depthMesh_Eigen.img3D_pose = cv::Mat(face_rect.height, face_rect.width, CV_32FC3);

    //std::cout<<"before face_rect"<<std::endl;
    start = clock();
    for (int i = 0; i < face_rect.height; ++i)
    {
        int y = i + face_rect.y;
        cv::Vec3f* Mi = depthMesh_Eigen.img3D_pose.ptr<cv::Vec3f>(i);
        for (int j = 0; j < face_rect.width; ++j)
        {
            int x = j + face_rect.x;
            unsigned short depth = depthImage.at<unsigned short>(y,x);
            if (depth == 0 || depth > maxDepth)
            {
                Mi[j] = 0;
                continue;
            }

            double real_z = faceImg_smooth.at<float>(i,j);
            double real_x = (x*1.0/depthImage.cols - 0.5) * real_z * fx;
            double real_y = (0.5 - y*1.0/depthImage.rows) * real_z * fy;
            temp_vertices[3*pointcloud_idx] = real_x;
            temp_vertices[3*pointcloud_idx + 1] = real_y;
            temp_vertices[3*pointcloud_idx + 2] = -real_z;
            Mi[j][0] = real_x;
            Mi[j][1] = -real_y;
            Mi[j][2] = real_z;

            for (int k = 0; k < 3; ++k)
                temp_colors[3*pointcloud_idx + k] = ((colorImage.at<cv::Vec<unsigned char,3> >(y, x)[k])/255.0);
            inDepthImage(y, x) = pointcloud_idx++;
        }
    }
    //std::cout<<"after face_rect"<<std::endl;
    depthMesh_Eigen.vertices.resize(3, pointcloud_idx);
    depthMesh_Eigen.colors.resize(3, pointcloud_idx);
    memcpy(depthMesh_Eigen.vertices.data(), &temp_vertices[0], sizeof(double)*3*pointcloud_idx);
    memcpy(depthMesh_Eigen.colors.data(), &temp_colors[0], sizeof(double)*3*pointcloud_idx);

    std::cout<<"...cost vc: "<<(clock()-start)*1.0/CLOCKS_PER_SEC<<std::endl;
    //std::cout<<"before kdtree"<<std::endl;
    //std::cout<<"vertices: "<< depthMesh_Eigen.vertices.size()<<std::endl;
    if (depthMesh_Eigen.kdtree != NULL)
        delete depthMesh_Eigen.kdtree;
    if (depthMesh_Eigen.vertices.size() == 0)
    {
        depthMesh_Eigen.kdtree = NULL;
        return;
    }
    depthMesh_Eigen.kdtree = new KDTree(depthMesh_Eigen.vertices);

    std::cout<<"cost vc and kdtree: "<<(clock()-start)*1.0/CLOCKS_PER_SEC<<std::endl;
    //compute normal
    start = clock();
    depthMesh_Eigen.normals.resize(3, depthMesh_Eigen.vertices.cols());
    depthMesh_Eigen.normals.setZero();
    //#pragma omp parallel for
    for (int i = 0; i < face_rect.height; ++i)
    {
        int y = i + face_rect.y;
        for (int j = 0; j < face_rect.width; ++j)
        {
            int x = j + face_rect.x;
            int index_o = inDepthImage(y,x);
            int index_x = inDepthImage(y,x+1);
            int index_y = inDepthImage(y+1,x);
            if (index_o == -1 || index_x == -1 || index_y == -1)
            {
                continue;
            }
            Eigen::Vector3d gradient_x = depthMesh_Eigen.vertices.col(index_x) - depthMesh_Eigen.vertices.col(index_o);
            Eigen::Vector3d gradient_y = depthMesh_Eigen.vertices.col(index_y) - depthMesh_Eigen.vertices.col(index_o);
            depthMesh_Eigen.normals.col(index_o) = gradient_x.cross(gradient_y).normalized();
        }
    }

    //compute face
    start = clock();
    pointcloud_idx = 0;
    std::vector<int> temp_faces(6*face_rect.height*face_rect.width);
    for(int i=0; i<face_rect.height; i++)
    {
        int y = i + face_rect.y;
        for(int j=0; j<face_rect.width; j++)
        {
            int x = j + face_rect.x;
            if(inDepthImage(y+1,x) != -1 && inDepthImage(y,x+1) != -1)
            {
                if(inDepthImage(y, x) != -1)
                {
                    temp_faces[pointcloud_idx++] = inDepthImage(y, x + 1);
                    temp_faces[pointcloud_idx++] = inDepthImage(y, x);
                    temp_faces[pointcloud_idx++] = inDepthImage(y + 1, x);
                }
                if(inDepthImage(y+1, x+1) != -1)
                {
                    temp_faces[pointcloud_idx++] = inDepthImage(y + 1, x + 1);
                    temp_faces[pointcloud_idx++] = inDepthImage(y, x + 1);
                    temp_faces[pointcloud_idx++] = inDepthImage(y + 1, x);
                }
            }
        }
    }
    depthMesh_Eigen.faces.resize(3, pointcloud_idx/3);
    memcpy(depthMesh_Eigen.faces.data(), &temp_faces[0], sizeof(int)*pointcloud_idx);
    std::cout<<"cost face: "<<(clock()-start)*1.0/CLOCKS_PER_SEC<<std::endl;
}

void FacePerformanceFunction::ChooseRegisterPoint(const Eigen::MatrixXd &beforeChoose, Eigen::MatrixXd& afterChoose, std::vector<int>& registerIndex )
{
    int sizes = registerIndex.size();
    int ss = sizes*3;
    int colss = beforeChoose.cols();
    afterChoose.resize(ss, colss);
     #pragma omp parallel for
    for (int i = 0; i < sizes; i++)
    {
        int index = registerIndex[i];
        afterChoose.block(i*3, 0, 3, colss) = beforeChoose.block(index * 3, 0, 3, colss);
    }
}
Eigen::Matrix2Xd FacePerformanceFunction::ConvertRealWorldToProjective_Matrix(const Eigen::MatrixXd& realWorldPoint, const cv::Mat& colorImage, double fx, double fy)
{
    //std::cout<<0<<std::endl;
    Eigen::MatrixXd constant(1, realWorldPoint.cols());
    constant.setOnes();
    constant *= 0.5;
    Eigen::Matrix2Xd coor(2, realWorldPoint.cols()); //std::cout<<1<<std::endl;
    coor.row(0) = realWorldPoint.row(0).array()/(-realWorldPoint.row(2).array())*(colorImage.cols/fx);//std::cout<<2<<std::endl;
    coor.row(0) += constant.row(0) * (colorImage.cols + 1);//std::cout<<3<<std::endl;
    //) * colorImage.cols + constant.row(0);

    coor.row(1) = realWorldPoint.row(1).array()/realWorldPoint.row(2).array()*(colorImage.rows/fy);//std::cout<<4<<std::endl;
    coor.row(1) += constant.row(0) * (colorImage.rows + 1);//std::cout<<5<<std::endl;

    coor.row(0) = (coor.row(0).array() < 0).select(0, coor.row(0));//std::cout<<6<<std::endl;
    coor.row(0) = (coor.row(0).array()>colorImage.cols-1).select(colorImage.cols-1, coor.row(0));//std::cout<<7<<std::endl;

    coor.row(1) = (coor.row(1).array()<0).select(0, coor.row(1));
    coor.row(1) = (coor.row(1).array()>colorImage.rows-1).select(colorImage.rows-1, coor.row(1));//std::cout<<8<<std::endl;

    return coor;

//        coor(0) = (realWorldPoint(0,0)/fx/(-realWorldPoint(2,0))+0.5)*colorImage.cols + 0.5;
//        coor(1) = (0.5-realWorldPoint(1,0)/fy/(-realWorldPoint(2,0)))*colorImage.rows + 0.5;
//        if(coor(0) < 0) coor(0) = 0;
//        else if(coor(0) >= colorImage.cols) coor(0) = colorImage.cols - 1;
//        if(coor(1) < 0) coor(1) = 0;
//        else if(coor(1) >= colorImage.rows) coor(1) = colorImage.rows - 1;
//        return coor;
}

SparseMatrixXd FacePerformanceFunction::GetUpSamplingSparseMatrix(PolyMesh& mesh, std::vector<int>& init_registerIndex, std::vector<int>& selected_upsamplingIndex)
{
    std::vector<T> triplets;
    /*for(int i=0; i<init_registerIndex.size(); i++)
    {
        triplets.push_back(T(i,i,1.0));
    }*/
    bool *mark = NULL;
    mark = new bool[mesh.n_vertices()];
    for(int i=0; i<mesh.n_vertices(); i++)
    {
        mark[i] = false;
    }
    for(int i=0; i<selected_upsamplingIndex.size(); i++)
    {
        int index = selected_upsamplingIndex[i];
        mark[index] = true;
    }

    int current_col = 0;//init_registerIndex.size();
    //add point in trangle
    for (PolyMesh::FaceIter faceIt = mesh.faces_begin(); faceIt != mesh.faces_end(); ++faceIt)
    {
        Eigen::Vector3i index3;
        int i = 0;
        for (PolyMesh::FaceVertexIter vertexIt = mesh.fv_begin(faceIt); vertexIt != mesh.fv_end(faceIt); ++vertexIt,i++)
        {
            int idx = vertexIt.handle().idx();
            if(mark[idx]==false) break;
            index3[i] = idx;
        }
        if(i<3)continue;

        for(int j=0; j<3; j++)
        {
            triplets.push_back(T(index3[j], current_col, 1.0/3.0));
        }
        current_col ++;
        for(int j=0; j<3; j++)
        {
            triplets.push_back(T(index3[j], current_col, 2.0/3.0));
            triplets.push_back(T(index3[(j+1)%3], current_col, 1.0/6.0));
            triplets.push_back(T(index3[(j+2)%3], current_col, 1.0/6.0));
            current_col ++;
        }
    }
    //add point on edge
    for (PolyMesh::EdgeIter edgeIt = mesh.edges_begin(); edgeIt != mesh.edges_end(); ++edgeIt)
    {
        PolyMesh::HalfedgeHandle heh = mesh.halfedge_handle(edgeIt, 0);
        PolyMesh::VertexHandle vh_from = mesh.from_vertex_handle(heh);
        PolyMesh::VertexHandle vh_to = mesh.to_vertex_handle(heh);
        int index_from = vh_from.idx();
        int index_to = vh_to.idx();
        if(mark[index_from] && mark[index_to])
        {
            triplets.push_back(T(index_from, current_col, 0.5));
            triplets.push_back(T(index_to, current_col, 0.5));
            current_col ++;
        }
    }
    if(mark!=NULL)
        delete [] mark;

    SparseMatrixXd sp(init_registerIndex.size(), current_col);
    sp.setFromTriplets(triplets.begin(), triplets.end());

    return sp;
}
void FacePerformanceFunction::GetBoundaryIndexOfRegisterIndex(const std::vector<int>& registerIndex, const std::string& mesh_filename, std::vector<int>& boundaryIndex)
{
    boundaryIndex.clear();
    PolyMesh mesh;
    if(!OpenMesh::IO::read_mesh(mesh, mesh_filename))
    {
        std::cout<<"read boundary index error"<<std::endl;
        return;
    }
    bool* mark = NULL;
    mark = new bool[mesh.n_vertices()];
    for(int i=0; i<mesh.n_vertices(); i++)
    {
        mark[i] = false;
    }
    for(int i=0; i<registerIndex.size(); i++)
    {
        int index = registerIndex[i];
        mark[index] = true;
    }
    PolyMesh::VertexIter v_it = mesh.vertices_begin();
    PolyMesh::VertexIter v_end = mesh.vertices_end();
    for(; v_it != v_end; ++v_it)
    {
        int idx = v_it.handle().idx();
        if(!mark[idx]) continue;
        PolyMesh::VertexVertexIter vv_it = mesh.vv_begin(v_it.handle());
        PolyMesh::VertexVertexIter vv_end = mesh.vv_end(v_it.handle());
        bool is_boundary = false;
        for( ; vv_it != vv_end; ++vv_it)
        {
            int vv_idx = vv_it.handle().idx();
            if(!mark[vv_idx])
            {
                is_boundary = true;
                break;
            }
        }
        if(is_boundary)
        {
            boundaryIndex.push_back(idx);
        }
    }
    if(mark != NULL)
        delete [] mark;
}

void FacePerformanceFunction::GetFacesIndexOfVerticesIndex(const std::vector<int>& verticesIndex, PolyMesh& mesh, std::vector<int>& facesIndex)
{
    facesIndex.clear();

    int n_vertices = mesh.n_vertices();
    bool * mark = NULL;
    mark = new bool[n_vertices];
    for(int i=0; i<n_vertices; i++)
    {
        mark[i] = false;
    }

    std::map<int, int> map_index;
    for(int i=0; i<verticesIndex.size(); i++)
    {
        int index = verticesIndex[i];
        map_index[index] = i;
        mark[index] = true;
    }

    //select the facesIndex which contain verticesIndex
    for (PolyMesh::FaceIter faceIt = mesh.faces_begin(); faceIt != mesh.faces_end(); ++faceIt)
    {
        //Get the vertices of the point
        bool isRigidFace = true;
        for (PolyMesh::FaceVertexIter vertexIt = mesh.fv_begin(faceIt); vertexIt != mesh.fv_end(faceIt); ++vertexIt)
        {
            int idx = vertexIt.handle().idx();
            if(!mark[idx]) isRigidFace = false;
        }
        if(isRigidFace)
        {
            for (PolyMesh::FaceVertexIter vertexIt = mesh.fv_begin(faceIt); vertexIt != mesh.fv_end(faceIt); ++vertexIt)
            {
                int idx = vertexIt.handle().idx();
                facesIndex.push_back(map_index[idx]);
            }
        }
    }
    if(mark != NULL)
    delete [] mark;
}

//recognition part
void FacePerformanceFunction::SaveInitRegisterPart_binary(const Eigen::MatrixXd& face, const char* filename, int init_registerSize)
{
    FILE * fout = fopen(filename, "wb");
    int m = 3;
    int n = init_registerSize;
    fwrite(&m, sizeof(int), 1, fout);
    fwrite(&n, sizeof(int), 1, fout);
    fwrite(face.data(), sizeof(double), m*n, fout);
    fclose(fout);
}

void FacePerformanceFunction::LoadInitRegisterPart_binary(Eigen::MatrixXd& init_registerFace, const char* filename)
{
    FILE * fin = fopen(filename, "rb");
    int m,n;
    fread(&m, sizeof(int), 1, fin);
    fread(&n, sizeof(int), 1, fin);
    init_registerFace.resize(m, n);
    fread(init_registerFace.data(), sizeof(double), m*n, fin);
    fclose(fin);
}
void FacePerformanceFunction::ChooseRecognitionFacePart(const Eigen::MatrixXd& src, Eigen::MatrixXd& tar, std::vector<int>& chooseIndex)
{
    int m = src.rows();
    tar.resize(m, chooseIndex.size());
    for(int i=0; i<chooseIndex.size(); i++)
    {
        int idx = chooseIndex[i];
        tar.col(i) = src.col(idx);
    }
}

int FacePerformanceFunction::GetNumFace(std::string filename_face_index)
{
    int num_face;
    std::ifstream fin(filename_face_index.c_str());
    fin>>num_face;
    fin.close();
    return num_face;
}
void FacePerformanceFunction::SaveNumFace(int num_face, std::string filename_face_index)
{
    std::ofstream fout(filename_face_index.c_str());
    fout<<num_face;
    fout.close();
}
