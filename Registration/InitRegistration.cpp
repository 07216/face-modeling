#include "InitRegistration.h"

InitRegister::InitRegister(FacePerformanceParameter* face_paras)
{
    of.open("error_InitRegister.txt");
    this->face_paras = face_paras;
    this->paras = face_paras->init_paras;

    omp_set_num_threads(16);
}

InitRegister::~InitRegister()
{
}

void InitRegister::InitRt_yz0()
{
    yz0.resize(paras->num_yz0);
    yz0.setZero();

    #ifdef USE_KEYPOINT_INIT
    //init with 2d key point
    if (face_paras->marker_registerIndex.size() >= 3)
    {
        int num_cols = face_paras->marker_registerIndex.size();
        Eigen::Matrix3Xd marker_neutral(3, num_cols);
        Eigen::Matrix3Xd marker_pointcloud(3, num_cols);

        #pragma omp parallel for
        for(int i=0; i<face_paras->marker_registerIndex.size(); i++)
        {
            int index1 = face_paras->marker_registerIndex[i];
            marker_neutral.col(i) = face_paras->mean_neutral.block(index1*3, 0, 3, 1);
            int index2 = face_paras->marker_nearestIndex[i];
            marker_pointcloud.col(i) = face_paras->pointCloudMesh->vertices.col(index2);
        }
        face_paras->RT = RigidMotionEstimator::point_to_point(marker_neutral,
                                                              marker_pointcloud, Eigen::VectorXd::Ones(num_cols));
    }
    else
    #endif
   //init with pose estimation
    {
        int idx = face_paras->poseEstimation->best_cluster_index;
        double mult = 0.0174532925;

        double rx = -face_paras->poseEstimation->g_means[idx][3] * mult;
        double ry = face_paras->poseEstimation->g_means[idx][4] * mult;
        double rz = face_paras->poseEstimation->g_means[idx][5] * mult;
        face_paras->RT =    Eigen::AngleAxisd(-rx, Eigen::Vector3d::UnitX())
                            * Eigen::AngleAxisd(-ry, Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(-rz, Eigen::Vector3d::UnitZ());
        face_paras->RT.translation()(0) = face_paras->poseEstimation->g_means[idx][0];
        face_paras->RT.translation()(1) = -face_paras->poseEstimation->g_means[idx][1];
        face_paras->RT.translation()(2) = -face_paras->poseEstimation->g_means[idx][2];
   }
}

void InitRegister::Update_yz0()
{
    int num_rows = paras->init_registerIndex_size + paras->fixed_registerIndex.size()*3;
#ifdef USE_KEYPOINT_INIT
    if (face_paras->marker_registerIndex.size() >= 3)
    {
        num_rows += face_paras->marker_registerIndex.size() * 2;
    }
#endif

    paras->A.resize(num_rows, paras->num_yz0);
    paras->b.resize(num_rows);
    paras->A.setZero();
    paras->b.setZero();

    Eigen::Matrix3d R = face_paras->RT.rotation();
    Eigen::Vector3d T = face_paras->RT.translation();

    paras->nt_R_t.noalias() = R.transpose() * paras->Qn;
    paras->init_P_eigenvectors_T.resize(3, paras->init_registerIndex_size*paras->num_y);
    paras->init_E_eigenvectors_T.resize(3, paras->init_registerIndex_size*paras->num_z0);
    paras->init_mean_neutral.resize(3, paras->init_registerIndex_size);
    paras->R_m_t_c.noalias() = R*paras->init_mean_neutral;
    paras->R_m_t_c -= paras->Qp;
    paras->R_m_t_c.colwise() += T;

    paras->b.topRows(paras->init_registerIndex_size)
            = -(paras->Qn.array() * paras->R_m_t_c.array()).colwise().sum().transpose();

    #pragma omp parallel for
    for (int i = 0; i < paras->num_y; ++i)
    {
        //point to plane
        paras->A.block(0, i, paras->init_registerIndex_size, 1)
                = (paras->nt_R_t.array()
                   *paras->init_P_eigenvectors_T.block(0,
                                                       i*paras->init_registerIndex_size,
                                                       3,
                                                       paras->init_registerIndex_size
                                                       ).array()).colwise().sum().transpose();
    }
    #pragma omp parallel for
    for (int i = 0; i < paras->num_z0; ++i)
    {
        paras->A.block(0,paras->num_y+i, paras->init_registerIndex_size, 1)
                = (paras->nt_R_t.array()
                   *paras->init_E_eigenvectors_T.block(0,
                                                       i*paras->init_registerIndex_size,
                                                       3,
                                                       paras->init_registerIndex_size
                                                       ).array()).colwise().sum().transpose();
    }

    for (int i = 0; i < paras->init_registerIndex_size; ++i)
    {
        if (paras->weight_correspond(i) == 0.0)
        {
            paras->b.row(i).setZero();
            paras->A.row(i).setZero();
        }
    }

    //jaw part
    #pragma omp parallel for
    for(int i=0; i<paras->jaw_registeIndex.size(); ++i)
    {
        int index = paras->jaw_registeIndex[i];
        paras->b.row(index) *= 2.0;
        paras->A.row(index) *= 2.0;
    }

    paras->init_mean_neutral.resize(3*paras->init_registerIndex_size, 1);
    paras->init_P_eigenvectors_T.resize(3*paras->init_registerIndex_size, paras->num_y);
    paras->init_E_eigenvectors_T.resize(3*paras->init_registerIndex_size, paras->num_z0);

    //point to point boundary constraint
#ifdef FIX_MOUTH_NOSE_EYE
    double weight_constraint = 1;
    paras->A.block(paras->init_registerIndex_size, 0, paras->fixed_registerIndex.size()*3, paras->num_y).setZero();
    paras->A.block(paras->init_registerIndex_size, paras->num_y, paras->fixed_registerIndex.size()*3, paras->num_z0)
            = face_paras->E_eigenvectors_T.block(paras->init_registerIndex_size*3, 0, paras->fixed_registerIndex.size()*3, paras->num_z0).setZero() * weight_constraint;
    paras->b.block(paras->init_registerIndex_size, 0, paras->fixed_registerIndex.size()*3, 1).setZero();
#else
    paras->A.block(paras->init_registerIndex_size, 0, paras->fixed_registerIndex.size()*3, paras->num_yz0).setZero();
    paras->b.block(paras->init_registerIndex_size, 0, paras->fixed_registerIndex.size()*3, 1).setZero();
#endif

    //point to point keypoint constraint
#ifdef USE_KEYPOINT_INIT
    int start_rowid = paras->init_registerIndex_size + paras->fixed_registerIndex.size()*3;
    if(face_paras->marker_registerIndex.size()>=3)
    {
        for(int i=0; i<face_paras->marker_registerIndex.size(); i++)
        {
            int meshIndex = face_paras->marker_registerIndex[i];
            Eigen::VectorXd Rm_t = -(R * face_paras->mean_neutral.middleRows(meshIndex*3, 3) + T);
            Eigen::MatrixXd RP = R * face_paras->P_eigenvectors_T.middleRows(meshIndex*3, 3);
            Eigen::MatrixXd RE = R * face_paras->E_eigenvectors_T.middleRows(meshIndex*3, 3);
            double rx = -face_paras->fx * (face_paras->marker_Coordinates[i*2]/face_paras->colorImage.cols - 0.5);
            double ry = -face_paras->fy * (0.5 - face_paras->marker_Coordinates[i*2+1]/face_paras->colorImage.rows);
            paras->A.block(start_rowid+i*2, 0, 1, paras->num_y) = RP.row(0) - RP.row(2)*rx;
            paras->A.block(start_rowid+i*2, paras->num_y, 1, paras->num_z0) = RE.row(0) - RE.row(2)*rx;
            paras->A.block(start_rowid+i*2+1, 0, 1, paras->num_y) = RP.row(1) - RP.row(2)*ry;
            paras->A.block(start_rowid+i*2+1, paras->num_y, 1, paras->num_z0) = RE.row(1) - RE.row(2)*ry;
            paras->b(start_rowid+i*2) = rx*Rm_t(2) - Rm_t(0);
            paras->b(start_rowid+i*2+1) = ry*Rm_t(2)-Rm_t(1);
        }
        paras->A.bottomRows(face_paras->marker_Coordinates.size()) *= paras->markerWeight_yz0;
        paras->b.tail(face_paras->marker_Coordinates.size()) *= paras->markerWeight_yz0;
    }
#endif

    paras->ATA.resize(paras->num_yz0, paras->num_yz0);
    paras->ATA.setZero();
    paras->ATA.diagonal() = paras->diagonal;
    paras->ATA.selfadjointView<Eigen::Upper>().rankUpdate(paras->A.transpose(), 1.0);

    Eigen::VectorXd ATb = paras->A.transpose()*paras->b;
    yz0.noalias() = paras->ATA.selfadjointView<Eigen::Upper>().ldlt().solve(ATb);
}

void InitRegister::UpdateRtNeutral()
{
#ifdef EYETRACKER
    face_paras->eye_paras->temp_x.resize(5);
    for(int i=0; i<face_paras->eye_paras->temp_x.size(); i++)
    {
        face_paras->eye_paras->temp_x[i].resize(2);
        face_paras->eye_paras->temp_x[i].setZero();
    }
#endif

    //store temp rotation
    face_paras->temp_rotation.resize(5);
    face_paras->temp_translation.resize(5);
    for (int i = 0; i < 5; ++i)
    {
        face_paras->temp_rotation[i] = Eigen::Quaterniond(face_paras->RT.rotation());
        face_paras->temp_translation[i] = face_paras->RT.translation();
    }
    face_paras->RT_smooth = face_paras->RT;

    //expression
    face_paras->y = yz0.topRows(paras->num_y);
    face_paras->z0 = yz0.bottomRows(paras->num_z0);
    face_paras->neutral_expression.resize(face_paras->mean_neutral.size(), 1);
    face_paras->neutral_expression = face_paras->mean_neutral;
    face_paras->neutral_expression.noalias() += face_paras->P_eigenvectors.transpose()
            *yz0.topRows(paras->num_y);
    face_paras->neutral_expression.noalias() += face_paras->E_eigenvectors.transpose()
            *yz0.bottomRows(paras->num_z0);


    face_paras->expression = face_paras->neutral_expression;
    face_paras->expression.resize(3, face_paras->expression.size()/3);

    Eigen::Matrix3d R = face_paras->RT.rotation();
    Eigen::Matrix3d R_transpose = R.transpose();
    face_paras->RT_expression.noalias() = R_transpose.transpose() * face_paras->expression;
    face_paras->RT_expression.colwise() += face_paras->RT.translation();
}

void InitRegister::UpdateRegisteredRtNeutral()
{
    Eigen::Matrix3d R = face_paras->RT.rotation();
    Eigen::Matrix3d R_transpose = R.transpose();
    Eigen::Vector3d T = face_paras->RT.translation();

    paras->V.resize(3*paras->init_registerIndex_size, 1);
    paras->V = paras->init_mean_neutral;
    paras->V.noalias() += face_paras->P_eigenvectors.leftCols(3*paras->init_registerIndex_size)
            .transpose() * yz0.topRows(paras->num_y);
    paras->V.noalias() += face_paras->E_eigenvectors.leftCols(3*paras->init_registerIndex_size)
            .transpose() * yz0.bottomRows(paras->num_z0);
    paras->V.resize(3, paras->init_registerIndex_size);
    paras->temp_init_neutral = paras->V;
    paras->V.noalias() = (R_transpose.transpose()*paras->temp_init_neutral);
    paras->V.colwise() += T;

    //update mesh
    int vIndex = 0;
    PolyMesh::VertexIter vIt;
    for (vIt = face_paras->expression_mesh->vertices_begin(); vIndex < paras->init_registerIndex_size; ++vIt, ++vIndex)
    {
        face_paras->expression_mesh->point(vIt) = PolyMesh::Point(paras->V.col(vIndex).data());
    }

    //update kepoint on the mesh
    if (face_paras->marker_registerIndex.size() >= 3)
    {
         int num_cols = face_paras->marker_registerIndex.size();
         paras->V_marker.resize(3, num_cols);
         #pragma omp parallel for
         for (int i = 0; i < face_paras->marker_registerIndex.size(); ++i)
         {
             int index1 = face_paras->marker_registerIndex[i];
             paras->V_marker.col(i) = face_paras->mean_neutral.block(index1*3, 0, 3, 1);
             paras->V_marker.col(i) += face_paras->P_eigenvectors.middleCols(index1*3, 3)
                     .transpose()*yz0.topRows(paras->num_y);
             paras->V_marker.col(i) += face_paras->E_eigenvectors.middleCols(index1*3, 3)
                     .transpose()*yz0.bottomRows(paras->num_z0);
         }
         paras->V_marker = R_transpose.transpose() * paras->V_marker;
         paras->V_marker.colwise() += T;
    }
}

void InitRegister::FindNearestKdtree()
{
    #pragma omp parallel for
    for(int i=0; i<paras->init_registerIndex.size(); ++i)
    {
        int id = face_paras->pointCloudMesh->kdtree->closest(paras->V.col(i).data());
        paras->Qp.col(i) = face_paras->pointCloudMesh->vertices.col(id);
        paras->Qn.col(i) = face_paras->pointCloudMesh->normals.col(id);
    }
    paras->weight_correspond = ( paras->Qn.array()*(paras->V - paras->Qp).array())
                                .colwise().sum().abs().transpose();
    ICP::robust_weight(ICP::LESS, paras->weight_correspond, paras->registerRate);

    Eigen::Matrix3d M33;
    for (int i = 0; i < paras->init_face_index.size(); i += 4)
    {
        int i0 = paras->init_face_index[i];
        int i1 = paras->init_face_index[i+1];
        int i2 = paras->init_face_index[i+2];
        int i3 = paras->init_face_index[i+3];
        M33.col(0) = paras->V.col(i0);
        M33.col(1) = paras->V.col(i1);
        M33.col(2) = paras->V.col(i2);
        if (M33.determinant() >= 0)
        {
            paras->weight_correspond[i0] = 0.0;
            paras->weight_correspond[i1] = 0.0;
            paras->weight_correspond[i2] = 0.0;
            paras->weight_correspond[i3] = 0.0;
        }
    }
}

void InitRegister::FindNearestProjection()
{
    Eigen::Matrix2Xd coors = FacePerformanceFunction::ConvertRealWorldToProjective_Matrix(
                paras->V, face_paras->grayImage, face_paras->fx, face_paras->fy);
    #pragma omp parallel for
    for(int i=0; i<paras->init_registerIndex_size; i++)
    {
        int id = face_paras->inDepthImage.coeff((int)coors(1,i), (int)coors(0,i));
        if(id != -1)
        {
            paras->Qp.col(i) = face_paras->pointCloudMesh->vertices.col(id);
            paras->Qn.col(i) = face_paras->pointCloudMesh->normals.col(id);
        }
        else
        {
            id = face_paras->pointCloudMesh->kdtree->closest(paras->V.col(i).data());
            paras->Qp.col(i) = face_paras->pointCloudMesh->vertices.col(id);
            paras->Qn.col(i) = face_paras->pointCloudMesh->normals.col(id);
        }
    }
    paras->weight_correspond = ( paras->Qn.array()*(paras->V - paras->Qp).array()).colwise().sum().abs().transpose();
    ICP::robust_weight(ICP::LESS, paras->weight_correspond, paras->registerRate);
    //of<<"w ones:"<<paras->weight_correspond.sum()<<' '<<paras->weight_correspond.size()<<std::endl;

    Eigen::Matrix3d M33;
    for (int i = 0; i < paras->init_face_index.size(); i += 4)
    {
        int i0 = paras->init_face_index[i];
        int i1 = paras->init_face_index[i+1];
        int i2 = paras->init_face_index[i+2];
        int i3 = paras->init_face_index[i+3];
        M33.col(0) = paras->V.col(i0);
        M33.col(1) = paras->V.col(i1);
        M33.col(2) = paras->V.col(i2);
        if (M33.determinant() >= 0)
        {
            paras->weight_correspond[i0] = 0.0;
            paras->weight_correspond[i1] = 0.0;
            paras->weight_correspond[i2] = 0.0;
            paras->weight_correspond[i3] = 0.0;
        }
    }
}

void InitRegister::FitMesh()
{
    InitRt_yz0();

    clock_t start;
    clock_t cost_update_all = 0;

    paras->diagonal.resize(paras->num_yz0);
    paras->diagonal.head(paras->num_y) = face_paras->P_eigenvalues.cwiseInverse().cwiseAbs2() * paras->beta1;

    double beta_3_max = paras->beta3*100.0;
    double beta_3_min = paras->beta3;
    for (int iter = 0; iter<paras->outIters; ++iter)
    {
        std::cout<<"initialization iter: "<<iter<<std::endl;

        start = clock();

        ///update vertex positions with new y and z0
        UpdateRegisteredRtNeutral();

        ICP::Parameters paras_icp;
        paras_icp.f = ICP::LESS;
        paras_icp.max_icp = paras->maxInIters;
        paras_icp.p = paras->registerRate;
        Eigen::Map<Eigen::Matrix3Xd> Vp(paras->V.data(), 3, paras->V.cols());

        //compute normal of the mesh
        face_paras->expression_mesh->update_face_normals();
        face_paras->expression_mesh->update_vertex_normals();

        PolyMesh::VertexIter v_it = face_paras->expression_mesh->vertices_begin();
        double * addr_Vn = const_cast<double*>(face_paras->expression_mesh->normal(v_it.handle()).data());
        Eigen::Map<Eigen::Matrix3Xd> Vn(addr_Vn, 3, paras->init_registerIndex_size);

        Eigen::Affine3d RT_Affine;
        //init rigid register with 2d key point
        if (face_paras->marker_registerIndex.size() >= 3)
        {
            int num_cols = face_paras->marker_registerIndex.size();
            Eigen::Map<Eigen::Matrix2Xd > marker_coordinate(face_paras->marker_Coordinates.data(), 2, num_cols);
            //projection point_to_plane add point_to_point
            RT_Affine = ICP::point2plane_and_point2point(Vp, Vn,
                                            face_paras->pointCloudMesh->vertices,
                                            face_paras->pointCloudMesh->normals,
                                            paras->V_marker, marker_coordinate, paras->markerWeight_RT,
                                            paras->init_face_index, face_paras->grayImage,
                                            &(face_paras->inDepthImage), face_paras->fx, face_paras->fy,
                                            paras_icp);

        }
        else
        {
           //projection point_to_plane
           RT_Affine = ICP::point_to_plane(Vp, Vn,
                                          face_paras->pointCloudMesh->vertices,
                                          face_paras->pointCloudMesh->normals,
                                          paras->init_face_index, face_paras->grayImage,
                                          &(face_paras->inDepthImage),
                                          face_paras->fx, face_paras->fy, paras_icp);
        }

        face_paras->RT = RT_Affine * face_paras->RT;

        //update correspondence based on new vertex positions
        FindNearestProjection();//kdtree();

        //update mesh normal
        {
            PolyMesh::VertexIter v_it = face_paras->expression_mesh->vertices_begin();
            double * addr_mesh = const_cast<double*>(face_paras->expression_mesh->point(v_it.handle()).data());
            memcpy(addr_mesh, paras->V.data(), sizeof(double)*paras->init_registerIndex_size);

            face_paras->expression_mesh->update_face_normals();
            face_paras->expression_mesh->update_vertex_normals();
            for(int i=0; i<paras->init_registerIndex_size; i++)
            {
                if(std::fabs(paras->Qn.col(i).dot(Vn.col(i))) < 0.866)
                {
                    paras->weight_correspond[i] = 0.0;
                }
            }
        }
        cost_update_all += clock() - start;

        double temp_beta3 = beta_3_max + (beta_3_min - beta_3_max)*double(iter)/double(paras->outIters - 1);
        paras->diagonal.tail(paras->num_z0) = paras->beta2 * face_paras->E_eigenvalues.cwiseAbs2()
                + temp_beta3 * Eigen::VectorXd::Ones(paras->num_z0);
        Update_yz0();
    }

    of<<"cost update_all: "<<cost_update_all<<std::endl;
    UpdateRtNeutral();
    //calculate to selected point's color
    UpdateRegisteredRtNeutral();
    Eigen::MatrixXd init_upsampled_V(3, face_paras->nonrigid_paras->init_registerIndex_upsampled_size);
    memcpy(init_upsampled_V.data(), paras->V.data(), sizeof(double)*paras->V.size());
    init_upsampled_V.rightCols(face_paras->nonrigid_paras->init_registerIndex_upsampled_size - paras->init_registerIndex_size).noalias()
            = paras->V*face_paras->nonrigid_paras->sp_upsampling;
    face_paras->preframe_expression_blendshape_color.resize(1, face_paras->nonrigid_paras->init_registerIndex_upsampled_size);

    #pragma omp parallel for
    for (int i = 0; i < face_paras->nonrigid_paras->init_registerIndex_upsampled_size; ++i)
    {
        face_paras->preframe_expression_blendshape_color.col(i) = FacePerformanceFunction::Compute_f_color(init_upsampled_V.col(i), face_paras->grayImage, face_paras->fx, face_paras->fy);
    }
    std::cout<<"after init Register:"<<std::endl;
}

void InitRegister::Blendshape2Trimesh(TriMesh_Eigen& tmesh)
{
    tmesh.vertices.resize(3, face_paras->RT_expression.size()/3);
    memcpy(tmesh.vertices.data(), face_paras->RT_expression.data(), sizeof(double)*face_paras->RT_expression.size());
}

void InitRegister::LoadEigenMatrix(Eigen::MatrixXd& mat, std::string filename)
{
    FILE * fin = fopen(filename.c_str(), "rb");
    int m,n;
    fread(&m, sizeof(int), 1, fin);
    fread(&n, sizeof(int), 1, fin);
    mat.resize(m, n);
    fread(mat.data(), sizeof(double), m*n, fin);
    fclose(fin);
}
void InitRegister::LoadEigenVector(Eigen::VectorXd& vec, std::string filename)
{
    FILE * fin = fopen(filename.c_str(),"rb");
    int m;
    fread(&m, sizeof(int), 1, fin);
    vec.resize(m);
    fread(vec.data(), sizeof(double), m, fin);
    fclose(fin);
}
void InitRegister::LoadEigenMatrix_gl(Eigen::MatrixXd& mat, std::string filename)
{
    FILE * fin = fopen(filename.c_str(), "rb");
    int m,n;
    fread(&m, sizeof(int), 1, fin);
    fread(&n, sizeof(int), 1, fin);
    Eigen::MatrixXd temp_mat(m, n);
    fread(temp_mat.data(), sizeof(double), m*n, fin);

    mat.resize(m*3, n*3);
    mat.setZero();
    //std::string mesh_filename = "test.obj";
    PolyMesh mesh;
    mesh.request_face_normals();
    mesh.request_vertex_normals();
    if(!OpenMesh::IO::read_mesh(mesh, ConfigParameter::template_mesh_filename))
    {
        std::cout<<"read template mesh error"<<std::endl;
        return;
    }
    mesh.update_face_normals();
    mesh.update_vertex_normals();
    for(int i=0; i<m; i++)
    {
        PolyMesh::VertexIter v_it = mesh.vertices_begin();
        for(int j=0; j<n; j++,++v_it)
        {
            PolyMesh::Normal n = mesh.normal(v_it.handle());
            for(int k=0; k<3; k++)
            {
                mat(3*i+k, j*3+k) = temp_mat(i,j);
            }
        }
    }
}

void InitRegister::LoadEigenVector_gl(Eigen::VectorXd& vec, std::string filename)
{
    FILE * fin = fopen(filename.c_str(), "rb");
    int m;
    fread(&m, sizeof(int), 1, fin);
    Eigen::VectorXd temp_vec(m);
    fread(temp_vec.data(), sizeof(double), m, fin);
    vec.resize(m*3);
    for(int i=0; i<m; i++)
    {
        for(int j=0; j<3; j++)
        {
            vec(3*i+j) = temp_vec(i);
        }
    }
    fclose(fin);
}
