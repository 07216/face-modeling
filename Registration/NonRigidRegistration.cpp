#include "NonRigidRegistration.h"

NonRigidRegister::NonRigidRegister(FacePerformanceParameter* face_paras)
{
    omp_set_num_threads(16);

    this->face_paras = face_paras;
    this->paras = face_paras->nonrigid_paras;
#ifdef EYETRACKER
    this->eye_paras = face_paras->eye_paras;
#endif
    if (paras->num_frame == 0)
        of.open("error_NonRigidRegister.txt");
    else
        of.open("error_NonRigidRegister.txt",std::ios::app);
}

NonRigidRegister::~NonRigidRegister()
{
}

void NonRigidRegister::Init()
{
    if (paras->num_frame == 0)//first frame
	{
		paras->num_x = paras->pre_frame_x.size();
        face_paras->x = paras->pre_frame_x;
        paras->m_channels = 1;
        paras->R_init_b0.resize(3, paras->init_registerIndex_upsampled_size);
        paras->R_init_b0_Qp_T.resize(3, paras->init_registerIndex_upsampled_size);
        paras->R_init_b0_V_T.resize(3, paras->init_registerIndex_upsampled_size);
        paras->nt_R.resize(3, paras->init_registerIndex_upsampled_size);
        paras->nt_derive_Jf.resize(3, paras->init_registerIndex_upsampled_size);
        paras->init_expression.resize(3, paras->init_registerIndex_size);

        paras->V.resize(3, paras->init_registerIndex_upsampled_size);
        paras->Qp.resize(3, paras->init_registerIndex_upsampled_size);
        paras->Qn.resize(3, paras->init_registerIndex_upsampled_size);
        paras->weight_coorespond.resize(paras->init_registerIndex_upsampled_size);
        paras->J.resize(paras->init_registerIndex_upsampled_size, paras->m_channels);
        paras->derive_Jf.resize(3, paras->init_registerIndex_upsampled_size);

        //A_3d, b_3d, A_2d, b_2d passing to refinement
        paras->A_3d.resize(3, paras->init_registerIndex_upsampled_size);
        paras->b_3d.resize(paras->init_registerIndex_upsampled_size);
        paras->A_2d.resize(3, paras->init_registerIndex_upsampled_size);
        paras->b_2d.resize(paras->init_registerIndex_upsampled_size);
	}

    clock_t start = clock();
    paras->R = face_paras->RT_smooth.rotation();
    Eigen::Matrix3d R_transpose = paras->R.transpose();
    int vNumber = face_paras->neutral_expression.size()/3;
    face_paras->neutral_expression.resize(3, vNumber);
    paras->R_init_b0.leftCols(paras->init_registerIndex_size).noalias()
            = R_transpose.transpose() *
            face_paras->neutral_expression.block(0,
                                                 0,
                                                 3,
                                                 paras->init_registerIndex_size);
    //upsampling
    if(paras->init_registerIndex_upsampled_size != paras->init_registerIndex_size)
        paras->R_init_b0.rightCols(paras->init_registerIndex_upsampled_size-paras->init_registerIndex_size).noalias()
            = paras->R_init_b0.leftCols(paras->init_registerIndex_size) * paras->sp_upsampling;

    face_paras->neutral_expression.resize(face_paras->neutral_expression.size(), 1);

    paras->init_upsampled_deltaB.resize(3, paras->init_registerIndex_upsampled_size * face_paras->delta_B.cols());

    #pragma omp parallel for
    for (int i = 0; i < face_paras->delta_B.cols(); ++i)
    {
        memcpy( &paras->init_upsampled_deltaB(0, paras->init_registerIndex_upsampled_size*i),
                & (face_paras->delta_B(0, i)),
                sizeof(double)*paras->init_registerIndex_size*3);
        paras->init_upsampled_deltaB.middleCols(i*paras->init_registerIndex_upsampled_size+paras->init_registerIndex_size, paras->init_registerIndex_upsampled_size - paras->init_registerIndex_size).noalias()
                = paras->init_upsampled_deltaB.middleCols(i*paras->init_registerIndex_upsampled_size, paras->init_registerIndex_size) * paras->sp_upsampling;
    }

    of<<"prepare cost: "<<(clock() - start)*1.0/CLOCKS_PER_SEC<<std::endl;

    update_expression_cost = 0.0;
    computer_A_b_cost = 0.0;
    find_nearest_cost = 0.0;
    find_nearest_precompute_cost = 0.0;
    find_nearest_project_cost = 0.0;
    lasso_cost = 0.0;

    _3d_cost = 0.0;
    _3d_basis_ = 0.0;
    _2d_cost = 0.0;
    _2d_basis_ = 0.0;
    smooth_cost = 0.0;
    marker_cost = 0.0;
}

void NonRigidRegister::UpdateAll()
{
    clock_t start,end;
    start = clock();
    FindNearestProjection();//Kdtree();//
    end = clock();
    find_nearest_cost += (end-start)*1.0/CLOCKS_PER_SEC;

    start = clock();
    Compute_A_b();
    end = clock();
    computer_A_b_cost += (end-start)*1.0/CLOCKS_PER_SEC;
}

void NonRigidRegister::UpdateRtExpression()
{
    clock_t start = clock();
    face_paras->expression.resize(face_paras->neutral_expression.size(), 1);
    face_paras->RT_expression.resize(3, face_paras->neutral_expression.size()/3);
    Eigen::Matrix3d R_transpose = paras->R.transpose();
    face_paras->expression.noalias() = face_paras->neutral_expression;
    for (int i = 0; i < paras->num_x; ++i)
    {
        if (face_paras->x[i] != 0.0)
            face_paras->expression.noalias() += face_paras->delta_B.col(i) * face_paras->x[i];
    }
#ifdef EYETRACKER
    TrackEye();
#endif
    face_paras->expression.resize(3, face_paras->neutral_expression.size()/3);
    face_paras->RT_expression.noalias() = R_transpose.transpose() * face_paras->expression;
    face_paras->RT_expression.colwise() += face_paras->RT_smooth.translation();

    face_paras->expression.resize(face_paras->neutral_expression.size(), 1);
    face_paras->RT_expression.resize(face_paras->neutral_expression.size(), 1);
    of<<"update expression cost: "<<clock()-start<<std::endl;
}
#ifdef EYETRACKER
double getRate(cv::Point2d *p, int num)
{
    double width = std::fabs(p[0].x-p[3].x);
    double height = std::fabs(p[1].y-p[5].y);
    if(width <= 0.0l)return 0.0;
    else return height/width;
}

void NonRigidRegister::TrackEye()
{
    /*std::cout<<"before TrackEye"<<std::endl;
    int num_points = eye_paras->eye_keypoint_2d.size();
    std::cout<<eye_paras->x.size()<<std::endl;
    if(num_points == 0)
    {
        eye_paras->x.setZero();
        return;
    }
    eye_paras->R_delta_B_eye_kepoint_part.resize(num_points*3, eye_paras->delta_B.cols());
//    std::cout<<"size: "<<eye_paras->R_delta_B_eye_kepoint_part.rows()<<' '<<eye_paras->R_delta_B_eye_kepoint_part.cols()<<std::endl;
//    std::cout<<eye_paras->delta_B.rows()<<' '<<eye_paras->delta_B.cols()<<std::endl;

//    for(int i=0; i<eye_paras->eye_keypoint_index_3d.size(); i++)
//    {
//        std::cout<<eye_paras->eye_keypoint_index_3d[i]<<' ';
//    }
//    std::cout<<std::endl;
    eye_paras->RT_expression_eye_keypoit_part.resize(num_points*3, 1);

    std::cout<<"deltaB"<<std::endl;
    for(int i=0; i<num_points; i++)
    {
        int index= eye_paras->eye_keypoint_index_3d[i];
        eye_paras->R_delta_B_eye_kepoint_part.middleRows(i*3, 3) = paras->R * eye_paras->delta_B.middleRows(index*3, 3);
        eye_paras->RT_expression_eye_keypoit_part.middleRows(i*3, 3) = paras->R * face_paras->expression.middleRows(index*3, 3);
//        std::cout<<paras->R * eye_paras->delta_B.middleRows(index*3, 3)<<std::endl;
    }
//    std::cout<<std::endl;
    eye_paras->RT_expression_eye_keypoit_part.resize(3, num_points);
    eye_paras->RT_expression_eye_keypoit_part.colwise() += face_paras->RT_smooth.translation();

    ///save to file
//   std::cout<<"key_point correspond"<<std::endl;
//    for(int i=0; i<num_points; i++)
//    {
//        std::cout<<eye_paras->eye_keypoint_2d[i].x<<' '<<eye_paras->eye_keypoint_2d[i].y<<std::endl;
//    }
//    std::cout<<std::endl;
//    for(int i=0; i<num_points; i++)
//    {
//        Eigen::Vector2i v = FacePerformanceFunction::ConvertRealWorldToProjective(eye_paras->RT_expression_eye_keypoit_part.col(i), face_paras->colorImage, face_paras->fx, face_paras->fy);
//        std::cout<<v(0)<<' '<<v(1)<<std::endl;
//    }
//    std::cout<<std::endl;
//    for(int i=0; i<num_points; i++)
//    {
//        std::cout<<"v";
//        for(int j=0; j<3; j++)
//            std::cout<<" "<<eye_paras->RT_expression_eye_keypoit_part(j,i);
//        std::cout<<std::endl;
//    }
    Eigen::MatrixXd A(num_points*2, eye_paras->num_blendshape);
    Eigen::VectorXd b(num_points*2);
    for(int i=0; i<num_points; i++)
    {
        cv::Point2d& p = eye_paras->eye_keypoint_2d[i];
        double vx = face_paras->fx * (0.5 - p.x / face_paras->colorImage.cols);
        double vy = face_paras->fy * (p.y / face_paras->colorImage.rows - 0.5);

        A.row(i) = vx * eye_paras->R_delta_B_eye_kepoint_part.row(2) - eye_paras->R_delta_B_eye_kepoint_part.row(0);
        b(i) = eye_paras->RT_expression_eye_keypoit_part(0, i) - vx * eye_paras->RT_expression_eye_keypoit_part(2, i);
        A.row(i+num_points) = vy * eye_paras->R_delta_B_eye_kepoint_part.row(2) - eye_paras->R_delta_B_eye_kepoint_part.row(1);
        b(i+num_points) = eye_paras->RT_expression_eye_keypoit_part(1, i) - vy * eye_paras->RT_expression_eye_keypoit_part(2, i);
    }
    //solve with lasso
    if(paras->num_frame == 0)
    {
        eye_paras->x.setZero();
    }

//    std::cout<<"track eye norm: "<<A.norm()<<' '<<b.norm()<<std::endl;
//    std::cout<<"R:"<<std::endl;
//    std::cout<<paras->R<<std::endl;
//    std::cout<<"R_delta_B_eye_kepoint_part: "<<std::endl<<eye_paras->R_delta_B_eye_kepoint_part<<std::endl;
//    std::cout<<"A:"<<std::endl<<A<<std::endl;
//    std::cout<<"b:"<<b.transpose()<<std::endl;
//    std::cout<<"x:"<<eye_paras->x.transpose()<<std::endl;
    std::cout<<"before lasso eye: "<<(A*eye_paras->x-b).norm()<<std::endl;
    std::cout<<eye_paras->x.transpose()<<std::endl;
    Lasso(A, b, 0.0001, eye_paras->x, 1.0e-8);
    std::cout<<"after lasso eye: "<<(A*eye_paras->x-b).norm()<<std::endl;
    std::cout<<eye_paras->x.transpose()<<std::endl;
//    Eigen::MatrixXd ATA = A.transpose()*A;
//    Eigen::VectorXd ATb = A.transpose()*b;
//    eye_paras->x = ATA.ldlt().solve(ATb);
    for(int i=0; i<eye_paras->x.size(); i++)
    {
        if(eye_paras->x[i]<0.0) eye_paras->x[i] = 0.0;
        else if(eye_paras->x[i]>1.0)eye_paras->x[i] =  1.0;
    }
    std::cout<<"after lasso eye: "<<eye_paras->x.transpose()<<std::endl;
    //add to expression
    for(int i=0; i<eye_paras->num_blendshape; i++)
    {
        if(eye_paras->x[i] != 0.0)
            face_paras->expression += eye_paras->delta_B.col(i) * eye_paras->x[i];
    }
    std::cout<<"after TrackEye"<<std::endl;*/

    int num_points = eye_paras->eye_keypoint_2d.size();
    if(num_points == 0)
    {
        eye_paras->x.setZero();
        return;
    }
    const double max_height_width_area = 0.19;
    eye_paras->x.setZero();

    double left_eye_area = getRate(&(eye_paras->eye_keypoint_2d[0]), num_points/2);
    if(left_eye_area < max_height_width_area)
        eye_paras->x[1] = 1.0;
    double right_eye_area = getRate(&(eye_paras->eye_keypoint_2d[num_points/2]), num_points/2);
    if(right_eye_area < max_height_width_area)
        eye_paras->x[0] = 1.0;

    Eigen::VectorXd eye2 = eye_paras->x.topRows(2);
    double max_norm = 0.0;
    for(int i = 0; i < 5; ++i)
    {
        double norm = (eye_paras->temp_x[i]-eye2).norm();
        if(norm>max_norm) max_norm = norm;
    }

    //std::cout<<"max_norm:"<<max_norm<<std::endl;
    double sum_w = 1.0;
    Eigen::VectorXd temp_eye = eye2;
    for(int i = 0; i < 5; ++i)
    {
        double w = std::exp(-(i+1)*0.1*max_norm);
        temp_eye += eye_paras->temp_x[i] * w;
        sum_w += w;
    }
    temp_eye *= (1.0/sum_w);

    eye_paras->x.topRows(2) = temp_eye;
    //add to expression
    for(int i=0; i<eye_paras->num_blendshape; i++)
    {
        if(eye_paras->x[i] != 0.0)
            face_paras->expression += eye_paras->delta_B.col(i) * eye_paras->x[i];
    }

    for(int i=1; i<5; i++)
    {
        eye_paras->temp_x[i] = eye_paras->temp_x[i-1];
    }
    eye_paras->temp_x[0] = eye2;

    std::cout<<"eye :"<<temp_eye.transpose()<<std::endl;

}
#endif

void NonRigidRegister::FindNearestKdtree()
{
    clock_t start = clock();

    face_paras->neutral_expression.resize(face_paras->neutral_expression.size(), 1);
    Eigen::MatrixXd init_expression = face_paras->neutral_expression.topRows(paras->init_registerIndex_size*3);
    for(int i=0; i<face_paras->x.size(); i++)
    {
        if(face_paras->x[i] != 0.0)
            init_expression +=
                    face_paras->delta_B.block(0, i, paras->init_registerIndex_size*3, 1)
                    * face_paras->x[i];
    }

    init_expression.resize(3, paras->init_registerIndex_size);
    Eigen::MatrixXd R_transpose = paras->R.transpose();
    paras->V.leftCols(paras->init_registerIndex_size).noalias()
            = R_transpose.transpose() * init_expression;
    paras->V.leftCols(paras->init_registerIndex_size).colwise()
            += face_paras->RT_smooth.translation();
    //upsampling paras->V
    if(paras->init_registerIndex_upsampled_size != paras->init_registerIndex_size)
    paras->V.rightCols(paras->init_registerIndex_upsampled_size - paras->init_registerIndex_size).noalias()
           = paras->V.leftCols(paras->init_registerIndex_size) * paras->sp_upsampling;

    find_nearest_precompute_cost += (clock()-start)*1.0/CLOCKS_PER_SEC;

    start = clock();

    #pragma omp parallel for
    for(int i=0; i<paras->init_registerIndex_upsampled_size; i++)
    {
        int id = face_paras->pointCloudMesh->kdtree->closest(paras->V.col(i).data());
        paras->Qp.col(i) = face_paras->pointCloudMesh->vertices.col(id);//pointCloud_Y.col(id);
        paras->Qn.col(i) = face_paras->pointCloudMesh->normals.col(id);//for(int j=0; j<3; j++) paras->Qn(j, i) = paras->pointCloudMesh->normals[id][j];

        //derive_Jf
        if(paras->colorWeight != 0.0)
        {
            Eigen::MatrixXd derive_project_matrix = FacePerformanceFunction::ComputeDerive_f(paras->V.col(i), face_paras->grayImage, face_paras->fx, face_paras->fy);
            Eigen::MatrixXd derive_J = FacePerformanceFunction::Compute_Derive_Color(paras->V.col(i), face_paras->grayImage, face_paras->fx, face_paras->fy);
            paras->derive_Jf.col(i).noalias() = derive_project_matrix.transpose() * derive_J.transpose();
            paras->J.row(i) = FacePerformanceFunction::Compute_f_color(paras->V.col(i), face_paras->grayImage, face_paras->fx, face_paras->fy).transpose();
        }
    }
    paras->weight_coorespond = ((paras->V - paras->Qp).array() * paras->Qn.array()).colwise().sum().abs().transpose();
    ICP::robust_weight(ICP::LESS, paras->weight_coorespond, paras->registerRate);

    find_nearest_project_cost += (clock()-start)*1.0/CLOCKS_PER_SEC;
}

void NonRigidRegister::FindNearestProjection()
{
    clock_t start = clock();

    face_paras->neutral_expression.resize(face_paras->neutral_expression.size(), 1);
    Eigen::MatrixXd init_expression = face_paras->neutral_expression.topRows(paras->init_registerIndex_size*3);
    for (int i = 0; i < face_paras->x.size(); ++i)
    {
        if (face_paras->x[i] != 0.0)
            init_expression.noalias() += face_paras->delta_B.block(0, i, paras->init_registerIndex_size*3, 1)*face_paras->x[i];
    }

    init_expression.resize(3, paras->init_registerIndex_size);
    Eigen::MatrixXd R_transpose = paras->R.transpose();
    paras->V.leftCols(paras->init_registerIndex_size).noalias() = R_transpose.transpose() * init_expression;
    paras->V.leftCols(paras->init_registerIndex_size).colwise() += face_paras->RT_smooth.translation();
    //upsampling paras->V
    if(paras->init_registerIndex_upsampled_size != paras->init_registerIndex_size)
    paras->V.rightCols(paras->init_registerIndex_upsampled_size - paras->init_registerIndex_size).noalias()
           = paras->V.leftCols(paras->init_registerIndex_size) * paras->sp_upsampling;

    find_nearest_precompute_cost += (clock() - start)*1.0/CLOCKS_PER_SEC;

    start = clock();
    Eigen::Matrix2Xd coors = FacePerformanceFunction::ConvertRealWorldToProjective_Matrix(paras->V, face_paras->grayImage, face_paras->fx, face_paras->fy);

    #pragma omp parallel for
    for (int i = 0; i < paras->init_registerIndex_upsampled_size; ++i)
    {
        int id = face_paras->inDepthImage.coeff((int)coors(1,i), (int)coors(0,i));
        if (id != -1)
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
        //derive_Jf
        if (paras->colorWeight != 0.0)
        {
            Eigen::Vector2i coor;
            coor(0) = coors(0,i);
            coor(1) = coors(1,i);
            Eigen::MatrixXd derive_project_matrix = FacePerformanceFunction::ComputeDerive_f(paras->V.col(i), face_paras->grayImage, face_paras->fx, face_paras->fy);
            Eigen::MatrixXd derive_J = FacePerformanceFunction::Compute_Derive_Color(coor, face_paras->grayImage);
            paras->derive_Jf.col(i).noalias() = derive_project_matrix.transpose() * derive_J.transpose();
            paras->J.row(i) = FacePerformanceFunction::Compute_f_color(coor, face_paras->grayImage);
        }
    }
    paras->weight_coorespond = ((paras->V - paras->Qp).array() * paras->Qn.array()).colwise().sum().abs().transpose();
    ICP::robust_weight(ICP::LESS, paras->weight_coorespond, paras->registerRate);

    Eigen::Matrix3d M33(3,3);
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
            paras->weight_coorespond[i0] = 0.0;
            paras->weight_coorespond[i1] = 0.0;
            paras->weight_coorespond[i2] = 0.0;
            paras->weight_coorespond[i3] = 0.0;
        }
    }

    find_nearest_project_cost += (clock()-start)*1.0/CLOCKS_PER_SEC;
}

void NonRigidRegister::AfterFit()
{
    paras->ppre_frame_x = paras->pre_frame_x;
    paras->pre_frame_x = face_paras->x;
    paras->num_frame ++;

    // compute expression and RT_expression

    int m_channels = paras->m_channels;
    face_paras->preframe_expression_blendshape_color.resize(m_channels, paras->init_registerIndex_upsampled_size);
    paras->mapPoint2Image.resize(2, face_paras->meshMarkerPointIndex.size());

   #pragma omp parallel for
    for (int i = 0; i < face_paras->meshMarkerPointIndex.size(); ++i)
    {
        int index = face_paras->meshMarkerPointIndex[i];
        paras->mapPoint2Image.col(i) = FacePerformanceFunction::ConvertRealWorldToProjective(face_paras->RT_expression.block(index*3,0,3,1), face_paras->grayImage, face_paras->fx, face_paras->fy);
    }
}

void NonRigidRegister::FitMeshLasso()
{
    Init();
    for (int i = 0; i < paras->outIters; ++i)
    {
        UpdateAll();

        clock_t start = clock();
        if (paras->num_frame == 0 && i == 0)
        {
            face_paras->x.setZero();
        }

        Lasso(paras->A, paras->unchanged_b, paras->beta, face_paras->x);

        clock_t end = clock();
        lasso_cost += (end-start)*1.0/CLOCKS_PER_SEC;
    }

    UpdateRtExpression();
    //update mesh
    face_paras->RT_expression.resize(3, face_paras->RT_expression.size()/3);
    PolyMesh::VertexIter v_it = face_paras->expression_mesh->vertices_begin();
    memcpy(face_paras->expression_mesh->point(v_it).data(), face_paras->RT_expression.data(), sizeof(double)*face_paras->RT_expression.size());

//    of<<"Final x: "<<face_paras->x.transpose()<<std::endl;
//    of<<std::endl;

    of<<"Frame"<<paras->num_frame<<" cost:"<<std::endl;
    of<<"computer_A_b: "<<computer_A_b_cost<<std::endl;
    of<<"----3d cost: "<<_3d_cost<<std::endl;
    of<<"----3d basis: "<<_3d_basis_<<std::endl;
    of<<"----2d cost: "<<_2d_cost<<std::endl;
    of<<"----smooth cost: "<<smooth_cost<<std::endl;
    of<<"----marker cost: "<<marker_cost<<std::endl;

    of<<"update_expression: "<<update_expression_cost<<std::endl;
    of<<"find nearest: "<<find_nearest_cost<<std::endl;
    of<<"----find nearest precompute:"<<find_nearest_precompute_cost<<std::endl;
    of<<"----find nearest project: "<<find_nearest_project_cost<<std::endl;
    of<<"lasso: "<<lasso_cost<<std::endl<<std::endl;
}

//compute matrix A, and the unchanged part of b.
void NonRigidRegister::Compute_A_b()
{
    clock_t start, end;
    //int start_row_A_3d, start_row_A_2d_optical, start_row_A_smooth, start_row_A_marker;
    //rows of different term: 3d, 2d, marker, smooth
    int num_rows_3d, num_rows_2d, num_rows_marker, num_rows_smooth;
    num_rows_3d = paras->init_registerIndex_upsampled_size;
    if (paras->colorWeight != 0.0)
    {
        num_rows_2d = paras->init_registerIndex_upsampled_size;
    }
    else
    {
        num_rows_2d = 0;
    }
    if(paras->markerWeight != 0.0)
    {
        num_rows_marker = face_paras->marker_registerIndex.size()*2;
        //if(num_rows_marker>24) num_rows_marker = 24;
    }
    else
    {
        num_rows_marker = 0;
    }
    num_rows_smooth = paras->num_x;
    //start row of different term
    int start_row_A_3d = 0;
    int start_row_A_2d_optical = start_row_A_3d + num_rows_3d;
    int start_row_A_marker = start_row_A_2d_optical + num_rows_2d;
    int start_row_A_smooth = start_row_A_marker + num_rows_marker;

    int num_rows = num_rows_3d + num_rows_2d + num_rows_marker + num_rows_smooth;
    paras->A.resize( num_rows, paras->num_x );
    paras->unchanged_b.resize(num_rows);

    paras->nt_R = paras->R.transpose() * paras->Qn;
    if(paras->colorWeight != 0.0)
    paras->nt_derive_Jf = paras->R.transpose() * paras->derive_Jf;

    const double weight_3d = 1.0;
    //3d
    start = clock();
    #pragma omp parallel for
    for (int i = 0; i < paras->num_x; ++i)
    {
        paras->A.block(start_row_A_3d, i, paras->init_registerIndex_upsampled_size, 1)
                = (paras->nt_R.array()
                   *paras->init_upsampled_deltaB.block(0,
                                                       i*paras->init_registerIndex_upsampled_size,
                                                       3,
                                                       paras->init_registerIndex_upsampled_size
                                                       ).array()).colwise().sum().transpose() * weight_3d;
        if(paras->colorWeight != 0.0)
        paras->A.block(start_row_A_2d_optical,
                       i,
                       paras->init_registerIndex_upsampled_size,
                       1)
                = (paras->nt_derive_Jf.array()
                   *paras->init_upsampled_deltaB.block(0,
                                                       i*paras->init_registerIndex_upsampled_size,
                                                       3,
                                                       paras->init_registerIndex_upsampled_size
                                                       ).array()).colwise().sum().transpose() * paras->colorWeight;
    }

    end = clock();
    _3d_basis_ += (end-start)*1.0/CLOCKS_PER_SEC;

    end = clock();
    _3d_cost += (end-start)*1.0/CLOCKS_PER_SEC;

    paras->R_init_b0_Qp_T = (paras->R_init_b0 - paras->Qp).colwise() + face_paras->RT_smooth.translation();
    paras->R_init_b0_V_T = (paras->R_init_b0 - paras->V).colwise() + face_paras->RT_smooth.translation();

    //3d
    paras->unchanged_b.block( start_row_A_3d, 0, paras->init_registerIndex_upsampled_size, 1)
            = - (paras->Qn.array()
                 * paras->R_init_b0_Qp_T.array()
                 ).colwise().sum().transpose() * weight_3d;

    //2d optical flow
    if(paras->colorWeight != 0.0)
    {
        paras->unchanged_b.block( start_row_A_2d_optical, 0, paras->init_registerIndex_upsampled_size, 1)
                = -(paras->derive_Jf.array()*paras->R_init_b0_V_T.array()).colwise().sum().transpose();

        paras->unchanged_b.block( start_row_A_2d_optical, 0, paras->init_registerIndex_upsampled_size, 1)
                += face_paras->preframe_expression_blendshape_color.transpose() - paras->J;

        paras->unchanged_b.block( start_row_A_2d_optical, 0, paras->init_registerIndex_upsampled_size, 1)
                *= paras->colorWeight;
    }

    #pragma omp parallel for
    for(int i=0; i<paras->init_registerIndex_upsampled_size; i++)
    {
        if(paras->weight_coorespond(i)==0)
        {
            //3d
            paras->A.block( start_row_A_3d+i, 0, 1, paras->num_x).setZero();
            paras->unchanged_b(start_row_A_3d+i) = 0.0;
            //2d
            if(paras->colorWeight != 0.0)
            {
                paras->A.block( start_row_A_2d_optical+i, 0, 1, paras->num_x).setZero();
                paras->unchanged_b(start_row_A_2d_optical+i) = 0.0;
            }
        }
        if((!(paras->isOpticalIndex[i])) && paras->colorWeight != 0.0)
        {
            paras->A.block( start_row_A_2d_optical+i, 0, 1, paras->num_x).setZero();
            paras->unchanged_b(start_row_A_2d_optical+i) = 0.0;
        }
    }

    //smooth
    start = clock();
    Eigen::VectorXd smooth_diag(paras->num_x);
    smooth_diag.setOnes();
    smooth_diag *= (paras->alpha * paras->weight_coorespond.sum()/paras->weight_coorespond.size());
    paras->A.block(start_row_A_smooth, 0, paras->num_x, paras->num_x) = smooth_diag.asDiagonal();

    paras->unchanged_b.block(start_row_A_smooth, 0, paras->num_x, 1) = (paras->pre_frame_x * 2 - paras->ppre_frame_x)* (paras->alpha * paras->weight_coorespond.sum()/paras->weight_coorespond.size());
    end = clock();
    smooth_cost += (end-start)*1.0/CLOCKS_PER_SEC;

    //marker 2d
    if(paras->markerWeight != 0.0)
    {
        start = clock();
        Eigen::MatrixXd R_transpose = paras->R.transpose();
        for(int i=0; i<num_rows_marker/2; i++)
        {
            int index = face_paras->marker_registerIndex[i];
            Eigen::VectorXd Rb0_T = R_transpose.transpose() * face_paras->neutral_expression.block(index*3, 0, 3,  1) + face_paras->RT_smooth.translation();
            Eigen::MatrixXd RdeltaB = R_transpose.transpose() * face_paras->delta_B.block(index*3, 0, 3, paras->num_x);
            double c0, c1;
            c0 = (face_paras->marker_Coordinates[i*2] - face_paras->grayImage.cols/2.0 ) * face_paras->fx / face_paras->grayImage.cols;
            paras->A.block(start_row_A_marker+i*2, 0, 1, paras->num_x) = (RdeltaB.row(0) + RdeltaB.row(2) * c0) * paras->markerWeight;
            paras->unchanged_b(start_row_A_marker+i*2) = -(Rb0_T(0)+Rb0_T(2)*c0)*paras->markerWeight;

            c1 = (face_paras->grayImage.rows/2.0 - face_paras->marker_Coordinates[i*2+1]) * face_paras->fy / face_paras->grayImage.rows;
            paras->A.block(start_row_A_marker+i*2+1, 0, 1, paras->num_x) = (RdeltaB.row(1) + RdeltaB.row(2) * c1) * paras->markerWeight;
            paras->unchanged_b(start_row_A_marker+i*2+1) = -(Rb0_T(1)+Rb0_T(2)*c1)*paras->markerWeight;
        }
        end = clock();
        marker_cost += (end-start)*1.0/CLOCKS_PER_SEC;
    }
}

void NonRigidRegister::Compute_A_b_to_refinement()
{
    face_paras->expression_mesh->update_face_normals();
    face_paras->expression_mesh->update_vertex_normals();

    FindNearestProjection();//Kdtree();//

    Eigen::Matrix3d R = face_paras->RT_smooth.rotation();
    face_paras->A_3d_to_refinement.resize(3, paras->init_registerIndex_size);
    face_paras->A_3d_to_refinement.noalias()
            = R.transpose() * paras->Qn.leftCols(paras->init_registerIndex_size);

    paras->Qp.leftCols(paras->init_registerIndex_size).colwise()
            -= face_paras->RT_smooth.translation();

    face_paras->b_3d_to_refinement
            = (paras->Qn.leftCols(paras->init_registerIndex_size).array()
               * paras->Qp.leftCols(paras->init_registerIndex_size).array())
            .colwise().sum().transpose();


    if (paras->colorWeight != 0.0)
    {
        face_paras->A_2d_to_refinement.resize(3, paras->init_registerIndex_size);
        face_paras->A_2d_to_refinement.noalias() = R.transpose() * paras->derive_Jf.leftCols(paras->init_registerIndex_size);
        paras->V.leftCols(paras->init_registerIndex_size).colwise() -= face_paras->RT_smooth.translation();
        face_paras->b_2d_to_refinement = (paras->derive_Jf.leftCols(paras->init_registerIndex_size).array() * paras->V.leftCols(paras->init_registerIndex_size).array()).colwise().sum().transpose();
        face_paras->b_2d_to_refinement += paras->J.topRows(paras->init_registerIndex_size) - face_paras->preframe_expression_blendshape_color.leftCols(paras->init_registerIndex_size).transpose();
        face_paras->A_2d_to_refinement *= paras->colorWeight;
        face_paras->b_2d_to_refinement *= paras->colorWeight;
    }

    #pragma omp parallel for
    for (int i=0; i<paras->init_registerIndex_size; i++)
    {
        if (paras->weight_coorespond(i)==0)
        {
            face_paras->A_3d_to_refinement.col(i).setZero();
            face_paras->b_3d_to_refinement(i) = 0.0;
            if(paras->colorWeight != 0.0)
            {
                face_paras->A_2d_to_refinement.col(i).setZero();
                face_paras->b_2d_to_refinement(i) = 0.0;
            }
        }
    }
    face_paras->preframe_expression_blendshape_color = paras->J.transpose();
}



void NonRigidRegister::Lasso(Eigen::MatrixXd& A, Eigen::VectorXd& b, double lambda, Eigen::VectorXd& x, double tolerance)
{
    int p = A.cols();
    bool isFound = false;
    Eigen::VectorXd beta_last;
    Eigen::VectorXd ATb(A.cols(), 1);
    Eigen::MatrixXd ATA(A.cols(), A.cols());
    ATA.setZero();
    ATA.selfadjointView<Eigen::Upper>().rankUpdate(A.transpose(),1.0);
    ATA.triangularView<Eigen::StrictlyLower>() = ATA.transpose();
    ATb.noalias() = A.transpose() * b;
    srand(time(0));
    while (!isFound)
    {
        beta_last = x;

        for (int j = 0; j < p; j++)
        {
            int i = rand() % p;
            double deltai = ATb.coeff(i) - (ATA.col(i)).dot(x) + ATA(i, i)*x.coeff(i);
            if (deltai < -lambda)
            {
                x(i) = (deltai + lambda)/ATA(i, i);
            }
            else if (deltai > lambda)
            {
                x(i) = (deltai - lambda)/ATA(i, i);
            }
            else
            {
                x(i) = 0;
            }

            x(i) = (x.coeff(i) > 1.0)?1.0:x.coeff(i);
            x(i) = (x.coeff(i) < 0)?0:x.coeff(i);
        }
        if ((x - beta_last).cwiseAbs().maxCoeff() < tolerance)
        {
            isFound = true;
        }
    }
}

//X->A, beta->x
void NonRigidRegister::Shotgun(Eigen::MatrixXd& A, Eigen::VectorXd & y, double lambda, Eigen::VectorXd& x, double threshold)
{
    int n_cols = A.cols();
    int n_rows = A.rows();
    Eigen::VectorXd x_last(n_cols);
    //initialization
    Eigen::VectorXd convar = A.colwise().squaredNorm() * 2;
    Eigen::VectorXd Aty = A.transpose() * y * 2;
    Eigen::VectorXd Ax = A * x;

    int counter = 0;
    int regpathlength = 5;
    int regularization_path_length = (regpathlength <= 0 ? 1+(int)(n_cols/2000) : regpathlength);
    //compute max lambda
    double lambda_max = Aty.maxCoeff();
    double lambda_min = lambda;
    double alpha = pow(lambda_max/lambda_min, 1.0/(1.0*regularization_path_length));
    int regularization_path_step = regularization_path_length;
    double delta_threshold = threshold;

    do
    {
        lambda = lambda_min * pow(alpha, regularization_path_step);
        x_last = x;

        //#omp parallel for
        for(int i=0; i<n_cols; i++)
        {
            double oldvalue = x_last.coeff(i);
            double AtAxi = A.col(i).dot(Ax);
            double Si = 2 * AtAxi - convar(i)*oldvalue - Aty.coeff(i);

            double newvalue = 0.0;
            if(Si > lambda) newvalue = lambda - Si;
            if(Si < -lambda) newvalue = -lambda - Si;
            newvalue /= convar.coeff(i);

            double delta = (newvalue - oldvalue);
            if(delta != 0.0)
            {
                Ax += A.col(i) * delta;
                x(i) = newvalue;
            }
            x(i) = (x(i) > 1.0)?1.0:x(i);
            x(i) = (x(i) < 0)?0:x(i);
        }

        counter ++;

        double maxChange =(x - x_last).cwiseAbs().maxCoeff();
        double term_threshold = (regularization_path_step == 0
                                 ? delta_threshold
                                 : (delta_threshold + regularization_path_step*(delta_threshold*50)/regularization_path_length));
        bool converged = (maxChange <= term_threshold);
        if (converged || counter>std::min(100, (100-regularization_path_step)*2))
        {
            counter = 0;
            regularization_path_step--;
        }

    }while(regularization_path_step >= 0);
}

//void NonRigidRegister::Shotgun(Eigen::MatrixXd& X, Eigen::VectorXd & Y, double lambda, Eigen::VectorXd& beta, double threshold)
//{
//    int p = X.cols();
//    Eigen::VectorXd beta_last;
//    Eigen::VectorXd XTY(X.cols(), 1);
//    Eigen::MatrixXd XTX(X.cols(), X.cols());
//    XTX.setZero();
//    XTX.selfadjointView<Eigen::Upper>().rankUpdate(X.transpose(),1.0);
//    XTX = XTX.selfadjointView<Eigen::Upper>();
//    XTY.noalias() = X.transpose() * Y;


//    int counter = 0;
//    int regpathlength = 5;
//    int regularization_path_length = (regpathlength <= 0 ? 1+(int)(X.cols()/2000) : regpathlength);
//    //compute max lambda
//    double lambda_max = XTY.maxCoeff();
//    double lambda_min = lambda;
//    double alpha = pow(lambda_max/lambda_min, 1.0/(1.0*regularization_path_length));
//    int regularization_path_step = regularization_path_length;
//    double delta_threshold = threshold;

//    do
//    {
//        lambda = lambda_min * pow(alpha, regularization_path_step);
//        beta_last = beta;
//        for (int i = 0; i < p; i++)
//        {
//            double deltai = XTY(i) - (XTX.col(i)).dot(beta) + XTX(i, i)*beta(i);
//            if (deltai < -lambda)
//            {
//                beta(i) = (deltai + lambda)/XTX(i, i);
//            }
//            else if (deltai > lambda)
//            {
//                beta(i) = (deltai - lambda)/XTX(i, i);
//            }
//            else
//            {
//                beta(i) = 0;
//            }

//            beta(i) = (beta(i) > 1.0)?1.0:beta(i);
//            beta(i) = (beta(i) < 0)?0:beta(i);
//        }
//        counter ++;

//        double maxChange =(beta - beta_last).cwiseAbs().maxCoeff();
//        double term_threshold = (regularization_path_step == 0
//                                 ? delta_threshold
//                                 : (delta_threshold + regularization_path_step*(delta_threshold*50)/regularization_path_length));
//        bool converged = (maxChange <= term_threshold);
//        if (converged || counter>std::min(100, (100-regularization_path_step)*2))
//        {
//            counter = 0;
//            regularization_path_step--;
//        }

//    }while(regularization_path_step >= 0);
//    //std::cout<<"counter: "<<counter<<std::endl;
//}
