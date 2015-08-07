#include "RigidRegistration.h"

int RigidRegister::flip_num_count = 0;
RigidRegister::RigidRegister(FacePerformanceParameter* face_paras)
{
    omp_set_num_threads(16);
    of.open("error_RigidRegister.txt", std::ios::app);
    this->face_paras = face_paras;
    this->paras = face_paras->rigid_paras;
}

RigidRegister::~RigidRegister()
{
}

void RigidRegister::FitMeshICP()
{
    //compute normal of the mesh
    face_paras->expression_mesh->update_face_normals();
    face_paras->expression_mesh->update_vertex_normals();
    PolyMesh::VertexIter v_it = face_paras->expression_mesh->vertices_begin();
    double* addr_Vn = const_cast<double*> (face_paras->expression_mesh->normal(v_it.handle()).data());
    Eigen::Map<Eigen::Matrix3Xd> Vn(addr_Vn, 3, paras->init_registerIndex_size);

    paras->Vp = face_paras->RT_expression.leftCols(paras->init_registerIndex_size);
    Eigen::Matrix3Xd NoseRegionPoint = face_paras->RT_expression.leftCols(paras->nose_regionIndex_size);

    Eigen::Matrix3Xd& Y = face_paras->pointCloudMesh->vertices;
    Eigen::Matrix3Xd& N = face_paras->pointCloudMesh->normals;

    ICP::Parameters paras_icp;
    paras_icp.f = ICP::LESS;
    paras_icp.max_icp = paras->icpIters;
    paras_icp.p = paras->registerRate;
    Eigen::Affine3d RT_Affine;
    //rigid register with 2d key point
    if (face_paras->marker_registerIndex.size() >= 3)
    {
        int num_cols = face_paras->marker_registerIndex.size();
        Eigen::Matrix3Xd marker_expression(3, num_cols);
        #pragma omp parallel for
        for (int i = 0; i < face_paras->marker_registerIndex.size(); ++i)
        {
            int index = face_paras->marker_registerIndex[i];
            marker_expression.col(i) = face_paras->RT_expression.col(index);
        }
        Eigen::Map<Eigen::Matrix2Xd > marker_coordinate(face_paras->marker_Coordinates.data(), 2, num_cols);
        //projection point_to_plane add point_to_point
        RT_Affine = ICP::point2plane_and_point2point(paras->Vp, Vn, Y, N,
                                        marker_expression, marker_coordinate, paras->markerWeight,
                                        paras->init_face_index, face_paras->grayImage,
                                        &(face_paras->inDepthImage), face_paras->fx, face_paras->fy,
                                        paras_icp);

    }
    else
    {
       //projection point_to_plane
       RT_Affine = ICP::point_to_plane(paras->Vp, Vn, Y, N,
                                       paras->init_face_index, face_paras->grayImage,
                                       &(face_paras->inDepthImage), face_paras->fx, face_paras->fy,
                                       paras_icp);
    }

    NoseRegionPoint = RT_Affine * NoseRegionPoint;
    Eigen::Matrix3Xd& Select_Points = NoseRegionPoint;
    int num_nose_region_point = 0;
    int num_match_nose_regon_point = 0;
    double averge_nose_error = 0.0;

    bool* mark = new bool[Select_Points.cols()];
    memset(mark, true, sizeof(bool)*Select_Points.cols());
    for (int i = 0; i < paras->nose_face_index.size(); i += 4)
    {
        Eigen::MatrixXd M33(3,3);
        int i0 = paras->nose_face_index[i];
        int i1 = paras->nose_face_index[i+1];
        int i2 = paras->nose_face_index[i+2];
        int i3 = paras->nose_face_index[i+3];
        M33.col(0) = Select_Points.col(i0);
        M33.col(1) = Select_Points.col(i1);
        M33.col(2) = Select_Points.col(i2);
        if (M33.determinant() >= 0)
        {
            mark[i0] = false;
            mark[i1] = false;
            mark[i2] = false;
            mark[i3] = false;
        }
    }
    for (int i = 0; i < Select_Points.cols(); ++i)
    {
        if (!mark[i])
            continue;
        int id = face_paras->pointCloudMesh->kdtree->closest(Select_Points.col(i).data());
        if (id == -1)
        {
            paras->tracking_succeed = false;
            paras->isNonrigidUseMarker = false;
            return;
        }
        double dist = (Select_Points.col(i) - Y.col(id)).squaredNorm();
        if (dist < paras->max_nose_error)
            num_match_nose_regon_point++;
        averge_nose_error += dist;
        num_nose_region_point++;
    }
    if (mark != NULL)
        delete [] mark;

    if (num_nose_region_point != 0)
        averge_nose_error /= num_nose_region_point;
    averge_nose_error = std::sqrt(averge_nose_error);

    if (num_nose_region_point > 10 && (double (num_match_nose_regon_point))/ num_nose_region_point > 0.5)
    {
        face_paras->RT = RT_Affine * face_paras->RT;
        paras->tracking_succeed = true;
        //compute smooth RT
        Eigen::Quaterniond temp_r(face_paras->RT.rotation());
        Eigen::VectorXd temp_t = face_paras->RT.translation();
        double max_norm_r = 0.0;
        double max_norm_t = 0.0;
        for(int i = 0; i < 5; ++i)
        {
            double norm_r = 0.0;
            norm_r += (temp_r.w()-face_paras->temp_rotation[i].w()) * (temp_r.w()-face_paras->temp_rotation[i].w());
            norm_r += (temp_r.x()-face_paras->temp_rotation[i].x()) * (temp_r.x()-face_paras->temp_rotation[i].x());
            norm_r += (temp_r.y()-face_paras->temp_rotation[i].y()) * (temp_r.y()-face_paras->temp_rotation[i].y());
            norm_r += (temp_r.z()-face_paras->temp_rotation[i].z()) * (temp_r.z()-face_paras->temp_rotation[i].z());

            norm_r = std::sqrt(norm_r);
            double norm_t = (temp_t-face_paras->temp_translation[i]).norm();
            if(norm_r>max_norm_r) max_norm_r = norm_r;
            if(norm_t>max_norm_t) max_norm_t = norm_t;
        }
        double sum_w_r = 1.0;
        double sum_w_t = 1.0;

        for(int i = 0; i < 5; ++i)
        {
            double w_r = std::exp(-(i+1)*face_paras->H_rotation*max_norm_r);
            double w_t = std::exp(-(i+1)*face_paras->H_translation*max_norm_t);
            temp_r.w() += face_paras->temp_rotation[i].w() * w_r;
            temp_r.x() += face_paras->temp_rotation[i].x() * w_r;
            temp_r.y() += face_paras->temp_rotation[i].y() * w_r;
            temp_r.z() += face_paras->temp_rotation[i].z() * w_r;

            temp_t += face_paras->temp_translation[i] * w_t;
            sum_w_r += w_r;
            sum_w_t += w_t;
        }
        temp_r.w() *= (1.0/sum_w_r);
        temp_r.x() *= (1.0/sum_w_r);
        temp_r.y() *= (1.0/sum_w_r);
        temp_r.z() *= (1.0/sum_w_r);
        temp_t *= (1.0/sum_w_t);

        face_paras->RT_smooth = temp_r;
        face_paras->RT_smooth.translation() = temp_t;
        for (int i = 1; i < 5; ++i)
        {
            face_paras->temp_rotation[i] = face_paras->temp_rotation[i-1];
            face_paras->temp_translation[i] = face_paras->temp_translation[i-1];
        }
        face_paras->temp_rotation[0] = temp_r;
        face_paras->temp_translation[0] = temp_t;
    }
    else
    {
        paras->tracking_succeed = false;
        static const float mult = 0.0174532925;
        flip_num_count ++;
        if (flip_num_count < 5)
        {
            if (face_paras->poseEstimation->estimate_pose(face_paras->pointCloudMesh->img3D_pose))
            {
                int idx = face_paras->poseEstimation->best_cluster_index;
                double rx = -face_paras->poseEstimation->g_means[idx][3]*mult;
                double ry = face_paras->poseEstimation->g_means[idx][4]*mult;
                double rz = face_paras->poseEstimation->g_means[idx][5]*mult;

                //Eigen::Affine3d temp_RT = face_paras->RT;
                face_paras->RT = Eigen::AngleAxisd(-rx, Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(-ry, Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(-rz, Eigen::Vector3d::UnitZ());
                face_paras->RT.translation()[0] = face_paras->poseEstimation->g_means[idx][0];
                face_paras->RT.translation()[1] = -face_paras->poseEstimation->g_means[idx][1];
                face_paras->RT.translation()[2] = -face_paras->poseEstimation->g_means[idx][2];

                //update RT_expression
                Eigen::Map<Eigen::Matrix3Xd > expression(face_paras->expression.data(), 3, face_paras->expression.size()/3);
                face_paras->RT_expression.noalias() = face_paras->RT.rotation() * expression;
                face_paras->RT_expression.colwise() += face_paras->RT.translation();

                //reset smooth RT
                for (int i = 0; i < 5; ++i)
                {
                    face_paras->temp_rotation[i] = Eigen::Quaterniond(face_paras->RT.rotation());
                    face_paras->temp_translation[i] = face_paras->RT.translation();
                }
            }
        }
        else
        {
            flip_num_count = 0;
            if (face_paras->poseEstimation->detect_face_from_depthimage(face_paras->depthImage,
                                                                    face_paras->fx, face_paras->fy,
                                                                    900))//face_paras->poseEstimation->estimate_pose(face_paras->pointCloudMesh->img3D_pose))
            {
                //reset face rect
                face_paras->depthImage_chosedRegion = face_paras->poseEstimation->face_rect;

                int idx = face_paras->poseEstimation->best_cluster_index;
                double rx = -face_paras->poseEstimation->g_means[idx][3]*mult;
                double ry = face_paras->poseEstimation->g_means[idx][4]*mult;
                double rz = face_paras->poseEstimation->g_means[idx][5]*mult;

                face_paras->RT = Eigen::AngleAxisd(-rx, Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(-ry, Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(-rz, Eigen::Vector3d::UnitZ());
                face_paras->RT.translation()[0] = face_paras->poseEstimation->g_means[idx][0];
                face_paras->RT.translation()[1] = -face_paras->poseEstimation->g_means[idx][1];
                face_paras->RT.translation()[2] = -face_paras->poseEstimation->g_means[idx][2];

                //update RT_expression
                Eigen::Map<Eigen::Matrix3Xd > expression(face_paras->expression.data(), 3, face_paras->expression.size()/3);
                face_paras->RT_expression.noalias() = face_paras->RT.rotation() * expression;
                face_paras->RT_expression.colwise() += face_paras->RT.translation();

                //reset smooth RT
                for (int i = 0; i < 5; ++i)
                {
                    face_paras->temp_rotation[i] = Eigen::Quaterniond(face_paras->RT.rotation());
                    face_paras->temp_translation[i] = face_paras->RT.translation();
                }
            }
        }
    }
}
