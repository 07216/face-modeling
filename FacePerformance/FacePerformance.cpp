#include "FacePerformance.h"


FacePerformance::FacePerformance()
{
    omp_set_num_threads(16);
}

void FacePerformance::InitializePlugin()
{
    of.open("error_faceperformance.txt");
    depthMesh.kdtree = NULL;
    faceParas.init_paras = &initParas;
    faceParas.rigid_paras = &rigidParas;
    faceParas.nonrigid_paras = &nonrigidParas;
    faceParas.refine_paras = &refineParas;
#ifdef EYETRACKER
    faceParas.eye_paras = &eyeParas;
#endif
    mutex_pointCloud.lock();
    depthMesh.clear();
    faceParas.pointCloudMesh = &depthMesh;
    userViewMesh->pointCloud = &depthMesh;
    mutex_pointCloud.unlock();
    InitSystem();
    OpenSensor();
}

void FacePerformance::InitSystem()
{
    std::string mesh_index_filename = ConfigParameter::mesh_keypoint_index_filename;
    std::cout<<mesh_index_filename<<std::endl;
    //read mesh marker
    FacePerformanceFunction::LoadRegisterIndexFromFile(faceParas.meshMarkerPointIndex, mesh_index_filename);
#ifdef EYETRACKER
    eyeParas.x.resize(eyeParas.num_blendshape);
    eyeParas.eye_keypoint_index_3d.resize(12);
    memcpy(eyeParas.eye_keypoint_index_3d.data(), faceParas.meshMarkerPointIndex.data()+1, sizeof(int)*12);
#endif
    LoadTStar();
    LoadAllParameters();

    of<<"init finish"<<std::endl;
}

void FacePerformance::LoadFaceParameters()
{
    InitRegister::LoadEigenVector(faceParas.mean_neutral, ConfigParameter::neutral_meanface_filename);
    faceParas.mean_neutral *= 100.0;

    InitRegister::LoadEigenVector(faceParas.P_eigenvalues_all,
                                  ConfigParameter::neutral_eigenvalue_filename);
    InitRegister::LoadEigenMatrix(faceParas.P_eigenvectors_all,
                                  ConfigParameter::neutral_eigenvector_filename);
    InitRegister::LoadEigenVector_gl(faceParas.E_eigenvalues_all,
                                     ConfigParameter::neutral_gl_eigenvalue_filename);
    InitRegister::LoadEigenMatrix_gl(faceParas.E_eigenvectors_all,
                                     ConfigParameter::neutral_gl_eigenvector_filename);
    faceParas.H_rotation = 10;
    faceParas.H_translation = 0.1;
}

void FacePerformance::LoadInitRegisterParameters()
{
    of<<"before load initParameters"<<std::endl;
    FacePerformanceFunction::LoadRegisterIndexFromFile(initParas.init_registerIndex,
                                                       ConfigParameter::init_register_index_filenname);
    initParas.init_registerIndex_size = initParas.init_registerIndex.size();
    FacePerformanceFunction::GetFacesIndexOfVerticesIndex(initParas.init_registerIndex,
                                                          userViewMesh->mesh,
                                                          initParas.init_face_index);

    FacePerformanceFunction::LoadRegisterIndexFromFile(initParas.jaw_registeIndex,
                                                       ConfigParameter::jaw_register_index_filename);

    FacePerformanceFunction::LoadRegisterIndexFromFile(initParas.fixed_registerIndex,
                                                       ConfigParameter::fixed_register_index_filename);
    of<<"num_E: "<<initParas.num_E<<std::endl;
    of<<"num_P: "<<initParas.num_P<<std::endl;
    faceParas.P_eigenvalues = faceParas.P_eigenvalues_all.block(0, 0, initParas.num_P, 1);
    faceParas.P_eigenvectors = faceParas.P_eigenvectors_all.block(0, 0, initParas.num_P, faceParas.P_eigenvectors_all.cols());
    faceParas.E_eigenvalues = faceParas.E_eigenvalues_all.block(0, 0, initParas.num_E, 1);
    faceParas.E_eigenvectors = faceParas.E_eigenvectors_all.block(0, 0, initParas.num_E, faceParas.E_eigenvectors_all.cols());
    faceParas.P_eigenvectors_T = faceParas.P_eigenvectors.transpose();
    faceParas.E_eigenvectors_T = faceParas.E_eigenvectors.transpose();

    initParas.num_r = 3;
    initParas.num_t = 3;
    initParas.num_y = faceParas.P_eigenvalues.size();
    initParas.num_z0 = faceParas.E_eigenvalues.size();
    initParas.num_Rt = initParas.num_r + initParas.num_t;
    initParas.num_yz0 = initParas.num_y + initParas.num_z0;
    initParas.num_all = initParas.num_r + initParas.num_t + initParas.num_y + initParas.num_z0;

    initParas.start_r = 0;
    initParas.start_t = initParas.start_r + initParas.num_r;
    initParas.start_y = initParas.start_t + initParas.num_t;
    initParas.start_z0 = initParas.start_y + initParas.num_y;

    faceParas.expression.resize(faceParas.mean_neutral.size(), 1);
    faceParas.neutral_expression.resize(faceParas.mean_neutral.size(),1);
    faceParas.RT_expression.resize(faceParas.mean_neutral.size(), 1);

    initParas.init_mean_neutral.resize(initParas.init_registerIndex.size()*3, 1);
    initParas.V.resize(3, initParas.init_registerIndex.size());
    initParas.Qn.resize(3, initParas.init_registerIndex.size());
    initParas.Qp.resize(3, initParas.init_registerIndex.size());
    initParas.weight_correspond.resize(initParas.init_registerIndex.size());

    initParas.init_mean_neutral = faceParas.mean_neutral.head(initParas.init_registerIndex_size*3);
    initParas.init_E_eigenvectors_T = faceParas.E_eigenvectors.leftCols(initParas.init_registerIndex_size*3).transpose();//initParas.init_E_eigenvectors.transpose();
    initParas.init_P_eigenvectors_T = faceParas.P_eigenvectors.leftCols(initParas.init_registerIndex_size*3).transpose();//initParas.init_P_eigenvectors.transpose();

    initParas.init_P_eigenvectors_T.resize(3, initParas.init_registerIndex.size()*initParas.num_y);
    initParas.init_E_eigenvectors_T.resize(3, initParas.init_registerIndex.size()*initParas.num_z0);

    //for solving y z0
    initParas.A.resize(initParas.init_registerIndex_size + initParas.fixed_registerIndex.size()*3, initParas.num_yz0);
    initParas.b.resize(initParas.init_registerIndex_size + initParas.fixed_registerIndex.size()*3);
    initParas.ATA.resize(initParas.num_yz0, initParas.num_yz0);
    initParas.nt_R_t.resize(3, initParas.init_registerIndex_size);
    initParas.R_m_t_c.resize(3, initParas.init_registerIndex_size);

}
void FacePerformance::LoadRigidRegisterParameters()
{
    FacePerformanceFunction::LoadRegisterIndexFromFile(rigidParas.init_registerIndex,
                                                       ConfigParameter::rigid_register_index_filename);
    FacePerformanceFunction::LoadRegisterIndexFromFile(rigidParas.nose_regionIndex,
                                                       ConfigParameter::nose_register_index_filename);
    rigidParas.init_registerIndex_size = rigidParas.init_registerIndex.size();
    rigidParas.nose_regionIndex_size = rigidParas.nose_regionIndex.size();

    //get the face which cover the init_registerIndex
    FacePerformanceFunction::GetFacesIndexOfVerticesIndex(rigidParas.init_registerIndex, userViewMesh->mesh, rigidParas.init_face_index);
    FacePerformanceFunction::GetFacesIndexOfVerticesIndex(rigidParas.nose_regionIndex, userViewMesh->mesh, rigidParas.nose_face_index);

}
void FacePerformance::LoadNonrigidRegisterParameters()
{


    FacePerformanceFunction::LoadRegisterIndexFromFile(nonrigidParas.init_registerIndex,
                                                       ConfigParameter::nonrigid_register_index_filename);
    nonrigidParas.init_registerIndex_size = nonrigidParas.init_registerIndex.size();
    FacePerformanceFunction::GetFacesIndexOfVerticesIndex(initParas.init_registerIndex,
                                                          userViewMesh->mesh,
                                                          initParas.init_face_index);

    std::vector<int> update_index;
    FacePerformanceFunction::LoadRegisterIndexFromFile(update_index,
                                                       ConfigParameter::update_register_index_filename);

    for(int i=nonrigidParas.init_registerIndex_size; i<update_index.size(); i++)
    {
        nonrigidParas.fixed_registerIndex.push_back(i);
    }

    std::vector<int> mouth_brow_registerIndex;
    FacePerformanceFunction::LoadRegisterIndexFromFile(mouth_brow_registerIndex,
                                                       ConfigParameter::mouth_brow_upsample_index_filename);
    nonrigidParas.sp_upsampling
            = FacePerformanceFunction::GetUpSamplingSparseMatrix(userViewMesh->mesh,
                                                                 nonrigidParas.init_registerIndex,
                                                                 mouth_brow_registerIndex);
    nonrigidParas.init_registerIndex_upsampled_size = nonrigidParas.init_registerIndex_size;


    //optical index
    std::vector<int> mouth_brow_jaw_optical_registerIndex;
    FacePerformanceFunction::LoadRegisterIndexFromFile(mouth_brow_jaw_optical_registerIndex,
                                                       ConfigParameter::optical_index_filename);
    nonrigidParas.isOpticalIndex = new bool[nonrigidParas.init_registerIndex_upsampled_size];
    memset(nonrigidParas.isOpticalIndex, 0, sizeof(bool)*nonrigidParas.init_registerIndex_upsampled_size);
    for(int i=0; i<mouth_brow_jaw_optical_registerIndex.size(); i++)
    {
        int idx = mouth_brow_jaw_optical_registerIndex[i];
        nonrigidParas.isOpticalIndex[idx] = true;
    }
}

void FacePerformance::LoadRefinementParameters()
{
    std::cout<<"before initRefinement"<<std::endl;
    clock_t start = clock();
    FacePerformanceFunction::LoadRegisterIndexFromFile(refineParas.update_blendshape_index,
                                                       ConfigParameter::update_register_index_filename);
    refineParas.update_blendshape_index_size = refineParas.update_blendshape_index.size();

    refineParas.init_registerIndex = nonrigidParas.init_registerIndex;
    refineParas.init_registerIndex_size = refineParas.init_registerIndex.size();
    faceParas.isBackIndex.resize(faceParas.P_eigenvectors.cols()/3);
    faceParas.isBackIndex.setOnes();
    for(int i=0; i<refineParas.update_blendshape_index.size(); i++)
    {
        faceParas.isBackIndex(i) = 0;
    }
    refineParas.Ti_Star_P.resize(faceParas.num_blendshape-1);
    refineParas.Ti_Star_E.resize(faceParas.num_blendshape-1);
    refineParas.Ti_Star_m.resize(faceParas.num_blendshape-1);


    refineParas.part_Ti_Star_P_T.resize(faceParas.num_blendshape-1);
    refineParas.part_Ti_Star_E_T.resize(faceParas.num_blendshape-1);
    refineParas.part_Ti_Star_m.resize(faceParas.num_blendshape-1);

    int colNumOfy = faceParas.P_eigenvectors.rows();
    int colNumOfz = faceParas.E_eigenvectors.rows();
    int rowP = faceParas.P_eigenvectors.cols();
    int rowE = faceParas.E_eigenvectors.cols();

    Eigen::MatrixXd eachP(rowP,colNumOfy),eachE(rowE, colNumOfz),eachM;

    Eigen::MatrixXd P_eigenvectors_T = faceParas.P_eigenvectors.transpose();
    Eigen::MatrixXd E_eigenvectors_T = faceParas.E_eigenvectors.transpose();

    int j = 0;
    for(std::vector<SparseMatrixXd >::iterator iter = faceParas.GTHGF.begin(); iter != faceParas.GTHGF.end(); iter++,j++)
    {
        eachP = P_eigenvectors_T;
        refineParas.part_Ti_Star_P_T[j].resize(faceParas.part_T_Star_index[j].size()*3, colNumOfy);
        #pragma omp parallel for
        for(int i = 0; i < colNumOfy; i++)
        {
            for(int k=0; k<faceParas.part_T_Star_index[j].size(); k++)
            {
                int index = faceParas.part_T_Star_index[j][k];
                refineParas.part_Ti_Star_P_T[j].block(k*3, i, 3, 1) = eachP.block(index*3, i, 3, 1);
            }
            refineParas.part_Ti_Star_P_T[j].col(i) = faceParas.LDLT_solver[j].solve(*iter * refineParas.part_Ti_Star_P_T[j].col(i));
            for(int k=0; k<faceParas.part_T_Star_index[j].size(); k++)
            {
                int index = faceParas.part_T_Star_index[j][k];
                eachP.block(index*3, i, 3, 1) = refineParas.part_Ti_Star_P_T[j].block(k*3, i, 3, 1);
            }
        }

        eachE = E_eigenvectors_T;
        refineParas.part_Ti_Star_E_T[j].resize(faceParas.part_T_Star_index[j].size()*3, colNumOfz);
        #pragma omp parallel for
        for(int i = 0; i < colNumOfz; i++)
        {
            for(int k=0; k<faceParas.part_T_Star_index[j].size(); k++)
            {
                int index = faceParas.part_T_Star_index[j][k];
                refineParas.part_Ti_Star_E_T[j].block(k*3, i, 3, 1) = eachE.block(index*3, i, 3, 1);
            }
            refineParas.part_Ti_Star_E_T[j].col(i) = faceParas.LDLT_solver[j].solve(*iter * refineParas.part_Ti_Star_E_T[j].col(i));
            for(int k=0; k<faceParas.part_T_Star_index[j].size(); k++)
            {
                int index = faceParas.part_T_Star_index[j][k];
                eachE.block(index*3, i, 3, 1) = refineParas.part_Ti_Star_E_T[j].block(k*3, i, 3, 1);
            }
        }

        {
            eachM = faceParas.mean_neutral;
            refineParas.part_Ti_Star_m[j].resize(faceParas.part_T_Star_index[j].size()*3, 1);
            for(int k=0; k<faceParas.part_T_Star_index[j].size(); k++)
            {
                int index = faceParas.part_T_Star_index[j][k];
                refineParas.part_Ti_Star_m[j].block(k*3, 0, 3, 1) = eachM.block(index*3, 0, 3, 1);
            }
            refineParas.part_Ti_Star_m[j] = faceParas.LDLT_solver[j].solve(*iter * refineParas.part_Ti_Star_m[j]);
            for(int k=0; k<faceParas.part_T_Star_index[j].size(); k++)
            {
                int index = faceParas.part_T_Star_index[j][k];
                eachM.block(index*3, 0, 3, 1) = refineParas.part_Ti_Star_m[j].block(k*3, 0, 3, 1);
            }
        }
        refineParas.Ti_Star_P[j] = eachP;
        refineParas.Ti_Star_E[j] = eachE;
        refineParas.Ti_Star_m[j] = eachM;
    }

    //different expression with different E
    std::cout<<"before compute expression with different E"<<std::endl;
    refineParas.E_eigenvalues_gl.resize(faceParas.num_blendshape-1);
    refineParas.E_eigenvectors_T_gl.resize(faceParas.num_blendshape-1);
    refineParas.init_E_eigenvectors_T_gl.resize(faceParas.num_blendshape-1);
    refineParas.choose_E_eigenvectors_T_gl.resize(faceParas.num_blendshape-1);

    Eigen::MatrixXd tempMatrix;
    for(int i=1; i<faceParas.num_blendshape; i++)
    {
        std::string in_filename_vector = ConfigParameter::gl_EDir + "gl_eigenvector" + QString::number(i+13).toStdString() + ".txt";
        std::string in_filename_value = ConfigParameter::gl_EDir + "gl_eigenvalue" + QString::number(i+13).toStdString() + ".txt";
        InitRegister::LoadEigenMatrix_gl(tempMatrix, in_filename_vector);

        InitRegister::LoadEigenVector_gl(refineParas.E_eigenvalues_gl[i-1], in_filename_value);
        refineParas.E_eigenvectors_T_gl[i-1] = tempMatrix.transpose();
        refineParas.init_E_eigenvectors_T_gl[i-1] = refineParas.E_eigenvectors_T_gl[i-1].topRows(refineParas.init_registerIndex_size*3);
        refineParas.choose_E_eigenvectors_T_gl[i-1]
                .resize(refineParas.init_registerIndex_size*2, refineParas.E_eigenvalues_gl[i-1].size());
    }
    refineParas.start_index_E.resize(faceParas.num_blendshape-1);
    refineParas.num_index_E.resize(faceParas.num_blendshape-1);
    for(int i=0; i<refineParas.E_eigenvalues_gl.size(); i++)
    {
        if(i==0)
        {
            refineParas.start_index_E[i] = 0;
        }
        else
        {
            refineParas.start_index_E[i] = refineParas.start_index_E[i-1] + refineParas.num_index_E[i-1];
        }
        refineParas.num_index_E[i] = refineParas.E_eigenvalues_gl[i].size();
    }
    refineParas.num_all_E = refineParas.start_index_E[faceParas.num_blendshape-2]
            + refineParas.num_index_E[faceParas.num_blendshape-2];
    std::cout<<"after compute expression with different E: "<<refineParas.num_all_E<<std::endl;

    of<<"end initrefinement."<<clock() - start<<std::endl;
}
void FacePerformance::UpdateParas()
{
    mutex_paras.lock();
    QString config_filename(ConfigParameter::config_filename.c_str());
    QSettings config(config_filename,QSettings::IniFormat);//"Config.ini"
    initParas.maxInIters = config.value("init/maxInIters").toInt();
    initParas.outIters = config.value("init/outIters").toInt();
    initParas.registerRate = config.value("init/registerRate").toDouble();
    initParas.markerWeight_RT = config.value("init/markerWeight_RT").toDouble();
    initParas.markerWeight_yz0 = config.value("init/markerWeight_yz0").toDouble();
    initParas.beta1 = config.value("init/beta1").toDouble();
    initParas.beta2 = config.value("init/beta2").toDouble();
    initParas.beta3 = config.value("init/beta3").toDouble();
    initParas.num_P = config.value("init/numP").toInt();
    initParas.num_E = config.value("init/numE").toInt();
    initParas.radius_bilateral = config.value("init/radiusBilateral").toInt();
    initParas.max_facerecognition_error = config.value("init/maxFacerecognitionError").toDouble();

    rigidParas.maxInIters = config.value("rigid/maxInIters").toInt();
    rigidParas.icpIters = config.value("rigid/outIters").toInt();
    rigidParas.registerRate = config.value("rigid/registerRate").toDouble();
    rigidParas.markerWeight = config.value("rigid/markerWeight").toDouble();
    rigidParas.max_nose_error = config.value("rigid/max_nose_error").toDouble();

    nonrigidParas.outIters = config.value("nonrigid/outIters").toInt();
    nonrigidParas.registerRate = config.value("nonrigid/registerRate").toDouble();
    nonrigidParas.colorWeight = config.value("nonrigid/colorWeight").toDouble();
    nonrigidParas.markerWeight = config.value("nonrigid/markerWeight").toDouble();
    nonrigidParas.alpha = config.value("nonrigid/alpha").toDouble();
    nonrigidParas.beta = config.value("nonrigid/beta").toDouble();

    refineParas.MaxIter = config.value("refine/maxInIters").toInt();
    refineParas.gamma = config.value("refine/gamma").toDouble();
    refineParas.beta1 = config.value("refine/beta1").toDouble();
    refineParas.beta2 = config.value("refine/beta2").toDouble();
    refineParas.beta3 = config.value("refine/beta3").toDouble();
    refineParas.sigma = config.value("refine/sigma").toDouble();

    mutex_show.lock();
    showParas.pointCloudFormat = (enum ShowPointCloudFormat)config.value("show/pointCloudFormat").toInt();
    showParas.faceMeshFormat = (enum ShowFaceMeshFormat)config.value("show/faceMeshFormat").toInt();
    mutex_show.unlock();

    mutex_paras.unlock();
}
void FacePerformance::LoadAllParameters()
{
    UpdateParas();
    LoadFaceParameters();
    LoadInitRegisterParameters();
    LoadRigidRegisterParameters();
    LoadNonrigidRegisterParameters();
    LoadRefinementParameters();

    initRegister = new InitRegister(&faceParas);
    rigidRegister = new RigidRegister(&faceParas);
    nonrigidRegister = new NonRigidRegister(&faceParas);
    refineMent = new RefineMent(&faceParas);

}

void FacePerformance::ComputeBlendShapeByNeutral(MyTriMesh* neutral_mesh, int id_mesh, Eigen::CholmodSimplicialLDLT<SparseMatrixXd>& LDLT_solver, const SparseMatrixXd& GTHGF)//SparseMatrixXd& T_Star)
{
    int num_vertices = neutral_mesh->n_vertices();
    Eigen::VectorXd vec(num_vertices*3);
    MyTriMesh::VertexIter v_it, v_end = neutral_mesh->vertices_end();
    for(v_it = neutral_mesh->vertices_begin(); v_it != v_end; ++v_it)
    {
        MyTriMesh::Point point = neutral_mesh->point(v_it);
        int idx = v_it->idx();
        for(int i=0; i<3; i++)
        {
            vec(idx*3+i) = point[i];
        }
    }
    vec = GTHGF * vec;
    vec = LDLT_solver.solve(vec);
    for(v_it = neutral_mesh->vertices_begin(); v_it != v_end; ++v_it)
    {
        int idx = v_it->idx();
        neutral_mesh->point(v_it) = MyTriMesh::Point(vec(idx*3), vec(idx*3+1), vec(idx*3+2));
    }
    OpenMesh::IO::write_mesh(*neutral_mesh, "transfer.obj");
}
void FacePerformance::ComputeDeltaBByNeutral()
{
    faceParas.neutral_expression.resize(faceParas.neutral_expression.size(), 1);
    Eigen::MatrixXd& neutral_mesh = faceParas.neutral_expression;
    faceParas.delta_B.resize(neutral_mesh.size(), faceParas.num_blendshape-1);
    faceParas.delta_B.setZero();
#pragma omp parallel for
    for(int i=0; i<faceParas.delta_B.cols(); i++)
    {
        Eigen::MatrixXd part_neutral_mesh(faceParas.part_T_Star_index[i].size()*3, 1);
        for(int k=0; k<faceParas.part_T_Star_index[i].size(); k++)
        {
            int index = faceParas.part_T_Star_index[i][k];
            part_neutral_mesh.block(k*3, 0, 3, 1) = neutral_mesh.block(index*3, 0, 3, 1);
        }
        part_neutral_mesh = faceParas.LDLT_solver[i].solve(faceParas.GTHGF[i] * part_neutral_mesh);
        for(int k=0; k<faceParas.part_T_Star_index[i].size(); k++)
        {
            int index = faceParas.part_T_Star_index[i][k];
            faceParas.delta_B.block(index*3, i, 3, 1) = part_neutral_mesh.block(k*3, 0, 3, 1)-neutral_mesh.block(index*3, 0, 3, 1);
        }
    }
    //eye paras
#ifdef EYETRACKER
    eyeParas.delta_B.resize(neutral_mesh.size(), eyeParas.num_blendshape);
    eyeParas.delta_B.setZero();
    #pragma omp parallel for
    for(int i=0; i<eyeParas.delta_B.cols(); i++)
    {
        Eigen::MatrixXd part_neutral_mesh(eyeParas.part_T_Star_index[i].size()*3, 1);
        for(int k=0; k<eyeParas.part_T_Star_index[i].size(); k++)
        {
            int index = eyeParas.part_T_Star_index[i][k];
            part_neutral_mesh.block(k*3, 0, 3, 1) = neutral_mesh.block(index*3, 0, 3, 1);
        }
        part_neutral_mesh = eyeParas.LDLT_solver[i].solve(eyeParas.GTHGF[i] * part_neutral_mesh);
        for(int k=0; k<eyeParas.part_T_Star_index[i].size(); k++)
        {
            int index = eyeParas.part_T_Star_index[i][k];
            eyeParas.delta_B.block(index*3, i, 3, 1) = part_neutral_mesh.block(k*3, 0, 3, 1)-neutral_mesh.block(index*3, 0, 3, 1);
        }
    }
#endif
}

void FacePerformance::LoadTStar()
{
    faceParas.GTHGF.resize(faceParas.num_blendshape-1);
    faceParas.GTGF.resize(faceParas.num_blendshape-1);
    faceParas.part_T_Star_index.resize(faceParas.num_blendshape-1);
    QString tstar_dir(ConfigParameter::TStarDir.c_str());

    #pragma omp parallel for
    for (int i = 1; i < faceParas.num_blendshape; i++)
    {
        of<<"load by: "<<omp_get_thread_num()<<std::endl;
        FacePerformanceFunction::LoadSparseMatrix(faceParas.GTGF[i-1], QString("").append(tstar_dir).append("GTGF_").append(QString::number(i+13)).append(".txt").toStdString());
        faceParas.LDLT_solver[i-1].compute(faceParas.GTGF[i-1]);
        FacePerformanceFunction::LoadSparseMatrix(faceParas.GTHGF[i-1], QString("").append(tstar_dir).append("GTHGF_").append(QString::number(i+13)).append(".txt").toStdString());
        FacePerformanceFunction::LoadRegisterIndexFromFile(faceParas.part_T_Star_index[i-1], QString("").append(tstar_dir).append("index_").append(QString::number(i+13)).append(".txt").toStdString());
    }

    //eye paras
#ifdef EYETRACKER
    eyeParas.GTHGF.resize(eyeParas.num_blendshape);
    eyeParas.GTGF.resize(eyeParas.num_blendshape);
    eyeParas.part_T_Star_index.resize(eyeParas.num_blendshape);
    #pragma omp parallel for
    for (int i = 0; i < eyeParas.num_blendshape; i++)
    {
        FacePerformanceFunction::LoadSparseMatrix(eyeParas.GTGF[i], QString("").append(tstar_dir).append("GTGF_").append(QString::number(i)).append(".txt").toStdString());
        eyeParas.LDLT_solver[i].compute(eyeParas.GTGF[i]);
        FacePerformanceFunction::LoadSparseMatrix(eyeParas.GTHGF[i], QString("").append(tstar_dir).append("GTHGF_").append(QString::number(i)).append(".txt").toStdString());
        FacePerformanceFunction::LoadRegisterIndexFromFile(eyeParas.part_T_Star_index[i], QString("").append(tstar_dir).append("index_").append(QString::number(i)).append(".txt").toStdString());
    }
#endif
}

void FacePerformance::OpenSensor()
{
    maxDepth_openni = 1200;
    faceParas.colorImage.create(480, 640, CV_8UC3);
    faceParas.depthImage.create(480, 640, CV_16UC1);
#ifdef UNDER_LINUX
    faceParas.colorImage240X320.create(240, 320, CV_8UC3);
#endif
    faceParas.inDepthImage.resize(480,640);
    featureExtraction = new FeatureExtraction;//(640, 480);
    poseEstimation = new PoseEstimation;
    faceParas.poseEstimation = poseEstimation;
    poseEstimation->init();

#ifdef USE_OPENNI1
    //openni1
    {
        //Context context;
        xn::ScriptNode scriptNode;
        xn::EnumerationErrors errors;
        result_openni = XN_STATUS_OK;
        //context
        result_openni = context.Init();
        result_openni = depthGenerator.Create( context );
        result_openni = imageGenerator.Create( context );

        //set map mode
        mapMode.nXRes = 640;
        mapMode.nYRes = 480;
        mapMode.nFPS = 30;
        result_openni = depthGenerator.SetMapOutputMode( mapMode );
#ifndef UNDER_LINUX
        result_openni = imageGenerator.SetMapOutputMode( mapMode );
        //correct view port, align depth to image
        depthGenerator.GetAlternativeViewPointCap().SetViewPoint( imageGenerator );
#endif
        XnFieldOfView fov;
        depthGenerator.GetFieldOfView(fov);
        faceParas.fx = tan(fov.fHFOV / 2) * 2;
        faceParas.fy = tan(fov.fVFOV / 2) * 2;

        //start read data
        result_openni = context.StartGeneratingAll();
        result_openni = context.WaitNoneUpdateAll();
    }
#else
     //openni2
    {
        //context
        result_openni2 = openni::OpenNI::initialize();
        result_openni2 = oniDevice.open(openni::ANY_DEVICE);
        result_openni2 = oniDepthStream.create(oniDevice, openni::SENSOR_DEPTH);
        result_openni2 = oniColorStream.create(oniDevice, openni::SENSOR_COLOR);

        oniDepthMode.setResolution(640, 480);
        oniDepthMode.setFps(30);
        oniDepthMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
        oniDepthStream.setVideoMode(oniDepthMode);

        oniColorMode.setResolution(640,480);
        oniColorMode.setFps(30);
        oniColorMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
        oniColorStream.setVideoMode(oniColorMode);

        if(oniDevice.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
        {
            std::cout<<"yes"<<std::endl;
            oniDevice.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
        }
        oniDepthStream.start();
        oniColorStream.start();
        faceParas.fx = std::tan(oniDepthStream.getHorizontalFieldOfView()/2)*2;
        faceParas.fy = std::tan(oniDepthStream.getVerticalFieldOfView()/2)*2;
        std::cout<<"fx fy: "<<faceParas.fx<<' '<<faceParas.fy<<std::endl;

    }
#endif
    std::cout<<"after open sensor"<<std::endl;
}

bool FacePerformance::ReadFrame()
{

#ifdef FRAME_FROM_FILE1
    //read from file
    static int num_frame = 0;
    if(num_frame>145)exit(0);
    std::string filename_img = "face_stream/"+QString::number(num_frame).toStdString()+".jpg";
    FILE* fin_img = fopen(filename_img.c_str(), "rb");
    fread(faceParas.colorImage.data, sizeof(uchar), 640*480*3, fin_img);
    fclose(fin_img);
    std::string filename_dep = "face_stream/"+QString::number(num_frame).toStdString()+".dep";
    FILE* fin_dep = fopen(filename_dep.c_str(), "rb");
    fread(faceParas.depthImage.data, sizeof(uchar), 640*480*2, fin_dep);
    fclose(fin_dep);
    ++num_frame;
#else
    #ifdef USE_OPENNI1
        if(!(result_openni = context.WaitNoneUpdateAll()))//WaitOneUpdateAll(depthGenerator)))//.WaitAndUpdateAll()))//
        {
            //get meta data
            depthGenerator.GetMetaData(depthMD);
            imageGenerator.GetMetaData(imageMD);

            memcpy(faceParas.depthImage.data, depthMD.Data(), 640*480*2);
            cv::flip(faceParas.depthImage, faceParas.depthImage, 1);

            mutex_colorImage.lock();
#ifdef UNDER_LINUX
            memcpy(faceParas.colorImage240X320.data, imageMD.Data(), 240*320*3);
            cv::flip(faceParas.colorImage240X320, faceParas.colorImage240X320, 1);
            cv::resize(faceParas.colorImage240X320, faceParas.colorImage, cv::Size(640,480));
#else
            memcpy(faceParas.colorImage.data, imageMD.Data(), 640*480*3);
            cv::flip(faceParas.colorImage, faceParas.colorImage, 1);
#endif
        }
        else return false;
    #else

        if (oniColorStream.readFrame(&oniColorImg) == openni::STATUS_OK && oniDepthStream.readFrame( &oniDepthImg ) == openni::STATUS_OK)
        {
            //get depth data
            memcpy(faceParas.depthImage.data, oniDepthImg.getData(), 640*480*2);
            //get color data
            mutex_colorImage.lock();
            memcpy(faceParas.colorImage.data, oniColorImg.getData(), 640*480*3);
        }
        else return false;

    #endif
#endif

    //judge is tracking or not
    mutex_isTracking.lock();
    if(!isTracking)
    {
        mutex_colorImage.unlock();

        emit ShowColorImage(faceParas.colorImage.data, faceParas.colorImage.cols, faceParas.colorImage.rows);
        mutex_isTracking.unlock();
        return false;
    }
    else
    {
        mutex_isTracking.unlock();
    }

    mutex_isInitFrame.lock();
    if(isInitFrame)
    {
        faceParas.allColorImage.clear();
        faceParas.allDepthImage.clear();
    }
    mutex_isInitFrame.unlock();

    cv::cvtColor(faceParas.colorImage, faceParas.grayImage, CV_RGB2GRAY);
    mutex_isInitFrame.lock();

    if (isInitFrame)
    {
        std::cout<<"isInitFrame"<<std::endl;
    }
    else
    {
        std::cout<<"isNotInitFrame"<<std::endl;
    }
    //static int i=0;
    if (isInitFrame)
    {
        //detect face and pose with depth image
        if (!poseEstimation->detect_face_from_depthimage(faceParas.depthImage,faceParas.fx, faceParas.fy, maxDepth_openni))
        {
            std::cout<<"false depth image face detection"<<std::endl;
            mutex_isInitFrame.unlock();
            mutex_colorImage.unlock();
            emit ShowColorImage(faceParas.colorImage.data, faceParas.colorImage.cols, faceParas.colorImage.rows);

            return false;
        }
        faceParas.depthImage_chosedRegion = poseEstimation->face_rect;
#ifdef USE_KEYPOINT_INIT
        //detect keypoint with color image
        if(featureExtraction->GetFacePoint(faceParas.colorImage))
        {
            cv::Point center;
            center.x = featureExtraction->face_rect.x + featureExtraction->face_rect.width/2;
            center.y = featureExtraction->face_rect.y + featureExtraction->face_rect.height/2;
            if(!FacePerformanceFunction::IsInFaceRect(center, faceParas.depthImage_chosedRegion))
            {
                featureExtraction->feature.resize(0);
            }
        }
        else
#endif
        {
            featureExtraction->feature.resize(0);
        }
    }
#ifdef EYETRACKER
    else
    {
        std::cout<<"before getfacepoint"<<std::endl;
        if(!featureExtraction->GetFacePoint(faceParas.colorImage))
        {
            featureExtraction->feature.resize(0);
            eyeParas.eye_keypoint_2d.resize(0);
        }
        else
        {
            eyeParas.eye_keypoint_2d.resize(12);
            memcpy(eyeParas.eye_keypoint_2d.data(), featureExtraction->feature.data()+1, sizeof(cv::Point2d)*12);
            for(int i=0; i<eyeParas.eye_keypoint_2d.size(); i++)
            {
                //cv::circle(faceParas.colorImage, eyeParas.eye_keypoint_2d[i], 1,cv::Scalar(255,0,0),1);
            }
        }
    }
#else
    else if(1)//nonrigidParas.markerWeight != 0.0) //use marker according to nonrigidParas.markerWeight
    {
        if(!featureExtraction->GetFacePoint(faceParas.colorImage))
        {
            featureExtraction->feature.resize(0);
#ifdef FEATURE_WITH_EYE_CENTER
            featureExtraction->feature_eyes_center.resize(0);
#endif
        }
    }
    else
    {
        featureExtraction->feature.resize(0);
#ifdef FEATURE_WITH_EYE_CENTER
        featureExtraction->feature_eyes_center.resize(0);
#endif
    }
#endif

    //Gaussian blur
    cv::Mat gray_roi = faceParas.grayImage(faceParas.depthImage_chosedRegion);
    cv::Mat gray_blur;
    cv::GaussianBlur(gray_roi, gray_blur, cv::Size(5,5), 5, 5);
    gray_blur.copyTo(gray_roi);

    mutex_colorImage.unlock();
    mutex_isInitFrame.unlock();

    //generate point cloud
    mutex_pointCloud.lock();
    mutex_colorImage.lock();
    clock_t start = clock();
    FacePerformanceFunction::GeneratePointCloud(depthMesh, faceParas.fx, faceParas.fy, faceParas.depthImage_chosedRegion, faceParas.colorImage, faceParas.depthImage, maxDepth_openni, faceParas.inDepthImage, initParas.radius_bilateral);
    std::cout<<"generate pointcloud cost: "<<clock()-start<<std::endl;
    if (depthMesh.kdtree == NULL)
    {
        mutex_colorImage.unlock();
        mutex_pointCloud.unlock();
        std::cout<<"kdtree null"<<std::endl;
        emit ShowColorImage(faceParas.colorImage.data, faceParas.colorImage.cols, faceParas.colorImage.rows);
        return false;
    }
    mutex_colorImage.unlock();
    mutex_pointCloud.unlock();

    //map 2d keypoint to pointcloud
    faceParas.marker_registerIndex.clear();
    faceParas.marker_nearestIndex.clear();
    faceParas.marker_Coordinates.clear();
    faceParas.marker_Coordinates_all.resize(2, featureExtraction->feature.size());
    std::cout<<"feature size: "<<featureExtraction->feature.size()<<std::endl;
    mutex_isInitFrame.lock();
    mutex_colorImage.lock();
    for(int i=0; i<featureExtraction->feature.size(); i++)
    {
        int x = (int)(featureExtraction->feature[i].x + 0.5);
        int y = (int)(featureExtraction->feature[i].y + 0.5);
        x = x>=0?x:0;
        x = x<faceParas.colorImage.cols? x : faceParas.colorImage.cols-1;
        y = y>=0?y:0;
        y = y<faceParas.colorImage.rows? y : faceParas.colorImage.rows-1;
        if(i>=12 && (!isInitFrame || faceParas.inDepthImage(y,x) != -1))
        {
            faceParas.marker_nearestIndex.push_back(faceParas.inDepthImage(y, x));
            faceParas.marker_Coordinates.push_back(x);
            faceParas.marker_Coordinates.push_back(y);
            faceParas.marker_registerIndex.push_back(faceParas.meshMarkerPointIndex[i]);
        }
        faceParas.marker_Coordinates_all(0,i) = featureExtraction->feature[i].x;
        faceParas.marker_Coordinates_all(1,i) = featureExtraction->feature[i].y;
    }

    mutex_colorImage.unlock();
    mutex_pointCloud.lock();
    std::cout<<"point cloud size: "<<depthMesh.vertices.size()<<std::endl;
    if (depthMesh.vertices.size() <= 100)
    {
        mutex_pointCloud.unlock();
        mutex_isInitFrame.unlock();
        emit ShowColorImage(faceParas.colorImage.data, faceParas.colorImage.cols, faceParas.colorImage.rows);
        return false;
    }
    else
    {
        mutex_pointCloud.unlock();
        mutex_isInitFrame.unlock();
        return true;
    }
}

void FacePerformance::StartTrackThread()
{
    ShowThread * thread = new ShowThread(Name() + "tracking");
    connect(thread, SIGNAL(track()), this, SLOT(TrackFace()), Qt::DirectConnection);
    thread->start();                                                                                           // start thread
}

void FacePerformance::TrackFace()
{
    while (1)
    {
        clock_t start, finish;
        start  = clock();
        while (!ReadFrame())
        {
            std::cout<<"read Frame wrong!: "<<clock()-start<<std::endl;
        }
        finish = clock();
        mutex_isInitFrame.lock();
        if (isInitFrame)
        {
            isInitFrame = false;
            mutex_isInitFrame.unlock();

            mutex_pointCloud.lock();
            depthMesh.write("init_pointCloud.obj");
            mutex_pointCloud.unlock();

            mutex_paras.lock();
            emit ShowColorImage(faceParas.colorImage.data, faceParas.colorImage.cols, faceParas.colorImage.rows);

            of<<"isInitFrame"<<std::endl;
            clock_t start, finish;
            start  = clock();

            //creat mesh object
            expression_mesh = &(userViewMesh->mesh);
            faceParas.expression_mesh = expression_mesh;
            //end of creating mesh object

            initRegister->FitMesh();
            cv::imwrite("gray.jpg", faceParas.grayImage);
            //////////////////////////
//            cv::Mat temp;
//            cv::cvtColor(faceParas.colorImage, temp, CV_RGB2BGR);
//            cv::imwrite(std::to_string(nonrigidParas.num_frame)+".jpg", temp);

            finish = clock();
            double duration = (double)(finish - start) / CLOCKS_PER_SEC;
            of<<"init register time: "<<duration<<std::endl;

            of<<"init registerMesh start"<<std::endl;
            start = clock();
            ComputeDeltaBByNeutral();
            of<<"compute DeltaBByNeutral cost: "<< clock() - start<<std::endl;
            of<<"init RegisterMesh finish"<<std::endl;

            //re initialize nonrigid
            nonrigidParas.pre_frame_x.resize(faceParas.num_blendshape-1);
            nonrigidParas.ppre_frame_x.resize(faceParas.num_blendshape-1);
            nonrigidParas.pre_frame_x.setZero();
            nonrigidParas.ppre_frame_x.setZero();
            nonrigidParas.num_frame = 0;
            refineMent->Initialization();

            mutex_pointCloud.lock();

            mutex_show.lock();
            mesh_color.resize(3, faceParas.RT_expression.size()/3);
            mutex_show.unlock();

            TriMesh_Eigen finalMesh;
            initRegister->Blendshape2Trimesh(finalMesh);
            finalMesh.write("init_finalMesh.obj");

            faceParas.RT_expression.resize(3, faceParas.RT_expression.size()/3);
            int vIndex = 0;
            PolyMesh::VertexIter v_it, v_end = expression_mesh->vertices_end();
            for (v_it = expression_mesh->vertices_begin(); v_it != v_end; ++v_it, vIndex++)
            {
                mutex_colorImage.lock();
                Eigen::VectorXd color_rgb = FacePerformanceFunction::Compute_f_color_RGB(faceParas.RT_expression.col(vIndex), faceParas.colorImage, faceParas.fx, faceParas.fy);
                mutex_colorImage.unlock();
                expression_mesh->point(v_it) = PolyMesh::Point(faceParas.RT_expression.col(vIndex).data());
                mutex_show.lock();
                mesh_color.col(vIndex) = color_rgb;
                mutex_show.unlock();
            }
            expression_mesh->update_face_normals();
            expression_mesh->update_vertex_normals();
            userViewMesh->translation_show = -faceParas.RT_expression.rowwise().mean();
            mutex_x.lock();
            avatarViewMesh->translation_show = userViewMesh->translation_show;
            mutex_x.unlock();

            mutex_pointCloud.unlock();
            of<<"after construct mesh"<<std::endl;
            mutex_paras.unlock();
        }
        else //realtime tracking
        {
            mutex_isInitFrame.unlock();
            mutex_paras.lock();

            //rigid register
            clock_t start, finish;
            start  = clock();
            clock_t start_rigid, cost_rigid = 0;
            clock_t start_nonrigid, cost_nonrigid = 0;
            bool isContinue = false;
            for (int i = 0; i < rigidParas.maxInIters; ++i)
            {
                start_rigid = clock();
                isContinue = false;

                rigidRegister->FitMeshICP();
                cost_rigid += clock() - start_rigid;
                if (!rigidParas.tracking_succeed)
                {
                    isContinue = true;
                    continue;
                }

                //nonrigid register
                start_nonrigid = clock();
                mutex_pointCloud.lock();
                mutex_pointCloud.unlock();

                //start  = clock();
                nonrigidRegister->FitMeshLasso();

                cost_nonrigid += clock() - start_nonrigid;
            }
            if(!isContinue)nonrigidRegister->AfterFit();

            if (isContinue)
            {
                mutex_paras.unlock();
                emit ShowColorImage(faceParas.colorImage.data, faceParas.colorImage.cols, faceParas.colorImage.rows);
                emit UpdateView();
                continue;
            }

            finish = clock();
            double duration = (double)(finish - start)/CLOCKS_PER_SEC;

            mutex_colorImage.lock();
            Eigen::Matrix2Xi key_point_3d_to_2d(2, faceParas.meshMarkerPointIndex.size());
            for(int i=0; i<faceParas.meshMarkerPointIndex.size(); i++)
            {
                int index = i;
                cv::Point p1 = cv::Point(nonrigidParas.mapPoint2Image(0,index), nonrigidParas.mapPoint2Image(1,index));
                //cv::circle(faceParas.colorImage,p1,1,cv::Scalar(255,0,0),1);
                key_point_3d_to_2d(0,i) = p1.x;
                key_point_3d_to_2d(1,i) = p1.y;
            }
            FacePerformanceFunction::ComputeNewDepthImageRegion(faceParas.depthImage_chosedRegion, faceParas.colorImage, key_point_3d_to_2d);
            mutex_colorImage.unlock();
            //if(nonrigidParas.num_frame == 1){of<<"imwrite: "<<nonrigidParas.num_frame<<std::endl; cv::imwrite("1.jpg", faceParas.colorImage);}

            emit ShowColorImage(faceParas.colorImage.data, faceParas.colorImage.cols, faceParas.colorImage.rows);

            of<<"Frame "<<nonrigidParas.num_frame<<std::endl;
            of<<"...rigid time: "<< cost_rigid << std::endl;
            of<<"...nonrigid time: "<< cost_nonrigid << std::endl;
            of<<"rigid and nonrigid register time: "<<duration<<std::endl;
            start  = clock();

            mutex_pointCloud.lock();
            nonrigidRegister->Compute_A_b_to_refinement();

            mutex_pointCloud.unlock();
            //////****************************************************************///////////
            //Refinement
            start = clock();

            if (refineParas.MaxIter != 0.0)// && nonrigidParas.num_frame < 50)
                refineMent->Refine(faceParas.y, faceParas.z0);

            finish = clock();
            of<<"...refinement time: "<< (double)(finish - start)/CLOCKS_PER_SEC << std::endl;


            mutex_paras.unlock();
            mutex_x.lock();
            avatarViewMesh->x.resize(46);
            avatarViewMesh->x.setZero();
#ifdef EYETRACKER
            avatarViewMesh->x.head(eyeParas.num_blendshape) = eyeParas.x;
#endif
            avatarViewMesh->x.tail(faceParas.num_blendshape-1) = faceParas.x;
            avatarViewMesh->R = faceParas.RT_smooth.rotation();//faceParas.s *
            avatarViewMesh->T = faceParas.RT_smooth.translation();
            mutex_x.unlock();
            emit UpdateAvatar();
        }
        emit UpdateView();
    }
}

