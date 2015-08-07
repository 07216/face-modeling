#include "RefineMent.h"

RefineMent::RefineMent()
{
    omp_set_num_threads(16);
    weight_fix = 10.1;

    of.open("error_Refinement.txt");
    of<<"start RefineMent!"<<std::endl;
}

RefineMent::RefineMent(FacePerformanceParameter *face_paras)
{
    omp_set_num_threads(16);
    of.open("error_Refinement.txt");
    of<<"start RefineMent!"<<std::endl;

    this->face_paras = face_paras;
    this->paras = face_paras->refine_paras;
    Initialization();
}

RefineMent::~RefineMent()
{
    of.close();
}

void RefineMent::Initialization()
{
    //pre compute
    middle_s = 0; //the s of algorithm 1
    colNumOfy = face_paras->P_eigenvalues.size();
    colNumOfz = face_paras->E_eigenvalues.size();
    colNumOfy_z = colNumOfy + colNumOfz;
    sizeOfx = face_paras->num_blendshape - 1;//number of expressions
    colNumOfAll = colNumOfy_z + paras->num_all_E; //size of u
    rowAfterChoose = paras->init_registerIndex_size * 2;//size of 3d + 2d

    //convergence of blendshape
    paras->sigma_blendshape.resize(sizeOfx);
    paras->sigma_blendshape.setZero();
    paras->sigma_b0 = 0.0;

    //guass_seidel LHS and RHS
    middleMatrix.resize(colNumOfAll, colNumOfAll);
    middleMatrix.setZero();
    middleVector.resize(colNumOfAll);
    middleVector.setZero();

    numFrames = 0;
    convergedExpressions = 0;

    //fix part of E
#ifdef FIX_MOUTH_NOSE_EYE
    fixedETE.resize(face_paras->num_blendshape);
    int startIndex = paras->init_registerIndex_size * 3;
    int fixedSize = face_paras->nonrigid_paras->fixed_registerIndex.size()*3;
    Eigen::MatrixXd temp_fixedE = face_paras->E_eigenvectors_T.middleRows(startIndex, fixedSize);
    fixedETE[0] = temp_fixedE.transpose() * temp_fixedE;
    for(int i=1; i<face_paras->num_blendshape; i++)
    {
        temp_fixedE = paras->E_eigenvectors_T_gl[i-1].middleRows(startIndex, fixedSize);
        fixedETE[i] = temp_fixedE.transpose() * temp_fixedE;
    }
#endif

    face_paras->u.resize(colNumOfAll);
    face_paras->u.setZero();
    A.resize(rowAfterChoose, colNumOfy_z);
    c.resize(rowAfterChoose);

    rowBeforeChoose = paras->init_registerIndex_size * 3;
    paras->update_Tib0.resize(face_paras->delta_B.rows(), face_paras->delta_B.cols());
    paras->update_Tib0.setZero();
    paras->update_Ezi.resize(paras->update_blendshape_index_size*3, sizeOfx);
    paras->update_Ezi.setZero();

    //regular term diagonal
    paras->diag.resize(colNumOfAll);

    #pragma omp parallel for
    for (int i = 0; i < colNumOfy; ++i)
    {
        paras->diag(i) = paras->beta1/(face_paras->P_eigenvalues.coeff(i)*face_paras->P_eigenvalues.coeff(i));
    }

    #pragma omp  parallel for
    for (int i = 0; i < colNumOfz; ++i)
    {
        paras->diag(colNumOfy+i) = paras->beta2*face_paras->E_eigenvalues.coeff(i)*face_paras->E_eigenvalues.coeff(i)
                + paras->beta3;
    }

    #pragma omp parallel for
    for (int i = 0; i < paras->start_index_E.size(); ++i)
    {
        for (int j = 0; j < paras->num_index_E[i]; ++j)
        {
           paras->diag(colNumOfy_z + paras->start_index_E[i] + j) = paras->beta2*paras->E_eigenvalues_gl[i][j]*paras->E_eigenvalues_gl[i][j]
                   + paras->beta3;
        }
    }
}

void RefineMent::addParas( FacePerformanceParameter* face_paras)
{
    this->face_paras = face_paras;
    this->paras = face_paras->refine_paras;
    Initialization();
    //paras->isInitRefinement = true;
}

void RefineMent::Refine(Eigen::VectorXd &y, Eigen::VectorXd &z0)
{
	clock_t start = clock();

    //judge whether enough expressions are refined
    if (convergedExpressions > paras->sigma_blendshape.size()/3 || numFrames > 200)
    {
        return;
    }

    //compute A and c
    PreProcessing();

    of <<"prepare time: "<<clock() - start << std::endl;

    double old_w_middle_s_div = 1.0/(paras->gamma * middle_s + 1);
    double oldS = middle_s;
    middle_s = paras->gamma * middle_s + 1;
    double w_middleMatrix = (paras->gamma * oldS / middle_s);
    double w_middle_s_div = 1.0/middle_s;

    //update M and y of algorithm 1
    middleMatrix *= w_middleMatrix;
    middleVector *= w_middleMatrix;

    //compute the size of choosed rows, whether 3d or 2d+3d
    int choose_rows = paras->init_registerIndex_size;
    if (face_paras->nonrigid_paras->colorWeight != 0.0)
    {
        choose_rows = paras->init_registerIndex_size * 2;
    }

    Eigen::MatrixXd Apart = A.topRows(choose_rows);
    Eigen::VectorXd cpart = c.topRows(choose_rows);
    if (paras->sigma_b0 < paras->sigma)
    {
        //Add ATA and ATc;
        middleMatrix.block(0, 0, colNumOfy_z, colNumOfy_z).selfadjointView<Eigen::Upper>().rankUpdate(Apart.transpose(), old_w_middle_s_div);
        middleVector.block(0, 0, colNumOfy_z, 1) += (Apart.transpose()*cpart)*old_w_middle_s_div;
    }
    of <<"ATA, Atc time: "<<clock() - start <<std::endl;

    #pragma omp parallel for
    for (int i = 0; i < nonzero_and_noconvergent_x_index.size(); ++i)
    {
        int index = nonzero_and_noconvergent_x_index[i];
        int row_idx = colNumOfy_z + paras->start_index_E[index];
        int E_num = paras->num_index_E[index];

        //diag
        double scale = face_paras->x.coeff(index)*face_paras->x.coeff(index)*w_middle_s_div;
        Eigen::MatrixXd egl = paras->choose_E_eigenvectors_T_gl[index].topRows(choose_rows);
        middleMatrix.block(row_idx, row_idx, E_num, E_num).selfadjointView<Eigen::Upper>().rankUpdate(egl.transpose(), scale);

        //[TP TE]*Ei and its tarnspose
        middleMatrix.block(0, row_idx, colNumOfy_z, E_num) += (face_paras->x.coeff(index)*w_middle_s_div)*(Apart.transpose()*egl);
        middleVector.block(row_idx, 0, E_num, 1) += (face_paras->x.coeff(index)*w_middle_s_div)*(egl.transpose()*cpart);
    }
    of <<"mul time 1: "<<clock() - start <<std::endl;

    #pragma omp parallel for
    for (int i = 0; i < nonzero_and_noconvergent_x_index_upper_pair.size(); ++i)
    {
        int index1 = nonzero_and_noconvergent_x_index_upper_pair[i].first;
        int index2 = nonzero_and_noconvergent_x_index_upper_pair[i].second;
        int row_idx1 = colNumOfy_z + paras->start_index_E[index1];
        int row_idx2 = colNumOfy_z + paras->start_index_E[index2];
        int E_num1 = paras->num_index_E[index1];
        int E_num2 = paras->num_index_E[index2];
        middleMatrix.block(row_idx1, row_idx2, E_num1, E_num2)
                += paras->choose_E_eigenvectors_T_gl[index1].transpose() * paras->choose_E_eigenvectors_T_gl[index2]
                * (face_paras->x.coeff(index1) * face_paras->x.coeff(index2) * w_middle_s_div);
    }
    of <<"mul time 2: "<<clock() - start << std::endl;

    face_paras->u.block(0, 0, colNumOfy, 1) = y;
    face_paras->u.block(colNumOfy, 0, colNumOfz, 1) = z0;

    //fixed part RHS
#ifdef FIX_MOUTH_NOSE_EYE
    middleMatrix.block(colNumOfy, colNumOfy, colNumOfz, colNumOfz) += fixedETE[0] * w_middle_s_div * weight_fix;
    #pragma omp parallel for
    for(int i=1; i<face_paras->num_blendshape; i++)
    {
        int E_idx = colNumOfy_z + paras->start_index_E[i-1];
        int E_num = paras->num_index_E[i-1];
        middleMatrix.block(E_idx, E_idx, E_num, E_num) += fixedETE[i] * w_middle_s_div * weight_fix;
    }
#endif

    face_paras->Last_u = face_paras->u;
    of <<"before gausseidel time: "<<clock() - start <<std::endl;
    GaussSeidel(middleMatrix, paras->diag, middleVector, face_paras->u);
    of <<"after gausseidel time: "<<clock() - start <<std::endl;

    y = face_paras->u.block(0,0,colNumOfy,1);
    z0 = face_paras->u.block(colNumOfy,0,colNumOfz,1);

    UpdateB0AndDeltaB(y,z0);

    paras->sigma_b0 += std::max(xbar, 0.0);
    paras->sigma_blendshape += face_paras->x;

    //check whether convergence of blendshape
    if (paras->sigma_b0 > paras->sigma)
    {
        middleMatrix.leftCols(colNumOfy_z).setZero();
        middleMatrix.topRows(colNumOfy_z).setZero();
        middleVector.block(0, 0, colNumOfy_z, 1).setZero();
        paras->diag.block(0, 0, colNumOfy_z, 1).setZero();
    }

    convergedExpressions = 0;
    for (int i = 0; i < paras->sigma_blendshape.size(); ++i)
    {
        if (paras->sigma_blendshape(i) > paras->sigma)
        {
            convergedExpressions++;
            middleMatrix.middleRows(colNumOfy_z + paras->start_index_E[i], paras->num_index_E[i]).setZero();
            middleMatrix.middleCols(colNumOfy_z + paras->start_index_E[i], paras->num_index_E[i]).setZero();
            middleVector.block(colNumOfy_z + paras->start_index_E[i], 0, paras->num_index_E[i], 1).setZero();
            paras->diag.block(colNumOfy_z + paras->start_index_E[i], 0, paras->num_index_E[i], 1).setZero();
        }
    }

    of <<"total refinement time: "<<clock() - start << std::endl;
    of <<"converged expressions: "<<convergedExpressions <<"/"<<paras->sigma_blendshape.size()<< std::endl;
    of <<"number of frames: "<<++numFrames<<std::endl;
    of<<"-----------------------"<<std::endl<<std::endl;
}

void RefineMent::PreProcessing()
{
    clock_t start = clock();

    // save convergent z_i values
    int ucompletesize = 0;
    if (paras->sigma_b0 > paras->sigma)
        ucompletesize += colNumOfy_z;

    for (int i = 0; i < face_paras->x.size(); ++i)
    {
        if (std::fabs(face_paras->x.coeff(i)) > 1.0e-8 && paras->sigma_blendshape.coeff(i) > paras->sigma)
        {
            ucompletesize += paras->num_index_E[i];
        }
    }
    ucomplete.resize(ucompletesize);
    Acomplete.resize(rowAfterChoose, ucompletesize);
    ucomplete.setZero();
    Acomplete.setZero();

    int completeIndex = 0;
    if (paras->sigma_b0 > paras->sigma)
    {
        ucomplete.block(0, 0, colNumOfy, 1) = face_paras->u.block(0, 0, colNumOfy, 1);
        ucomplete.block(colNumOfy, 0, colNumOfz, 1) = face_paras->u.block(colNumOfy, 0, colNumOfz, 1);
        completeIndex += colNumOfy + colNumOfz;
    }

    xbar = 1 - face_paras->x.sum();
    nonzero_x_index.clear();
    nonzero_and_noconvergent_x_index.clear();
    for (int i = 0; i < face_paras->x.size(); ++i)
    {
        if (std::fabs(face_paras->x.coeff(i)) > 1.0e-8)
        {
            nonzero_x_index.push_back(i);
            if (paras->sigma_blendshape.coeff(i) < paras->sigma)
            {
                nonzero_and_noconvergent_x_index.push_back(i);
            }
            else
            {
                ucomplete.block(completeIndex, 0, paras->num_index_E[i], 1)
                        = face_paras->u.block(colNumOfy_z + paras->start_index_E[i], 0, paras->num_index_E[i], 1);
                completeIndex += paras->num_index_E[i];
            }
        }
    }
    //end of save

    nonzero_and_noconvergent_x_index_upper_pair.clear();
    for (int i = 0; i < nonzero_and_noconvergent_x_index.size(); ++i)
    {
        for(int j = i + 1; j < nonzero_and_noconvergent_x_index.size(); ++j)
        {
            nonzero_and_noconvergent_x_index_upper_pair.push_back(std::pair<int, int>(nonzero_and_noconvergent_x_index[i], nonzero_and_noconvergent_x_index[j]));
        }
    }
    of <<"before get_A: "<<clock() - start << std::endl;

    Compute_A();
    of <<"after Compute_A: "<<clock() - start << std::endl;

    Compute_c();
    of <<"after Compute_c: "<<clock() - start << std::endl;
}

// A is the Abar in the paper, and the first and second block is the same with Abar, while the third block is without x_i
void RefineMent::Compute_A()
{
    Eigen::MatrixXd ABeforeChoose(rowBeforeChoose, colNumOfy_z);
    ABeforeChoose.setZero();

    #pragma omp parallel sections
    {
        #pragma omp section
        {
            ABeforeChoose.leftCols(colNumOfy).noalias() = xbar*face_paras->P_eigenvectors_T.topRows(rowBeforeChoose);
            for (int i = 0; i < nonzero_x_index.size(); ++i)
            {
                int index = nonzero_x_index[i];
                ABeforeChoose.leftCols(colNumOfy).noalias() += face_paras->x.coeff(index)*paras->Ti_Star_P[index].topRows(rowBeforeChoose);
            }
        }
        #pragma omp section
        {
            ABeforeChoose.middleCols(colNumOfy, colNumOfz).noalias() = xbar*face_paras->E_eigenvectors_T.topRows(rowBeforeChoose);
            for (int i = 0; i < nonzero_x_index.size(); ++i)
            {
                int index = nonzero_x_index[i];
                ABeforeChoose.middleCols(colNumOfy, colNumOfz).noalias() += face_paras->x.coeff(index)*paras->Ti_Star_E[index].topRows(rowBeforeChoose);
            }
        }
    }
    RegistrationMatrixChoose(ABeforeChoose, &A);

    int completeIndex = 0;
    if (paras->sigma_b0 > paras->sigma)
    {
        Acomplete.leftCols(colNumOfy_z) = A;
        A.setZero();
        completeIndex += colNumOfy_z;
    }

    for (int i = 0; i < nonzero_x_index.size(); ++i)
    {
        int index = nonzero_x_index[i];
        RegistrationMatrixChoose(paras->init_E_eigenvectors_T_gl[index], &(paras->choose_E_eigenvectors_T_gl[index]));

        if (paras->sigma_blendshape.coeff(index) > paras->sigma)
        {
            Acomplete.middleCols(completeIndex, paras->num_index_E[index]) = face_paras->x.coeff(index)*paras->choose_E_eigenvectors_T_gl[index];
            completeIndex += paras->num_index_E[index];
        }
    }
}

// c is the cbar in the paper
void RefineMent::Compute_c()
{
    Eigen::MatrixXd sumM;
    cBeforeChoose = xbar * face_paras->mean_neutral.topRows(rowBeforeChoose);

    for(int i = 0; i < nonzero_x_index.size(); ++i)
    {
        int j = nonzero_x_index[i];
        cBeforeChoose.noalias() += face_paras->x.coeff(j) * paras->Ti_Star_m[i].topRows(rowBeforeChoose);
    }

    sumM.resize(rowAfterChoose, 1);
    RegistrationMatrixChoose(cBeforeChoose, &sumM);

    c.topRows(face_paras->b_3d_to_refinement.rows()) = face_paras->b_3d_to_refinement - sumM.topRows(face_paras->b_3d_to_refinement.rows());
    if(face_paras->nonrigid_paras->colorWeight != 0.0)
        c.bottomRows(face_paras->b_2d_to_refinement.rows()) = face_paras->b_2d_to_refinement - sumM.bottomRows(face_paras->b_2d_to_refinement.rows());

    //the part from fix b_i
    c -= Acomplete*ucomplete;
}

// Select point based on registerIndex
void RefineMent::RegistrationMatrixChoose(Eigen::MatrixXd &originalMatrix, Eigen::MatrixXd *registrationMatrix)
{
    int rowNum_org = originalMatrix.rows();
    int colNum_org = originalMatrix.cols();
    int rowNum_reg = registrationMatrix->rows();
    int colNum_reg = registrationMatrix->cols();
    registrationMatrix->resize(1, registrationMatrix->size());
    registrationMatrix->setZero();
    originalMatrix.resize(3, originalMatrix.size()/3);

    #pragma omp parallel for
    for (int i = 0; i < colNum_org; ++i)
    {
        registrationMatrix->block(0, i*rowNum_reg, 1, paras->init_registerIndex_size)
                = (face_paras->A_3d_to_refinement.array()*originalMatrix.block(0, (paras->init_registerIndex_size)*i, 3, paras->init_registerIndex_size).array()).colwise().sum();
        if(face_paras->nonrigid_paras->colorWeight != 0.0)
        registrationMatrix->block(0, i*rowNum_reg+paras->init_registerIndex_size, 1, paras->init_registerIndex_size)
                = (face_paras->A_2d_to_refinement.array()*originalMatrix.block(0, (paras->init_registerIndex_size)*i, 3, paras->init_registerIndex_size).array()).colwise().sum();
    }
    originalMatrix.resize(rowNum_org, colNum_org);
    registrationMatrix->resize(rowNum_reg, colNum_reg);
}

void RefineMent::GaussSeidel(Eigen::MatrixXd &A, const Eigen::VectorXd& diag, const Eigen::VectorXd &b,  Eigen::VectorXd &u)
{
    A.triangularView<Eigen::StrictlyLower>() = A.transpose();

    for (int k = 0; k < paras->MaxIter; ++k)
    {
        for (int i = 0; i < u.size(); ++i)
        {
            if (A(i, i) != 0)
            {
                u(i) = (b.coeff(i) + A(i, i) * u.coeff(i) - A.col(i).dot(u))/(A(i, i) + diag.coeff(i));
            }
        }
    }
}

void RefineMent::UpdateB0AndDeltaB(const Eigen::VectorXd &y, const Eigen::VectorXd &z0)
{
    clock_t start1 = clock();
    int rowOfupdate = paras->update_blendshape_index_size*3;

    std::vector<int> updateTstarb; //update the T_star * neutral part
    std::vector<int> updateEzi; //upadate the Ei part
    std::vector<bool> updateFlag(face_paras->delta_B.cols(), false);
    bool updateNeutral = false;
    if (paras->sigma_b0 < paras->sigma && face_paras->u.topRows(colNumOfy_z).cwiseAbs().sum() != 0)
    {
        updateNeutral = true;
    }
    if (updateNeutral)
    {
        face_paras->neutral_expression.resize(face_paras->neutral_expression.size(), 1);
        face_paras->neutral_expression = face_paras->mean_neutral;
        face_paras->neutral_expression.noalias() += face_paras->P_eigenvectors.transpose()*y;
        face_paras->neutral_expression.noalias() += face_paras->E_eigenvectors.transpose()*z0;

        //need to update T_i^{*}b0 whenever b0 changed
        for (int i = 0; i < face_paras->delta_B.cols(); ++i)
        {
            updateTstarb.push_back(i);
            updateFlag[i] = true;
        }
    }

    clock_t cost1 = clock()-start1;
    for (int i = 0; i < paras->sigma_blendshape.size(); ++i)
    {
        //need to update E*z_i whenever z_i is not converged
        if (paras->sigma_blendshape(i) < paras->sigma)
        {
            updateEzi.push_back(i);
            updateFlag[i] = true;
        }
    }

    clock_t start2 = clock();
    #pragma omp parallel for schedule(guided)
    for (int i = 0; i < updateTstarb.size(); ++i)
    {
        int index = updateTstarb[i];
        paras->update_Tib0.col(index).setZero();
        Eigen::MatrixXd part_b0(face_paras->part_T_Star_index[index].size()*3, 1);
        for (int j = 0; j < face_paras->part_T_Star_index[index].size(); ++j)
        {
            int k = face_paras->part_T_Star_index[index][j];
            part_b0.block(j*3, 0, 3, 1) = face_paras->neutral_expression.block(k*3, 0, 3, 1);
        }

        part_b0 = face_paras->LDLT_solver[index].solve(face_paras->GTHGF[index]*part_b0);

        for (int j = 0; j < face_paras->part_T_Star_index[index].size(); ++j)
        {
            int k = face_paras->part_T_Star_index[index][j];
            paras->update_Tib0.block(k*3, index, 3, 1) = part_b0.block(j*3, 0, 3, 1) - face_paras->neutral_expression.block(k*3, 0, 3, 1);
        }
    }
    clock_t cost2 = clock()-start2;

    clock_t start3 = clock();
    #pragma omp parallel for
    for (int i = 0; i < updateEzi.size(); ++i)
    {
        int index = updateEzi[i];
        int row_idx = colNumOfy_z + paras->start_index_E[index];
        int E_num = paras->num_index_E[index];
        Eigen::VectorXd zi = face_paras->u.middleRows(row_idx, E_num);
        Eigen::VectorXd zdiff = zi - face_paras->Last_u.middleRows(row_idx, E_num);
        if (zdiff.cwiseAbs().sum() != 0)
            paras->update_Ezi.col(index).noalias() = paras->E_eigenvectors_T_gl[index].topRows(rowOfupdate)*zi;
    }
    clock_t cost3 = clock()-start3;

    clock_t start4 = clock();

    //T*deltab0 + Ezi
    #pragma omp parallel for schedule(guided)
    for (int i = 0; i < face_paras->delta_B.cols(); ++i)
    {
        if (updateFlag[i])
        {
            face_paras->delta_B.col(i) = paras->update_Tib0.col(i);
            face_paras->delta_B.block(0, i, paras->init_registerIndex_size*3, 1)
                    += paras->update_Ezi.block(0, i, paras->init_registerIndex_size*3, 1);
        }
    }
    clock_t cost4 = clock()-start4;

    of<<"end update: neutral.."<<updateNeutral<<", "<<paras->sigma_b0<<", cost: "<<cost1<<" , "<<cost2<<" , "<<cost3<<" , "<<cost4<<"\n";
}

double RefineMent::EneragyValue(const Eigen::VectorXd& u)
{
    double val = 0;
    int choose_rows;
    if (face_paras->nonrigid_paras->colorWeight != 0.0)
    {
        choose_rows = paras->init_registerIndex_size * 2;
    }
    else choose_rows = paras->init_registerIndex_size;

    Eigen::VectorXd Au_c(choose_rows);
    Au_c = A.block(0, 0, choose_rows, colNumOfy_z) * u.topRows(colNumOfy_z) - c.topRows(choose_rows);

    for (int i = 0; i < nonzero_and_noconvergent_x_index.size(); ++i)
    {
        int idx = nonzero_and_noconvergent_x_index[i];
        int E_index = paras->start_index_E[idx];
        int E_num = paras->num_index_E[idx];
        Au_c += paras->choose_E_eigenvectors_T_gl[idx].topRows(choose_rows)
                * u.middleRows(colNumOfy_z+E_index, E_num) * face_paras->x[idx];
    }
    val += Au_c.squaredNorm();

    of <<"energy: "<<val;

    for (int i = 0; i < colNumOfy; ++i)
    {
        val += paras->beta1 * (u(i) / face_paras->P_eigenvalues(i)) * (u(i) / face_paras->P_eigenvalues(i));
    }

    of <<", "<<val;

    for (int i = 0; i < colNumOfz; ++i)
    {
        double zValue = u(colNumOfy + i);
        val += paras->beta2 * zValue * zValue*face_paras->E_eigenvalues(i)*face_paras->E_eigenvalues(i)
               + paras->beta3*zValue * zValue;
    }

    of <<", "<<val;

    for (int i = 0; i < paras->num_index_E.size(); ++i)
    {
        int E_index = paras->start_index_E[i];
        int E_num = paras->num_index_E[i];
        for (int j = 0; j < E_num; ++j)
        {
            double zValue = u(colNumOfy_z + E_index + j);
            val += paras->beta2*zValue*zValue*paras->E_eigenvalues_gl[i][j]*paras->E_eigenvalues_gl[i][j]
                    + paras->beta3*zValue*zValue;
        }
    }

    of <<", "<<val<<std::endl;

    return val;
}
