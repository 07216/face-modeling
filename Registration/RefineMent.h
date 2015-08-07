#ifndef REFINEMENT_H_INCLUDED
#define REFINEMENT_H_INCLUDED
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/CholmodSupport>
#include <fstream>
#include "Util/TriMesh_Eigen.h"
#include "FacePerformance/FacePerformanceParameter.h"
#include <time.h>
#include "qdebug.h"
#include <omp.h>

typedef Eigen::SparseMatrix<double> SparseMatrixXd;

class RefineMent
{
public:
	RefineMent();
    RefineMent(FacePerformanceParameter* face_paras);
    void addParas(FacePerformanceParameter* face_paras);
	~RefineMent();

private:
    Eigen::MatrixXd M;
	Eigen::MatrixXd middleMatrix;
	Eigen::VectorXd middleVector;
#ifdef FIX_MOUTH_NOSE_EYE
    std::vector<Eigen::MatrixXd > fixedETE; //contain neutral expression and other expression, size = num_blendshape
#endif
    Eigen::MatrixXd unmatchE;
    Eigen::MatrixXd unmatchETE;
    double weight_fix;
	double middle_s;
	Eigen::VectorXd middle_u;
    std::ofstream of;
	Eigen::MatrixXd A;
    Eigen::VectorXd c;
    Eigen::MatrixXd Acomplete;
    Eigen::VectorXd ucomplete;
    FacePerformanceParameter* face_paras;
    RefineMentParameter* paras;
	double xbar;
    int rowAfterChoose;
	int colNumOfy;
    int colNumOfz;
    int colNumOfy_z;
	int sizeOfx;
    int colNumOfAll;
    int numFrames;
    Eigen::MatrixXd cBeforeChoose;
    int rowBeforeChoose;
    std::vector<int> nonzero_x_index;
    std::vector<int> nonzero_and_noconvergent_x_index;
    std::vector<std::pair<int, int> > nonzero_and_noconvergent_x_index_upper_pair;

    int convergedExpressions;

public:
    void Refine(Eigen::VectorXd &y, Eigen::VectorXd &z0);
    void Initialization();
private:
    void PreProcessing();
    void RegistrationMatrixChoose(Eigen::MatrixXd& originalMatrix, Eigen::MatrixXd *registerMatrix);
    void Compute_A();
    void Compute_c();
    double EneragyValue(const Eigen::VectorXd& u);
    void GaussSeidel(Eigen::MatrixXd &A, const Eigen::VectorXd &D, const Eigen::VectorXd &b, Eigen::VectorXd &u);
    void UpdateB0AndDeltaB(const Eigen::VectorXd &y, const Eigen::VectorXd &z0);
};
#endif
