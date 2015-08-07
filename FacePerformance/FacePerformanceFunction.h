#ifndef FACEPERFORMANCEFUNCTION_H
#define FACEPERFORMANCEFUNCTION_H
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <ObjectTypes/PolyMesh/PolyMesh.hh>
#include <ObjectTypes/TriangleMesh/TriangleMesh.hh>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "Util/TriMesh_Eigen.h"
#include <fstream>
#include <iostream>
#include <QString>
#include "Util/ConfigParameter.h"
typedef Eigen::SparseMatrix<double> SparseMatrixXd;
typedef Eigen::Triplet<double> T;

namespace FacePerformanceFunction
{
    double BilateralFilter(cv::Mat& depthImage, int y, int x, double sigma_color2_inv_half, double sigma_space2_inv_half, int R=6);
    //openni generate point cloud
    void GeneratePointCloud( TriMesh_Eigen& depthMesh_Eigen, double fx, double fy, cv::Rect& face_rect, cv::Mat& colorImage, cv::Mat& depthImage, unsigned short maxDepth, Eigen::MatrixXi& inDepthImage, int radius_bilateral);
    void ReadMatrixBinary(Eigen::MatrixXd &mat, FILE *input);
    void SaveSparseMatrix(SparseMatrixXd& matrix, const std::string & filename);
    void LoadSparseMatrix(SparseMatrixXd& matrix, const std::string & filename);
    void AddFace2Obj(const std::string& filename);
    void AddFace2Off(const std::string& filename);
    void PrintSparseMatrix(const SparseMatrixXd& matrix, const std::string & filename);
    void LoadSelectedIdx2SparseMatrix(const std::string& filename, SparseMatrixXd & spm);
    template<typename VAL> bool is_ieee_nan(const VAL& v) { return !(v == v); }

    template <typename Derived1>
    Eigen::MatrixXd ComputeDerive_f(const Eigen::MatrixBase<Derived1>& realWorldPoint, cv::Mat& colorImage, double fx, double fy)
    {
        Eigen::MatrixXd deriveMat(2,3);
        deriveMat(0,1) = deriveMat(1,0) = 0;
        deriveMat(0,0) = colorImage.cols/(fx*(-realWorldPoint(2,0)));
        deriveMat(1,1) = -colorImage.rows/(fy*(-realWorldPoint(2,0)));
        deriveMat(0,2) = colorImage.cols*realWorldPoint(0,0)/(fx*(-realWorldPoint(2,0))*(-realWorldPoint(2,0)));
        deriveMat(1,2) = -colorImage.rows*realWorldPoint(1,0)/(fy*(-realWorldPoint(2,0))*(-realWorldPoint(2,0)));
        return deriveMat;
    }

    template <typename Derived1>
    Eigen::Vector2i ConvertRealWorldToProjective(const Eigen::MatrixBase<Derived1>& realWorldPoint, const cv::Mat& colorImage, double fx, double fy)
    {
        Eigen::Vector2i coor;
        coor(0) = (realWorldPoint(0,0)/fx/(-realWorldPoint(2,0))+0.5)*colorImage.cols + 0.5;
        coor(1) = (0.5-realWorldPoint(1,0)/fy/(-realWorldPoint(2,0)))*colorImage.rows + 0.5;
        if(coor(0) < 0) coor(0) = 0;
        else if(coor(0) >= colorImage.cols) coor(0) = colorImage.cols - 1;
        if(coor(1) < 0) coor(1) = 0;
        else if(coor(1) >= colorImage.rows) coor(1) = colorImage.rows - 1;
        return coor;
    }

    //template <typename Derived1>
    Eigen::Matrix2Xd ConvertRealWorldToProjective_Matrix(const Eigen::MatrixXd& realWorldPoint, const cv::Mat& colorImage, double fx, double fy);
    template <typename Derived1>
    Eigen::VectorXd Compute_f_color(const Eigen::MatrixBase<Derived1>& realWorldPoint, const cv::Mat& colorImage, double fx, double fy)
    {
        Eigen::Vector2i coor = ConvertRealWorldToProjective(realWorldPoint, colorImage, fx, fy);
        Eigen::VectorXd color(1);
        color(0) = colorImage.at<unsigned char>(coor(1), coor(0));
        color /= 255.0;
        return color;
    }

    Eigen::VectorXd Compute_f_color(const Eigen::Vector2i& coor, const cv::Mat& colorImage);

    template <typename Derived1>
    inline static Eigen::VectorXd Compute_f_color_RGB(const Eigen::MatrixBase<Derived1>& realWorldPoint, const cv::Mat& colorImage, double fx, double fy)
    {
        Eigen::Vector2i coor = ConvertRealWorldToProjective(realWorldPoint, colorImage, fx, fy);
        Eigen::VectorXd color(3);
        color(0) = colorImage.at< cv::Vec3b >(coor(1), coor(0))[0];
        color(1) = colorImage.at< cv::Vec3b >(coor(1), coor(0))[1];
        color(2) = colorImage.at< cv::Vec3b >(coor(1), coor(0))[2];
        color /= 255.0;
        return color;
    }

    template <typename Derived1>
    Eigen::MatrixXd Compute_Derive_Color(const Eigen::MatrixBase<Derived1>& realWorldPoint, cv::Mat& colorImage, double fx, double fy)
    {
        Eigen::Vector2i coor = ConvertRealWorldToProjective(realWorldPoint, colorImage, fx, fy);
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

    Eigen::MatrixXd Compute_Derive_Color(const Eigen::Vector2i& coor, cv::Mat& colorImage);

    void LoadRegisterIndexFromFile(std::vector<int>& index, std::string filename);
    void ComputeNewDepthImageRegion(cv::Rect& face_rect, cv::Mat& img, Eigen::Matrix2Xi& key_point_3d_to_2d);
    void PolyMesh2GLMat(std::string mesh_filename);
    void PolyMesh2GLMat_FrontPart(std::string mesh_filename);
    void ChooseRegisterPoint(const Eigen::MatrixXd &beforeChoose, Eigen::MatrixXd& afterChoose, std::vector<int>& registerIndex );
    inline bool IsInFaceRect(cv::Point2d& colorPoint, cv::Rect& face_rect)
    {
        if(colorPoint.x>=face_rect.x && colorPoint.x<face_rect.x + face_rect.x + face_rect.width
                && colorPoint.y>=face_rect.y && colorPoint.y<face_rect.y + face_rect.height)
            return true;
        else return false;
    }
    inline bool IsInFaceRect(cv::Point& colorPoint, cv::Rect& face_rect)
    {
        if(colorPoint.x>=face_rect.x && colorPoint.x<face_rect.x + face_rect.x + face_rect.width
                && colorPoint.y>=face_rect.y && colorPoint.y<face_rect.y + face_rect.height)
            return true;
        else return false;
    }
    SparseMatrixXd GetUpSamplingSparseMatrix(PolyMesh& mesh, std::vector<int>& init_registerIndex, std::vector<int>& selected_upsamplingIndex);


    template <typename Derived1>
    void GaussSeidel(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, Eigen::MatrixBase<Derived1>& u, int num_iters)
    {

        for (int k = 0; k < num_iters; k++)
        {
            for (int i = 0; i < u.size(); i++)
            {
                double A_ii = A(i,i);
                u(i) = (b(i) + A_ii * u(i) - A.col(i).dot(u))/A_ii;
            }

        }
    }

    void GetBoundaryIndexOfRegisterIndex(const std::vector<int>& registerIndex, const std::string& meshFilename, std::vector<int>& boundaryIndex);
    void GetFacesIndexOfVerticesIndex(const std::vector<int>& verticesIndex, PolyMesh& mesh, std::vector<int>& facesIndex);

    //recognition part
    void SaveInitRegisterPart_binary(const Eigen::MatrixXd& face, const char* filename, int init_registerSize);
    void LoadInitRegisterPart_binary(Eigen::MatrixXd& init_registerFace, const char* filename);
    void ChooseRecognitionFacePart(const Eigen::MatrixXd& src, Eigen::MatrixXd& tar, std::vector<int>& chooseIndex);

    int GetNumFace(std::string face_index_filename = ConfigParameter::facedatabaseDir + "face_index.txt");
    void SaveNumFace(int num_face, std::string face_index_filename = ConfigParameter::facedatabaseDir + "face_index.txt");
}

#endif // FACEPERFORMANCEFUNCTION_H
