#ifndef TRIMESH_EIGEN_H
#define TRIMESH_EIGEN_H
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include "ICP/KDTreeAdaptor.h"
#include <opencv/cv.h>
typedef nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple> KDTree;
class TriMesh_Eigen
{
public:
    Eigen::Matrix3Xd vertices;
    Eigen::Matrix3Xd colors;
    Eigen::Matrix3Xd normals;
    Eigen::Matrix3Xi faces; //index
    KDTree* kdtree;
    cv::Mat img3D_pose;     //store the face part for pose estimation

    //std::vector<int> correspond_index;// store the mesh's vertices index which have corresponding point on the pointcloud
public:
    void clear();
    void write(const char *);
};

#endif // TRIMESH_EIGEN_H
