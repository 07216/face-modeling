#ifndef POSEESTIMATION_H
#define POSEESTIMATION_H
#include<string>
#include <algorithm>
#include <iostream>
#include <vector>
#include "CRForestEstimator.h"

class PoseEstimation
{
public:
    PoseEstimation();
    ~PoseEstimation();
    void init();
    //redetect face
    //void reInit();
    // load config file
    void loadConfig(const char* fileName);
    bool estimate_pose(cv::Mat& g_im3D);

    //detect face for the first frame
    bool detect_face_from_depthimage(cv::Mat& depthImage, double fx, double fy, unsigned short maxOpenni_depth);
    std::vector< cv::Vec<float,POSE_SIZE> > g_means; //outputs
    std::vector< std::vector< Vote > > g_clusters; //full clusters of votes
    std::vector< Vote > g_votes; //all votes returned by the forest
    int best_cluster_index;

public:
    // Path to trees
    std::string g_treepath;
    // Number of trees
    int g_ntrees;
    // Patch width
    //int g_p_width;
    // Patch height
    //int g_p_height;
    //maximum distance form the sensor - used to segment the person
    int g_max_z;
    //head threshold - to classify a cluster of votes as a head
    int g_th;
    //threshold for the probability of a patch to belong to a head
    float g_prob_th;
    //threshold on the variance of the leaves
    float g_maxv;
    //stride (how densely to sample test patches - increase for higher speed)
    int g_stride;
    //radius used for clustering votes into possible heads
    float g_larger_radius_ratio;
    //radius used for mean shift
    float g_smaller_radius_ratio;
    //pointer to the actual estimator
    CRForestEstimator* g_Estimate;
    cv::Rect face_rect;
};

#endif // POSEESTIMATION_H
