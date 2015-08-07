#ifndef FEATUREEXTRACTION_H_INCLUDED
#define FEATUREEXTRACTION_H_INCLUDED
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "LBF_Tracker/LocalBinaryFeature.h"

class FeatureExtraction
{
public:
    std::vector< cv::Point2d > feature;
    cv::Rect face_rect;
    std::vector<int> key_point;

    LocalBinaryFeature lbf;
    int iShapeSize;
    int iPointNum;
    float *pfShape;
public:
    FeatureExtraction();
    ~FeatureExtraction();
    bool GetFacePoint(cv::Mat& img);
};
#endif
