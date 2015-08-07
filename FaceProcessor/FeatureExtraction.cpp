#include "FeatureExtraction.h"

FeatureExtraction::FeatureExtraction()
{
    int index;
    std::ifstream in_kp("Data/LBF_Tracking_Model/key_point_lbf.txt");
    key_point.clear();
    while(in_kp>>index)
    {
        key_point.push_back(index);
    }
    lbf.LoadTrackingModel("Data/LBF_Tracking_Model/Track_51.bin");
    iShapeSize = lbf.GetShapeSize();
    iPointNum = iShapeSize >> 1;
    pfShape = new float[iShapeSize];
}

FeatureExtraction::~FeatureExtraction()
{
}
bool FeatureExtraction::GetFacePoint(cv::Mat& img)
{
    cv::Mat grayImage;
    cv::cvtColor(img, grayImage, CV_RGB2GRAY);
    if( lbf.Tracking(grayImage) )
    {
        lbf.GetKeypoint(pfShape);
        cv::Point2d center(0.0, 0.0);
        feature.resize(key_point.size());
        for(int i=0; i<key_point.size(); i++)
        {
            int index = key_point[i]-1;
            cv::Point2d p1(pfShape[index*2], pfShape[index*2+1]);
            feature[i] = p1;
            //cv::circle(img,p1,1,cv::Scalar(0,255,0),3);
            center += p1;
        }

        if(key_point.size() != 0)
        {
            center.x /= key_point.size();
            center.y /= key_point.size();
            int width = 100;
            int ul_x = center.x - width;
            int ul_y = center.y - width;
            int br_x = center.x + width;
            int br_y = center.y + width;

            ul_x = ul_x >= 6 ? ul_x : 6;
            ul_y = ul_y >= 6 ? ul_y : 6;
            br_x = br_x <= img.cols-7 ? br_x : img.cols-7;
            br_y = br_y <= img.rows-7 ? br_y : img.rows-7;

            face_rect.width = br_x - ul_x;
            face_rect.height = br_y - ul_y;
            face_rect.x = ul_x;
            face_rect.y = ul_y;
        }
        return true;
    }
    else return false;
}

