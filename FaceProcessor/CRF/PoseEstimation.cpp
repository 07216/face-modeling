#include "PoseEstimation.h"
PoseEstimation::PoseEstimation()
{

}
PoseEstimation::~PoseEstimation()
{

}
void PoseEstimation::init()
{
    loadConfig( "Data/CRF/config.txt" );
    g_Estimate =  new CRForestEstimator();
    if( !g_Estimate->loadForest(g_treepath.c_str(), g_ntrees) ){

        std::cerr << "could not read forest!" << std::endl;
        exit(-1);
    }
}

bool PoseEstimation::detect_face_from_depthimage(cv::Mat& depthImage, double fx, double fy, unsigned short g_max_z)
{
    cv::Mat g_im3D(depthImage.rows, depthImage.cols, CV_32FC3);
    int valid_pixels = 0;
    for(int y = 0; y < g_im3D.rows; y++)
    {
        cv::Vec3f* Mi = g_im3D.ptr<cv::Vec3f>(y);
        for(int x = 0; x < g_im3D.cols; x++){
            unsigned short depth = depthImage.at<unsigned short>(y,x);
            float d = (float)depth;

            if (depth < g_max_z && depth > 0 )
            {
                valid_pixels++;

                Mi[x][2] = d;
                Mi[x][0] = (x*1.0/640 - 0.5) * d * fx;
                Mi[x][1] = -(0.5 - y*1.0/480) * d * fy;
            }
            else
                Mi[x] = 0;
        }
    }
    std::cout<<"pose estimation valid pixels: "<<valid_pixels<<std::endl;
    if (valid_pixels >= 100 && estimate_pose(g_im3D) && g_means[best_cluster_index][2] > 0.0f)
    {
        int x = (g_means[best_cluster_index][0] / fx / g_means[best_cluster_index][2] + 0.5) * depthImage.cols;
        int y = (g_means[best_cluster_index][1] / fy / g_means[best_cluster_index][2] + 0.5) * depthImage.rows;

        //face rect
        face_rect.x = std::max(0, x-100);
        face_rect.y = std::max(0, y-150);
        face_rect.width = std::min(220, depthImage.cols-1-face_rect.x);
        face_rect.height = std::min(220, depthImage.rows-1-face_rect.y);
        std::cout<<"face_rect: "<<face_rect.x<<' '<<face_rect.y<<' '<<face_rect.width<<' '<<face_rect.height<<std::endl;

        return true;
    }
    else
    {
        return false;
    }
}

// load config file
void PoseEstimation::loadConfig(const char* filename)
{
    std::ifstream in(filename);
    std::string dummy;
    if(in.is_open()){
        // Path to trees
        in >> dummy;
        in >> g_treepath;

        // Number of trees
        in >> dummy;
        in >> g_ntrees;

        in >> dummy;
        in >> g_maxv;

        in >> dummy;
        in >> g_larger_radius_ratio;

        in >> dummy;
        in >> g_smaller_radius_ratio;

        in >> dummy;
        in >> g_stride;

        in >> dummy;
        in >> g_max_z;

        in >> dummy;
        in >> g_th;

    } else {
        std::cerr << "File not found " << filename << std::endl;
        exit(-1);
    }
    in.close();

    std::cout << std::endl << "------------------------------------" << std::endl << std::endl;
    std::cout << "Estimation:       " << std::endl;
    std::cout << "Trees:            " << g_ntrees << " " << g_treepath << std::endl;
    std::cout << "Stride:           " << g_stride << std::endl;
    std::cout << "Max Variance:     " << g_maxv << std::endl;
    std::cout << "Max Distance:     " << g_max_z << std::endl;
    std::cout << "Head Threshold:   " << g_th << std::endl;

    std::cout << std::endl << "------------------------------------" << std::endl << std::endl;

}
bool PoseEstimation::estimate_pose(cv::Mat& g_im3D)
{
    g_means.clear();
    g_votes.clear();
    g_clusters.clear();

    //do the actual estimation
    //clock_t start = clock();
    g_Estimate->estimate(g_im3D,
                         g_means,
                         g_clusters,
                         g_votes,
                         g_stride,
                         g_maxv,
                         g_prob_th,
                         g_larger_radius_ratio,
                         g_smaller_radius_ratio,
                         false,
                         g_th
                         );
    //std::cout<<clock()-start<<std::endl;
    //std::cout<<g_means.size()<<std::endl;
    if (g_means.size() > 0)
    {
        best_cluster_index = 0;
        int best_cluster_size = g_clusters[0].size();
        for (int i = 1; i < g_clusters.size(); i++)
        {
            if (g_clusters[i].size() > best_cluster_size)
            {
                best_cluster_index = i;
                best_cluster_size = g_clusters[i].size();
            }
        }
        //std::cout << g_means[0][0] << " " << g_means[0][1] << " " << g_means[0][2] << std::endl;
        //std::cout << g_means[0][3] << " " << g_means[0][4] << " " << g_means[0][5] << std::endl;
    }

    return (g_means.size() > 0);
}
