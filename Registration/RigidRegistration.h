#ifndef RIGIDREGISTER_H_INCLUDED
#define RIGIDREGISTER_H_INCLUDED
#include <vector>
#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <algorithm>
#include <math.h>
#include <Eigen/Dense>
#include <fstream>
#include "ICP/ICP.h"
#include "ICP/SparseICP.h"
#include "FacePerformance/FacePerformanceFunction.h"
#include "FacePerformance/FacePerformanceParameter.h"

class RigidRegister
{
public:
    RigidRegister(FacePerformanceParameter* face_paras);
    ~RigidRegister();
    void FitMeshICP();
private:
    FacePerformanceParameter* face_paras;
    RigidRegisterParameter* paras;
    std::ofstream of;
    static int flip_num_count;

};
#endif
