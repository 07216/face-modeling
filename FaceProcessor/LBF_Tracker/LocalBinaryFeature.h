#ifndef _LOCAL_BINARY_FEATURE_H
#define _LOCAL_BINARY_FEATURE_H

#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <map>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <time.h>
#include "RandomForest.h"
#include "Transform.h"
#include <FaceRecognitionLib.h>
#define min_float 1.0e-14

class LocalBinaryFeature
{
public:
	LocalBinaryFeature();
	~LocalBinaryFeature();
	void LoadTrackingModel(const char *filename);
	bool ReInitialization();
	bool Tracking(cv::Mat& grayImage);
	void GetKeypoint(float *pfShapeOut);
	int GetShapeSize();
	void SetReInitializetion();
	void SmoothShape();
private:
	void InitParameters();
	inline int GetLeafNodeIndex(RF_Node *pTree, int landmarkIndex);
	int iPointNum;
	int iShapeSize;
	int iStageNum;
	RF_Stage *pStage;
	float *pfMeanShape;
	RF_Judge pJudge;

	cv::Mat image;
	float *pfShapeIn;
	float *pfShapeSmooth;
	float *pfShapeTemp;
	float *pfShapeDelta;
	AffineCoeff9 affineCoeff;
	RotationCoeff4 rotationCoeffInv;
	bool isReInitialization;

	FaceRecognitionLib::FaceRecognitionLib faceDetector;
	FaceRecognitionLib::FaceRect pFaces[FaceRecognitionLib::MAX_FACE_NUMBER];

	//smooth shape
	static const int iSmoothFrameNum = 5;
	float *pfSmoothFramePool;
};

#endif