#include "LocalBinaryFeature.h"
LocalBinaryFeature::LocalBinaryFeature()
{
	pfShapeIn = NULL;
	pStage = NULL;
	pfMeanShape = NULL;
	pfShapeTemp = NULL;
	pfShapeDelta = NULL;
}
LocalBinaryFeature::~LocalBinaryFeature()
{
}
void LocalBinaryFeature::LoadTrackingModel(const char *filename)
{
	FILE* fin= fopen(filename, "rb");
	fread(&iPointNum, sizeof(int), 1, fin);   std::cout<<"point_num: "<<iPointNum<<std::endl;
	iShapeSize = iPointNum<<1;
	fread(&iStageNum, sizeof(int), 1, fin);   std::cout<<"stage_num: "<<iStageNum<<std::endl;
	pfMeanShape = new float[2*iPointNum];
	fread(pfMeanShape, sizeof(float), 2*iPointNum, fin);   std::cout<<"mean_shape"<<std::endl;for(int i=0; i<iPointNum; i++)std::cout<<pfMeanShape[i*2]<<' '<<pfMeanShape[i*2+1]<<std::endl;
	pStage = new RF_Stage[iStageNum];

	for(int i = 0; i < iStageNum; i++)
	{
		std::cout<<"stage: "<<i<<std::endl;
		int iTreeNum = pStage[i].iTreeNum;
		fread(&(iTreeNum), sizeof(int), 1, fin);    std::cout<<"--tree_num: "<<iTreeNum<<std::endl;
		pStage[i].iTreeNum = iTreeNum;
		fread(&(pStage[i].iPerPointTreeNum), sizeof(int), 1, fin);    std::cout<<"--pre_point_tree_num: "<<pStage[i].iPerPointTreeNum<<std::endl;
		pStage[i].pTree = new RF_Tree[iTreeNum];

		for(int j = 0; j < iTreeNum; j++)
		{
			RF_Tree *pTree = pStage[i].pTree + j;
			int iNodeNum;
			fread(&iNodeNum, sizeof(int), 1, fin); //std::cout<<"----node_num: "<<node_num<<std::endl;
			pTree->iNodeNum = iNodeNum;
			pTree->pNode = new RF_Node[iNodeNum];
			for(int k = 0; k < iNodeNum; k++)
			{
				RF_Node* pNode = pTree->pNode + k;
				
				fread(&(pNode->fX1Delta), sizeof(float), 1, fin);
				fread(&(pNode->fY1Delta), sizeof(float), 1, fin);
				fread(&(pNode->fX2Delta), sizeof(float), 1, fin);
				fread(&(pNode->fY2Delta), sizeof(float), 1, fin);
				fread(&(pNode->fThreadhold), sizeof(float), 1, fin);
				fread(&(pNode->iLeftIndex), sizeof(int), 1, fin);
				fread(&(pNode->iRightIndex), sizeof(int), 1, fin);
				fread(&(pNode->bLeftIsNodeLeaf), sizeof(bool), 1, fin);
				fread(&(pNode->bRightIsNodeLeaf), sizeof(bool), 1, fin);
				
			}
		}
		
		int iHeight;
		int iWidth;
		fread(&iHeight, sizeof(int), 1, fin); std::cout<<"----iHeight: "<<iHeight<<std::endl;
		fread(&iWidth, sizeof(int), 1, fin); std::cout<<"----iWidth: "<<iWidth<<std::endl;
		pStage[i].iRegressorDataHeight = iHeight;
		pStage[i].iRegressorDataWidth = iWidth;
		pStage[i].pfRegressorData = new float[iHeight*iWidth];
		fread(pStage[i].pfRegressorData, sizeof(float), iHeight*iWidth, fin);
		pStage[i].iBiasOffset = iHeight*iWidth - iWidth;
	}
	int iHeight;
	int iWidth;
	int iValidClassify;
	fread(&iValidClassify, sizeof(int), 1, fin); std::cout<<"iValidClassify: "<<iValidClassify<<std::endl;
	fread(&iHeight, sizeof(int), 1, fin); std::cout<<"iHeight: "<<iHeight<<std::endl;
	fread(&iWidth, sizeof(int), 1, fin); std::cout<<"iWidth: "<<iWidth<<std::endl;
	pJudge.iValidClassify = iValidClassify;
	pJudge.iRegressorDataHeight = iHeight;
	pJudge.iRegressorDataWidth = iWidth;
	pJudge.pfRegressorData = new float[iHeight*iWidth];
	fread(pJudge.pfRegressorData, sizeof(float), iHeight*iWidth, fin);
	pJudge.iBiasOffset = iHeight*iWidth - iWidth;
	fclose(fin);
	std::cout<<"load tracking model finished!"<<std::endl;
	InitParameters();
}

void LocalBinaryFeature::InitParameters()
{
	pfShapeIn = new float[iShapeSize];
	pfShapeTemp = new float[iShapeSize*3];
	pfShapeDelta = new float[iShapeSize];
	pfShapeSmooth = new float[iShapeSize];
	pfSmoothFramePool = new float[iShapeSize*iSmoothFrameNum];
	isReInitialization = true;
	
    if(!faceDetector.Init("Data/LBF_Tracking_Model/FaceRecognitionModel.bin"))
	{
		std::cout<<"Cannot load model!"<<std::endl;
		exit(0);
	}
}

bool LocalBinaryFeature::ReInitialization()
{
	// get faces
	int iFaceNum;
	if (!faceDetector.GetFaces((const unsigned char*)image.data, image.cols, image.rows, pFaces, &iFaceNum))
		return false;
	std::cout<<"Face Number: "<<iFaceNum<<std::endl;
	if (iFaceNum == 0)
		return false;
	// find largest face
	FaceRecognitionLib::FaceRect rcLargestFace;
    double max_area;
    for(int i=0; i<iFaceNum; i++)
    {
        double temp_area = (pFaces[i].Right - pFaces[i].Left)*(pFaces[i].Bottom - pFaces[i].Top);
        if(i==0 || temp_area>max_area)
        {
            rcLargestFace = pFaces[i];
            max_area = temp_area;
        }

    }
    /*rcLargestFace = *std::max_element(pFaces, pFaces + iFaceNum, [](FaceRecognitionLib::FaceRect &f1, FaceRecognitionLib::FaceRect &f2)
	{
		return (f1.Right - f1.Left)*(f1.Bottom - f1.Top) < (f2.Right - f2.Left)*(f2.Bottom - f2.Top);
    });*/
	cv::Rect faceRect;
	faceRect.x = rcLargestFace.Left;
	faceRect.y = rcLargestFace.Top;
	faceRect.width = rcLargestFace.Right - rcLargestFace.Left;
	faceRect.height = rcLargestFace.Bottom - rcLargestFace.Top;
	cv::Mat image_draw = image.clone();
	cv::rectangle(image_draw, faceRect, cv::Scalar(255), 3);
	
	
	int id =  0;
	for(int i = 0; i < iPointNum; i++)
	{
		pfShapeIn[id] = faceRect.x + pfMeanShape[id]*faceRect.width;
		pfShapeIn[id+1] = faceRect.y + pfMeanShape[id+1]*faceRect.height;
		id += 2;
	}
	return true;
}

bool LocalBinaryFeature::Tracking(cv::Mat& grayImage)
{
	image = grayImage;
	if(isReInitialization)
	{
		clock_t start = clock();
		if(!ReInitialization())
		{
			return false;
		}
		ComputeDiffSimTransCoeff(affineCoeff, pfShapeIn, pfMeanShape, iShapeSize, pfShapeTemp);
		ComputeInvRotation(rotationCoeffInv, affineCoeff);
	}
	else 
	{
		AffineCoeff9 tempAffineCoeff;
		ComputeDiffSimTransCoeff(tempAffineCoeff, pfMeanShape, pfShapeIn, iShapeSize, pfShapeTemp);
		AffineTransform(pfShapeIn, iShapeSize, tempAffineCoeff, pfMeanShape);
		ComputeInvAffine(affineCoeff, tempAffineCoeff);
		rotationCoeffInv[0] = tempAffineCoeff[0];
		rotationCoeffInv[1] = tempAffineCoeff[1];
		rotationCoeffInv[2] = tempAffineCoeff[3];
		rotationCoeffInv[3] = tempAffineCoeff[4];
		
	}
		
	float fScoreFlip;
	for(int i = 0; i < iStageNum; i++)
	{
		//compute deltaS
		memcpy(pfShapeDelta, pStage[i].pfRegressorData + pStage[i].iBiasOffset, sizeof(float)*iShapeSize);
		fScoreFlip = pJudge.pfRegressorData[pJudge.iBiasOffset];
		int iPerPointTreeNum = pStage[i].iPerPointTreeNum;
		int treeIdx = 0;		 
		for(int j = 0; j < iPointNum; j++)
		{
			for(int k=0; k<iPerPointTreeNum; k++)
			{
				RF_Tree *pTree = pStage[i].pTree + treeIdx;
				int treeLeafNodeIdx = GetLeafNodeIndex(pTree->pNode, j);
				AddupShape(pfShapeDelta, pStage[i].pfRegressorData+treeLeafNodeIdx*iShapeSize, iShapeSize);
				fScoreFlip += pJudge.pfRegressorData[treeLeafNodeIdx];
				treeIdx ++;
			}
		}
		int id = 0;
		for(int j = 0; j < iPointNum; j++)
		{
			pfShapeIn[id] += rotationCoeffInv[0]*pfShapeDelta[id] + rotationCoeffInv[1]*pfShapeDelta[id+1];
			pfShapeIn[id+1] += rotationCoeffInv[2]*pfShapeDelta[id] + rotationCoeffInv[3]*pfShapeDelta[id+1];
			id += 2;
		}
		
	}

	if(isReInitialization)
	{
		int offset = 0;
		for(int i = 0; i < iSmoothFrameNum; i++)
		{
			memcpy(pfSmoothFramePool+offset, pfShapeIn, sizeof(float)*iShapeSize);
			offset += iShapeSize;
		}
	}
	
	//judge whether flipped or not
	//std::cout<<"score flip: "<<fScoreFlip<<std::endl;
	if(fScoreFlip>0)
	{
		SmoothShape();
		isReInitialization = false;
		return true;
	}
	else 
	{
		isReInitialization = true;
		return false;
	}
	
}

int LocalBinaryFeature::GetLeafNodeIndex(RF_Node* pNode, int landmarkIndex)
{
	RF_Node* pNodeTemp = pNode;
	int id = landmarkIndex << 1;
	while(1)
	{
		float x1_delta_mean = pNodeTemp->fX1Delta;
		float y1_delta_mean = pNodeTemp->fY1Delta;
		float x2_delta_mean = pNodeTemp->fX2Delta;
		float y2_delta_mean = pNodeTemp->fY2Delta;
		float threadhold = pNodeTemp->fThreadhold;

		float x1_delta = rotationCoeffInv[0]*x1_delta_mean + rotationCoeffInv[1]*y1_delta_mean;
		float y1_delta = rotationCoeffInv[2]*x1_delta_mean + rotationCoeffInv[3]*y1_delta_mean;
		float x2_delta = rotationCoeffInv[0]*x2_delta_mean + rotationCoeffInv[1]*y2_delta_mean;
		float y2_delta = rotationCoeffInv[2]*x2_delta_mean + rotationCoeffInv[3]*y2_delta_mean;

		float x1 = pfShapeIn[id] + x1_delta;
		float y1 = pfShapeIn[id+1] + y1_delta;
		float x2 = pfShapeIn[id] + x2_delta;
		float y2 = pfShapeIn[id+1] + y2_delta;

		int x1i = x1 < 0.0f ? 0 : (int)x1;
		x1i = x1i > image.cols - 1 ? image.cols-1 : x1i;
		int y1i = y1 < 0.0f ? 0 : (int)y1;
		y1i = y1i > image.rows - 1 ? image.rows-1 : y1i;
		int x2i = x2 < 0.0f ? 0 : (int)x2;
		x2i = x2i > image.cols - 1 ? image.cols-1 : x2i;
		int y2i = y2 < 0.0f ? 0 : (int)y2;
		y2i = y2i > image.rows - 1 ? image.rows-1 : y2i;

		float fPixelValue1 = (float)image.at<unsigned char>(y1i, x1i);
		float fPixelValue2 = (float)image.at<unsigned char>(y2i, x2i);

		float judgeVal = (fPixelValue1-fPixelValue2) / (fPixelValue1+fPixelValue2+min_float);
		if(judgeVal<threadhold)
		{
			if(pNodeTemp->bLeftIsNodeLeaf)
			{
				return pNodeTemp->iLeftIndex;
			}
			else
			{
				pNodeTemp = pNode + pNodeTemp->iLeftIndex;
			}
		}
		else 
		{
			if(pNodeTemp->bRightIsNodeLeaf)
			{
				return pNodeTemp->iRightIndex;
			}
			else
			{
				pNodeTemp = pNode + pNodeTemp->iRightIndex;
			}
		}
	}
	return -1;
}

void LocalBinaryFeature::SmoothShape()
{

	//越后面的帧离当前帧越近
	static const float fHSmooth = 0.00000005f;
	memcpy(pfShapeSmooth, pfShapeIn, sizeof(float)*iShapeSize);
	int id = 0;
	for(int i = 0; i < iPointNum; i++, id += 2)
	{
		/*float fMaxNorm = 0.0f;
		int offset = 0;
		for(int j = 0; j < iSmoothFrameNum; j++, offset+=iShapeSize)
		{
			float x = pfShapeIn[id] - pfSmoothFramePool[offset+id];
			float y = pfShapeIn[id+1] - pfSmoothFramePool[offset+id+1];
			float fCurNorm = std::expf(x*x+y*y);
			if(fCurNorm > fMaxNorm) fMaxNorm = fCurNorm;
		}
		
		offset = 0;
		float fWeightTotal = 1.0; //current frame weight
		for(int j = 0; j < iSmoothFrameNum; j++, offset+=iShapeSize)
		{
			float fWeight = std::expf( -(iSmoothFrameNum-j)*fHSmooth*fMaxNorm );
			fWeightTotal += fWeight;
			pfShapeSmooth[id] += pfSmoothFramePool[offset+id] * fWeight;
			pfShapeSmooth[id+1] += pfSmoothFramePool[offset+id+1] * fWeight;
			//target_shape.col(j) += pre_frame[i].col(j) * weight;
		}*/
		int offset = 0;
		float fWeightTotal = 1.0; //current frame weight
		for(int j = 0; j < iSmoothFrameNum; j++, offset+=iShapeSize)
		{
			float x = pfShapeIn[id] - pfSmoothFramePool[offset+id];
			float y = pfShapeIn[id+1] - pfSmoothFramePool[offset+id+1];
			float fCurNorm = std::expf(x*x+y*y);
			float fWeight = std::expf( -(iSmoothFrameNum-j)*fHSmooth*fCurNorm );
			fWeightTotal += fWeight;
			pfShapeSmooth[id] += pfSmoothFramePool[offset+id] * fWeight;
			pfShapeSmooth[id+1] += pfSmoothFramePool[offset+id+1] * fWeight;
		}
		pfShapeSmooth[id] /= fWeightTotal;
		pfShapeSmooth[id+1] /= fWeightTotal;
	}
	
	memcpy(pfSmoothFramePool, pfSmoothFramePool+iShapeSize, sizeof(float)*iShapeSize*(iSmoothFrameNum-1));
	memcpy(pfSmoothFramePool+iShapeSize*(iSmoothFrameNum-1), pfShapeIn, sizeof(float)*iShapeSize);
}

void LocalBinaryFeature::GetKeypoint(float *pfOutShape)
{
	if(pfOutShape == NULL)
	{
		pfOutShape = new float[iShapeSize];
	}
	memcpy(pfOutShape, pfShapeSmooth, sizeof(float)*iShapeSize);
}
int LocalBinaryFeature::GetShapeSize()
{
	return iShapeSize;
}

void LocalBinaryFeature::SetReInitializetion()
{
	isReInitialization = true;
}
