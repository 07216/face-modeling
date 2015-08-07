#ifndef RANDOMFOREST_HH
#define RANDOMFOREST_HH

struct RF_Node
{
	float fX1Delta;
	float fY1Delta;
	float fX2Delta;
	float fY2Delta;
	float fThreadhold;
	int iLeftIndex;
	int iRightIndex;
	bool bLeftIsNodeLeaf;
	bool bRightIsNodeLeaf;
};

struct RF_Tree
{
	int iNodeNum;
	RF_Node* pNode;
};

struct RF_Stage
{
	int iTreeNum;
	int iPerPointTreeNum;
	RF_Tree* pTree;

	float *pfRegressorData;
	int iRegressorDataWidth;
	int iRegressorDataHeight;
	int iBiasOffset;
};

struct RF_Judge
{
	float *pfRegressorData;
	int iRegressorDataWidth;
	int iRegressorDataHeight;
	int iValidClassify;
	int iBiasOffset;
};
#endif