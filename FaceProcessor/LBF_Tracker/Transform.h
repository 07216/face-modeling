#ifndef _TRANSFORM_PARAMETERS_H
#define _TRANSFORM_PARAMETERS_H
#ifndef __out
#define __out
#endif
#ifndef __in
#define __in
#endif
#ifndef __in_opt
#define __in_opt
#endif
typedef float AffineCoeff9[9];
typedef float RotationCoeff4[4];

inline void PrintShape(const float *pfShape, int iShapeSize)
{
	for(int i = 0; i < iShapeSize; i+=2)std::cout<<pfShape[i]<<' ';std::cout<<std::endl;
	for(int i = 1; i < iShapeSize; i+=2)std::cout<<pfShape[i]<<' ';std::cout<<std::endl;
}

inline float NormShape(const float *pfShape, int iShapeSize)
{
	float res = 0.0f;
	for(int i = 0; i < iShapeSize; i++)
	{
		res += pfShape[i] * pfShape[i];
	}
    return sqrtf(res) / iShapeSize;
}

inline void ComputeInvAffine(__out AffineCoeff9& outCoeff, __in const AffineCoeff9& inCoeff)
{
	float a = inCoeff[0];
	float b = inCoeff[1];
	float c = inCoeff[3];
	float d = inCoeff[4];
	float ad_bc_inv = 1.0f / (a*d - b*c);
	outCoeff[0] = d * ad_bc_inv;
	outCoeff[1] = -b * ad_bc_inv;
	outCoeff[2] = -(outCoeff[0]*inCoeff[2] + outCoeff[1]*inCoeff[5]);

	outCoeff[3] = -c * ad_bc_inv;
	outCoeff[4] = a * ad_bc_inv;
	outCoeff[5] = -(outCoeff[3]*inCoeff[2] + outCoeff[4]*inCoeff[5]);

	outCoeff[6] = 0.0f;
	outCoeff[7] = 0.0f;
	outCoeff[8] = 1.0f;
}

inline void AddupShape(float *pfShape, const float *pfShapeDelta, int iShapeSize)
{
	for(int i = 0; i < iShapeSize; i++)
	{
		pfShape[i] += pfShapeDelta[i];
	}
}
//pfShapeOut and pfShapeIn can be the same
inline void AffineTransform(__out float *pfShapeOut, int iShapeSize, const AffineCoeff9 &coeff, __in const float *pfShapeIn)
{
	int iPointNum = iShapeSize>>1;
	int id = 0;
	for(int i = 0; i < iPointNum; i++)
	{
		float a = pfShapeIn[id];
		float b = pfShapeIn[id+1];
		pfShapeOut[id] = coeff[0]*a + coeff[1]*b + coeff[2];
		pfShapeOut[id+1] = coeff[3]*a + coeff[4]*b + coeff[5];
		id += 2;
	}
}

inline void ComputeInvRotation(__out RotationCoeff4 &rotationCoeff, __in const AffineCoeff9 &affineCoeff)
{
	float a = affineCoeff[0];
	float b = affineCoeff[1];
	float c = affineCoeff[3];
	float d = affineCoeff[4];
	float ad_bc_inv = 1.0f / (a*d - b*c);
	rotationCoeff[0] = d * ad_bc_inv;
	rotationCoeff[1] = -b * ad_bc_inv;
	rotationCoeff[2] = -c * ad_bc_inv;
	rotationCoeff[3] = a * ad_bc_inv;
}

inline void ShapeL1Norm(float *pfShape, int iShapeSize, float &fMeanX, float &fMeanY)
{
	int iPointNum = iShapeSize>>1;
	fMeanX = 0.0f;
	fMeanY = 0.0f;
	int id  = 0;
	for(int i = 0; i < iPointNum; i++)
	{
		fMeanX += pfShape[id];
		fMeanY += pfShape[id+1];
		id += 2;
	}
	float fPointNumInv = 1.0f / (float)iPointNum;
	fMeanX *= fPointNumInv;
	fMeanY *= fPointNumInv;

	id = 0;
	for(int i = 0; i < iPointNum; i++)
	{
		pfShape[id] -= fMeanX;
		pfShape[id+1] -= fMeanY;
		id += 2;
	}
}

inline void ComputeSimTransCoeff(__out AffineCoeff9 &coeff, const float *pfShapeOri, const float *pfShapeNew, int iShapeSize)
{
    int iPointNum = iShapeSize>>1;
    float a1 = 0, a3 = 0, a4 = 0;
    float c1 = 0, c2 = 0, c3 = 0, c4 = 0;
    int id = 0;
    for (int i = 0; i < iPointNum; i++)
    {
        float x = pfShapeOri[id];
        float y = pfShapeOri[id+1];
        float xn = pfShapeNew[id];
        float yn = pfShapeNew[id+1];
        a1 += x*x + y*y;
        a3 += x;
        a4 += y;
        c1 += x*xn + y*yn;
        c2 += y*xn - x*yn;
        c3 += xn;
        c4 += yn;
        id += 2;
    }
    float fPointNumInv = 1.0f / (float)iPointNum;
    a1 *= fPointNumInv;
    a3 *= fPointNumInv;
    a4 *= fPointNumInv;
    c1 *= fPointNumInv;
    c2 *= fPointNumInv;
    c3 *= fPointNumInv;
    c4 *= fPointNumInv;

    float t_inv = 1.0f / (a1 - a3*a3 - a4*a4);
    float b1 = t_inv;
    float b2 = a1*t_inv;
    float b3 = -a3*t_inv;
    float b4 = -a4*t_inv;

    float a = b1*c1 + b3*c3 + b4*c4;
    float b = b1*c2 + b4*c3 - b3*c4;
    float dx = b3*c1 + b4*c2 + b2*c3;
    float dy = b4*c1 - b3*c2 + b2*c4;

    coeff[0] = a;
    coeff[1] = b;
    coeff[2] = dx;
    coeff[3] = -b;
    coeff[4] = a;
    coeff[5] = dy;
    coeff[6] = 0;
    coeff[7] = 0;
    coeff[8] = 1;
}


//pfShapeTemp 3 * iShapeSize for fast calculate
inline void ComputeDiffSimTransCoeff(__out AffineCoeff9 &coeff, const float *pfShapeOri, const float *pfShapeNew, int iShapeSize, __in_opt float *pfShapeTemp = NULL)
{
	bool bAllocMem = false;
	if (pfShapeTemp == NULL)
	{
		bAllocMem = true;
		pfShapeTemp = new float[iShapeSize * 3];
	}
	float *pfShapeOriTemp = pfShapeTemp;
	float *pfShapeNewTemp = pfShapeTemp + iShapeSize;
	float *pfShapeOriRotateTemp = pfShapeTemp + iShapeSize * 2;

	//ShapeOri与ShapeNew都减去相应平均值
	float fOriMeanX, fOriMeanY;
	memcpy(pfShapeOriTemp, pfShapeOri, iShapeSize*sizeof(float));
	ShapeL1Norm(pfShapeOriTemp, iShapeSize, fOriMeanX, fOriMeanY);

	float fNewMeanX, fNewMeanY;
	memcpy(pfShapeNewTemp, pfShapeNew, iShapeSize*sizeof(float));
	ShapeL1Norm(pfShapeNewTemp, iShapeSize, fNewMeanX, fNewMeanY);

	//计算减去平均值后的ShapeOri与ShapeNew在x,y方向的方差
	float fShapeOriVarX(0.0), fShapeOriVarY(0.0), fShapeNewVarX(0.0), fShapeNewVarY(0.0);
	{
		int iPointNum = iShapeSize/2;
		int id = 0;
		for (int i = 0; i < iPointNum; i++)
		{
			float x = pfShapeOriTemp[id];
			float y = pfShapeOriTemp[id+1];
			float x_ = pfShapeNewTemp[id];
			float y_ = pfShapeNewTemp[id+1];
			fShapeOriVarX += x*x;
			fShapeOriVarY += y*y;
			fShapeNewVarX += x_*x_;
			fShapeNewVarY += y_*y_;
			id += 2;
		}
		fShapeOriVarX = sqrt(fShapeOriVarX);
		fShapeOriVarY = sqrt(fShapeOriVarY);
		fShapeNewVarX = sqrt(fShapeNewVarX);
		fShapeNewVarY = sqrt(fShapeNewVarY);
	}

	//计算减去平均值后的ShapeOri与ShapeNew在x,y方向的scale比
	float fScaleX = fShapeOriVarX / fShapeNewVarX;
	float fScaleY = fShapeOriVarY / fShapeNewVarY;

	//将scale比乘在ShapeNew上，以去除x,y方向上的scale比差异，这样就可以使用标准SimilarTrans
	for (int i = 0; i < iShapeSize; i+=2)
	{
		pfShapeNewTemp[i] *= fScaleX;
		pfShapeNewTemp[i+1] *= fScaleY;
	}

	ComputeSimTransCoeff(coeff, pfShapeOriTemp, pfShapeNewTemp, iShapeSize);
	float scale = float(1.0 / sqrt(coeff[0]*coeff[0] + coeff[1]*coeff[1]));
	coeff[0] *= scale; coeff[1] *= scale; coeff[2] = 0;
	coeff[3] *= scale; coeff[4] *= scale; coeff[5] = 0;

	AffineTransform(pfShapeOriRotateTemp, iShapeSize, coeff, pfShapeOriTemp);

	float fShapeOriRotateVarX(0.0), fShapeOriRotateVarY(0.0);
	{
		int iPointNum = iShapeSize/2;
		int id = 0;
		for (int i = 0; i < iPointNum; i++)
		{
			float x = pfShapeOriRotateTemp[id];
			float y = pfShapeOriRotateTemp[id+1];
			fShapeOriRotateVarX += x*x;
			fShapeOriRotateVarY += y*y;
			id += 2;
		}
		fShapeOriRotateVarX = sqrt(fShapeOriRotateVarX);
		fShapeOriRotateVarY = sqrt(fShapeOriRotateVarY);
	}

	fScaleX = fShapeNewVarX / fShapeOriRotateVarX;
	fScaleY = fShapeNewVarY / fShapeOriRotateVarY;

	coeff[0] *= fScaleX;
	coeff[1] *= fScaleX;
	coeff[2] = -(fOriMeanX*coeff[0] + fOriMeanY*coeff[1]) + fNewMeanX;
	coeff[3] *= fScaleY;
	coeff[4] *= fScaleY;
	coeff[5] = -(fOriMeanX*coeff[3] + fOriMeanY*coeff[4]) + fNewMeanY;
	coeff[6] = 0;
	coeff[7] = 0;
	coeff[8] = 1;

	if (bAllocMem)
		delete[] pfShapeTemp;
}

#endif
