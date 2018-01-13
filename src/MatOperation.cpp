#include "MatOperation.h"

CMatOperation::CMatOperation()
{

};

CMatOperation::~CMatOperation()
{

};

void CMatOperation::all(Mat src, bool *pSign)
{
	int rows = src.rows;
	int cols = src.cols;	
	for (int i = 0; i < rows; i++)
	{		
		float* pdat = src.ptr<float>(i);		
		pSign[i] = 1;
		for (int j = 0; j<cols; j++)
		{
			if (pdat[j] == 0)
			{				
				pSign[i] == 0;
				break;
			}
		}		
	}	
};

void CMatOperation::any(Mat src, bool *pSign)
{
	int rows = src.rows;
	int cols = src.cols;	
	for (int i = 0; i < rows; i++)
	{		
		float* pdat = src.ptr<float>(i);		
		pSign[i] = 0;
		for (int j = 0; j<cols; j++)
		{
			if (pdat[j] != 0)				
			{
				pSign[i] = 1;
				break;
			}
		}		
	}	
}



void CMatOperation::intersectSet(vector<int> v1, vector<int> v2, vector<int> &vout, vector<int> &keepIdx)
{
	int cols1 = v1.size();
	int cols2 = v2.size();
	for (int i = 0; i < cols1; i++)
	{
		float val1 = v1[i];
		for (int j = 0; j < cols2; j++)
		{
			float val2 = v2[j];
			if (val1 == val2)
			{
				vout.push_back(val1);
				keepIdx.push_back(i);
				break;
			}			
		}
	}
}

void CMatOperation::uniqueVector(vector<int> inVect, vector<int> &outVect, vector<int> &keepIdx)
{
	vector<float> sortedVal;
	vector<int> sortedIdx;
	cv::sort(inVect, sortedVal, SORT_EVERY_ROW);
	sortIdx(inVect, sortedIdx, SORT_EVERY_ROW);

	//int *pdat1 = sortedIdx.ptr<int>(0);
	//float *pdat2 = sortedVal.ptr<float>(0);
	keepIdx.push_back(sortedIdx[0]);
	outVect.push_back(sortedVal[0]);
	int nelem = inVect.size();
	for (int i = 1; i < nelem; i++)
	{
		if (sortedVal[i] != sortedVal[i-1])
		{
			keepIdx.push_back(sortedIdx[i]);
			outVect.push_back(sortedVal[i]);			
		}
	}
}

void CMatOperation::unionSet(vector<int> v1, vector<int> v2, vector<int> &vout)
{	
	int cols1 = v1.size();
	int cols2 = v2.size();

	for (int i = 0; i < cols2; i++)
	{
		v1.push_back(v2[i]);
	}		
	vector<int> vtmp;
	uniqueVector(v1, vout, vtmp);
}

void CMatOperation::unique(Mat mat, Mat &outMat, vector<int> &keptIdx)
{
	outMat.release();
	Mat tmat, sortedIdx;
	sortrows(mat, tmat, sortedIdx, 0, 1);
	int rows = tmat.rows;
	int* pdat = sortedIdx.ptr<int>(0);
	keptIdx.push_back(pdat[0]);
	bool *pSign = new bool[rows];
	for (int i = 1; i < rows; i++)
	{		
		Mat matSub = tmat.row(i) - tmat.row(i-1);		
		any(matSub, pSign);
		if ( pSign[0] )
		{
			keptIdx.push_back(pdat[i]);
			outMat.push_back(tmat.row(i));
		}
	}
}


void CMatOperation::sortrows(Mat inMat, Mat &outMat, Mat &sortedComIdx, int primiaryKey, int secondaryKey)
{
	int rows = inMat.rows;
	Mat primColumn = inMat.col(primiaryKey).clone();
	Mat secColumn = inMat.col(secondaryKey).clone();
	Mat comCol = primColumn + 0.01*secColumn;	
	sortIdx(comCol,  sortedComIdx, CV_SORT_EVERY_COLUMN);		

	outMat = Mat(inMat.size(), inMat.type());
	int nrows = outMat.rows;
	for (int i = 0;  i < nrows; i++)
	{
		int *pSer = sortedComIdx.ptr<int>(0);
		inMat.row(pSer[i]).copyTo(outMat.row(i));
	}	
}


//////////////////////////////////////////////////////////////////////////

void CMatOperation::intersectSet(Mat v1, Mat v2, Mat &vout, Mat &keepIdx)
{
	int nelem1 = v1.cols;
	int nelem2 = v2.cols;
	int* pdat1 = v1.ptr<int>(0);
	int* pdat2 = v2.ptr<int>(0);

	for (int i = 0; i < nelem1; i++)
	{
		float val1 = pdat1[i];
		for (int j = 0; j < nelem2; j++)
		{
			float val2 = pdat2[j];
			if (val1 == val2)
			{
				vout.push_back(val1);
				keepIdx.push_back(i);
				break;
			}			
		}
	}
}

void CMatOperation::unionSet(Mat v1, Mat v2, Mat &vout)
{	
	int nelem1 = v1.cols;
	int nelem2 = v2.cols;
	int* pdat1 = v1.ptr<int>(0);
	int* pdat2 = v2.ptr<int>(0);
	Mat tv1 = v1.t();
	
	for (int i = 0; i < nelem2; i++)
	{
		tv1.push_back(pdat2[i]);
	}		
	Mat keptIdx;
	uniqueVector(tv1.t(), vout, keptIdx);
}

void CMatOperation::uniqueVector(Mat inMat, Mat &outMat, Mat &keepIdx)
{
	Mat sortedVal, sortedIdx;
	cv::sort(inMat, sortedVal, SORT_EVERY_ROW);
	sortIdx(inMat, sortedIdx, SORT_EVERY_ROW);

	int* pdat1 = sortedIdx.ptr<int>(0);
	int* pdat2 = sortedVal.ptr<int>(0);
		
	keepIdx.push_back(pdat1[0]);
	outMat.push_back(pdat2[0]);
	int nelem = inMat.cols;
	for (int i = 1; i < nelem; i++)
	{
		if (pdat2[i] != pdat2[i-1])
		{
			keepIdx.push_back(pdat1[i]);
			outMat.push_back(pdat2[i]);			
		}
	}
	keepIdx = keepIdx.t();
	outMat = outMat.t();
}

Mat CMatOperation::genContinuousMat(int _floor, int _ceil, int step)
{
	Mat outMat;	
	for (int i = _floor; i < _ceil; i += step)
	{
		outMat.push_back(i);		
	}
	return outMat.t();
}



void CMatOperation::diffSet(vector<float> v1, vector<float> v2, vector<float> &vout, vector<int> &keepIdx)
{
	int cols1 = v1.size();
	int cols2 = v2.size();	
	vout = v1;
	int num = 0;
	for (int i = 0; i < cols1; i++)
	{
		bool flag = 1;
		float val1 = v1[i];
		for(int j = 0; j < cols2; j++)
		{
			float val2 = v2[j];
			if (val1 == val2)
			{
				vout.erase(vout.begin()+(i-num));
				num++;
				flag = 0;
				break;
			}
		}
		if (flag)
		{
			keepIdx.push_back(i);
		}
	}	
}


void CMatOperation::diffSet(Mat src1, Mat src2, Mat &dst, Mat &keptIdx)
{
	int cols1 = src1.cols;
	int cols2 = src2.cols;		
	int num = 0;
	int *pdat1 = src1.ptr<int>(0);
	int *pdat2 = src2.ptr<int>(0);
	for (int i = 0; i < cols1; i++)
	{
		bool flag = 1;
		int val1 = pdat1[i];
		for(int j = 0; j < cols2; j++)
		{
			int val2 = pdat2[j];
			if (val1 == val2)
			{				
				flag = 0;
				break;
			}
		}
		if (flag)
		{
			keptIdx.push_back(i);
			dst.push_back(val1);
		}
	}	
	keptIdx = keptIdx.t();
	dst = dst.t();
}

void CMatOperation::diffSet(Mat src1, Mat src2, Mat &dst)
{
	int cols1 = src1.cols;
	int cols2 = src2.cols;		
	int num = 0;
	int *pdat1 = src1.ptr<int>(0);
	int *pdat2 = src2.ptr<int>(0);
	for (int i = 0; i < cols1; i++)
	{
		bool flag = 1;
		int val1 = pdat1[i];
		for(int j = 0; j < cols2; j++)
		{
			int val2 = pdat2[j];
			if (val1 == val2)
			{				
				flag = 0;
				break;
			}
		}
		if (flag)
		{
			// keptIdx.push_back(i);
			dst.push_back(val1);
		}
	}	
	// keptIdx = keptIdx.t();
	dst = dst.t();
}





void CMatOperation::eraseRows(Mat src, Mat eRows, Mat dst)
{
	cv::sort(eRows, eRows, CV_SORT_EVERY_ROW);
	int nerows = eRows.cols;
	int *pdat = eRows.ptr<int>(0);
	int num = 0;
	for (int i = 0; i < nerows; i++)
	{		
		int ser = pdat[i] - num;
		eraseRow(src, ser, dst);
		num++;
	}
}

void CMatOperation::eraseRow(Mat src, int n, Mat &dst)
{
	if (n = src.cols)
	{
		src.pop_back();
		dst = src;
		return;
	}
	int rows = src.rows;
	int cols  = src.cols;
	dst.create(rows-1, cols, src.type());
	src.colRange(0, n).copyTo(dst.colRange(0, n));
	src.colRange(n+1, cols).copyTo(dst.colRange(n, cols));
}