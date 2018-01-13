#ifndef	 MATOPERATION_H
#define MATOPERATION_H

#include <iostream>
#include <cv.h>
#include <vector>  
using namespace cv;
using namespace std;  

//template <class T>

class CMatOperation
{
public:
	CMatOperation();
	~CMatOperation();
	void all(Mat inMat, bool *pSign);
	void any(Mat inMat, bool *pSign);
	void intersectSet(vector<int> v1, vector<int> v2, vector<int> &vout, vector<int> &keepIdx);
	void intersectSet(Mat v1, Mat v2, Mat &vout, Mat &keepIdx);
	void unionSet(Mat v1, Mat v2, Mat &vout);
	void unionSet(vector<int> v1, vector<int> v2, vector<int> &vout);	
	void unique(Mat mat, Mat &outMat, vector<int> &keptIdx);	
	void sortrows(Mat inMat, Mat &outMat, Mat &sortedComIdx, int primiaryKey, int secondaryKey);
	void uniqueVector(vector<int> inVect, vector<int> &outVect, vector<int> &keepIdx);
	void uniqueVector(Mat inVect, Mat &outMat, Mat &keepIdx);

	void diffSet(vector<float> v1, vector<float> v2, vector<float> &vout, vector<int> &keepIdx);		
	void diffSet(Mat src1, Mat src2, Mat &dst, Mat &keptIdx);		
	void diffSet(Mat src1, Mat src2, Mat &dst);	


	Mat genContinuousMat(int _floor, int _ceil, int step = 1);	
	void eraseRow(Mat src, int n, Mat &dst);
	void eraseRows(Mat src, Mat eRows, Mat dst);
};
#endif  // MATOPERATION_H