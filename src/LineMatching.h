#ifndef	 LINEMATCHING_H
#define LINEMATCHING_H
#include "cv.h"
#include <highgui.h>
#include "Timer.h"
#include <vector>
using namespace cv;
using namespace std;
//////////////////////////////////////////////////////////

struct strLine
{
	Point2f  ps;
	Point2f  pe;
	double direction;
	bool intensityDir;
	double intenRatio;
	int serial;
	//bool isMatched;
};

struct strBranch 
{
	Point2f  bpt;
	double ang;
	int lineSerial;
};	

struct strFanSection
{
	Point2f  intsection;
	int nodeSerail;
	strBranch strbranch1, strbranch2;	
	Mat mDes;
	int fanSerial;
	//bool isMatched;
};

struct strFanMatch
{
	int fserial1;
	int fserial2;
	float dist;
};

struct strLineMatch
{	
	int serLine1;	
	int serLine2;
	float dist;
};

struct strPointMatch
{	
	Point2f	point1;
	Point2f	point2;	
};

class CLineMatching
{
private:
	Mat colorImg1, colorImg2;
	Mat nodes1, nodes2;
	strLine* strline1, *strline2;
	vector<strLineMatch> vstrLineMatch;
	vector<strFanMatch> vstrFanMatch;
	vector<strPointMatch> vstrPointMatch;
	vector<strFanSection> vstrFanSection1, vstrFanSection2;	
	int nline1, nline2;	
	Mat gMag1,	gDir1, gMag2, gDir2;
	Mat FMat;
	int _nAvgDescriptorDistance;
	bool  _isTwoLineHomog;
	float _donwSampleRatio;
	int _nOctave;
	int _nOctaveLayer;
	float _fmatThr;
	float _hmatThr;
	int  _nNeighborPts;
	float _nEnterGroups;
	float _desDistThrEpi;
	float _rotAngThr;
	float _sameSideRatio;
	float _regionHeight;
	float _junctionDistThr;
	float _intensityProfileWidth;
	float _radiusPointMatchUnique;
	float _difAngThr;
	float _rcircle;
	bool sflag;
	float _truncateThr;
	float _fanThr;
	string _outFileName;

	void extendIntensityValue(Mat &src1, Mat &src2);
	void initialize_providedJunctions(Mat img, strLine* strline, int nline,  Mat &node, vector<strFanSection>  &vstrFanSection);
	void initialize_selfGenJunctions(Mat img, strLine* strline, int nline,  Mat &node, vector<strFanSection>  &vstrFanSection);
	void intensityDirection(Mat img,  strLine* strline, int nline);
	void buildGaussianPyramid(  Mat& base, vector<Mat>& pyr, int nOctaves, int nOctaveLayers, float downsampleRatio);
	void calcGrad(Mat img, Mat &gMag, Mat &gDir);

	void description(vector<strFanSection> &strFanSect,  Mat gMag,  Mat gDir, bool isOutputDesMat, Mat& desMat);
	void description(vector<strFanSection> &strFanSect,  Mat gMag,  Mat gDir);

	void description_fans(vector<strFanSection> &vstrFanSection,  Mat gMag,  Mat gDir, bool isOutputDesMat, Mat& desMat);
	void description_fans(vector<strFanSection> &vstrFanSection,  Mat gMag,  Mat gDir);


	void description(Mat pts,  Mat gMag,  Mat gDir, Mat &descriptors);
	
	void description_fanPts(Mat pts,  Mat gMag,  Mat gDir, Mat &descriptors);	
	void description_singleFan(Vec4f pt,  Mat gMag,  Mat gDir, Mat &des);
	void describeSingleLine(Vec3f pt,  Mat gMag,  Mat gDir, Mat &des);	
	void description_sift(vector<strFanSection> &vstrFanSection,  Mat gMag,  Mat gDir);
	void description_sift_single(Vec3f pt, const Mat gMag, const Mat gDir, Mat &des);
	void fanMatch(vector<strFanSection> v1, vector<strFanSection> v2, float distThr, vector<strFanMatch> &vFanMatch);	
	void plotPointMatches(Mat colorImg1, Mat colorImg2, vector<strPointMatch> vstrPointMatch, string imgName);
	void concatenateTwoImgs(Mat img1, Mat img2, Mat &outImg);
	void findRobustFundamentalMat(vector<Point2f> &PointSet1,vector<Point2f> &PointSet2, Mat &FMat, bool *pbIsKept);
	void getPointsonPolarline(vector<Point2f> &PointSet1,vector<Point2f> &PointSet2,Mat_<double> F,double T, bool *pbIsKept);
	void drawEpipolarline(Mat image1, Mat image2,vector<Point2f> pointSet1,vector<Point2f> pointSet2,Mat_<double> Fmatrix);
	void plotLineMatches(Mat img1, Mat img2, vector<strLineMatch> vStrLineMatch, string imgName);
	void sortrows(Mat inMat, Mat &outMat, Mat &sortedComIdx, int primiaryKey, int secondaryKey);

	void uniqueChk(Mat inMat, Vec3i regulation, vector<int> &vKeptIdx, Mat &outMat);
	void uniqueChk(Mat inMat, Vec3i regulation, vector<int> &vKeptIdx);


	int   descriptorsMatching(Mat pts1, Mat pts2, Mat des1, Mat des2, float distThr, vector<strFanMatch> &vFanMatch);
	void updatePointMatchFromFanMatches();
	bool isAcceptedToBePointMatch(Vec3f pt1, Vec3f pt2, float desDistThr, float fDistThr, float &desDist);
	void getResiduauls(Mat pts1, Mat pts2, Mat FMat, Mat_<float> &residuals);
	void unique(Mat mat, vector<int> &keptIdx);
	void uniquePointMatch(vector<strPointMatch> &vstrPointMatch)	;
	void topoFilter(vector<strFanMatch> &vstrFanMatch, vector<strPointMatch> vstrPointMatch);
	void vpointMatch2Mat(vector<strPointMatch> tvstrPointMatch, Mat &outMat);
	int	determinePtLinePos(Point2f pts, Mat_<float> pline, float refDirection);
	float distPt2Line(Point2f pt, Mat_<float> pline);
	void  addFansMatch(float desDistThr, float fDistThr);
	void  groupFans(vector<strFanSection> vMatchedFans, vector<strFanSection> vUnmatchedFans,  int nEnterCluster, 
							vector< vector<vector<int> > >  &vGroupedUnjunc);
	bool isConsistentWithFMat(Point2f pt1, Point2f pt2, Mat FMat, float FDistThr);
	bool isConsistentWithNeighborPointMatches(strFanSection strFan1, strFanSection strFan2, Mat pointMatch);
	void matchSingleLines(float desDistThr, float fDistThr);
	void updatePointMatchFromSingleLineMatch(vector<strFanMatch> vStrFanMatch, vector<strLineMatch> vStrLineMatch,  Mat FMat, vector<strPointMatch> &vPointMatches);
	void nearestMatchedPointsToLine(vector<strLine> vUnmatchedLines, Mat matchedPoints, int nNearestMatchedPts, Mat &nearestPts);
	void adjustLineMatchEndpoints(vector<strLineMatch> vStrLineMatch);
	void groupSingleLines(vector<strLine> vUnmatchedLines, vector<strFanSection> vMatchedFans, int nEnterGroup, vector< vector< vector<int> > > &vEnteredSingleLines);
	void bifurcateFans(vector<strFanSection> vStrFan1, vector<strFanSection> vStrFan2, vector<strFanMatch> vStrFanMatch, 
		vector<strFanSection> &vMatchedFans1, vector<strFanSection> &vMatchedFans2, vector<strFanSection> &vUnmatchedFans1, vector<strFanSection> &vUnmatchedFans2);
	void bifurcateLines(strLine* vStrLine1, strLine* vStrLine2, vector<strLineMatch> vStrLineMatch, vector<strLine> &vMatchedLines1, vector<strLine> &vMatchedLines2, vector<strLine> &vUnmatchedLines1, vector<strLine> &vUnmatchedLines2);
	float estimateLocalRotateAngle(strFanSection strFan1, strFanSection strFan2);
	bool topoFilter_singleLine(strLine curLine1, strLine curLine2, Mat mNPts1, Mat mNPts2, Mat pointMatches);
	void formAndMatchNewFans(strLine line1,  strLine line2, strFanSection fan1, strFanSection fan2, Mat pointMatches, Mat FMat, 
													strFanSection &tstrFan1, strFanSection &tstrFan2, float &desDist, float desDistThr, float fDistThr);
	Point2f intersectionOfLines(Point2f X1, Point2f Y1, Point2f X2, Point2f Y2);
	void formNewFanPair(Point2f intersection1, Point2f intersection2, Point2f junction1, Point2f junction2, Point2f endpt1, Point2f endpt2, strFanSection &tstrFan1, strFanSection &tstrFan2);
	float matchNewlyFormedFanPair(strFanSection tstrFan, strFanSection  tstrFan2, Mat pointMatches, Mat FMat, float desDistThr, float fDistThr);
	void intersectWithOnePairBranches(strLine line1,  strLine line2, Vec4f branch1, Vec4f branch2, Mat pointMatches, Mat FMat, strFanSection &tstrFan1, strFanSection &tstrFan2, float &desDist, float desDistThr, float fDistThr);
	Mat vStrFanMatch2Mat(vector<strFanMatch> vStrFanMatch);
	void uniqueFanMatch(vector<strFanMatch> &vStrFanMatch);
	Mat vStrLineMatch2Mat(vector<strLineMatch> vStrLinrMatch);
	void uniqueLineMatch(vector<strLineMatch> &vStrLineMatch, vector<int> &keptIdx);	
	void fanMatch2LineMatch(vector<strFanMatch> vStrFanMatch, vector<strLineMatch> &vLineMatch );
	void topoFilterLine(vector<strFanMatch> &vStrFanMatch, vector<strPointMatch> vStrPointMatch, vector<strLineMatch> &vStrLineMatch);
	bool isConsistentWithLocalRotateAngle(strFanSection fan1, strFanSection fan2, float rotAngle);
	void nearestMatchedPtsToFan(vector<strFanSection> vFans, Mat pointMatch, int nNearPts, Mat &NearestMPts);
	bool isCurPairFanConsistentWithNearMPts(strFanSection fan1, strFanSection fan2, Mat nMPts1, Mat nMPts2, Mat pointMatch);
	void affineTransformEstimation(Mat &img1, Mat &node1, strLine* strline1, vector<strFanSection> &vStrFanSect1, Mat &img2, Mat &node2,
		strLine* strline2, vector<strFanSection> &vStrFanSect2, vector<strFanMatch> &vFanMatch);
	void buildZoomPyramid( Mat& base, vector<Mat>& pyr, int nOctaves, float downsampleRatio);
	void matchAffineSequence(Mat oriImg, vector<strFanSection> oriFan, Mat oriDes, Mat oriPts, Mat transImg, vector<strFanSection> transFan, Mat transPts,
		vector<strFanMatch> &maxvMatches, Mat &maxGMag, Mat &maxGDir, vector<strFanSection> &maxvPyrFan, Mat &maxTransMat);
	void matchSingleLines_homography(float homDistThr);
	void calcLocalHomography(strFanSection strfan1, strFanSection strfan2, Mat fmat, Mat &hmat);
	void homoLines(Mat inLines, Mat &hoLines);
	void calcCorrespondence(Mat pts, Mat lines, Mat fmat, Mat &corPts);
	void intersectionOf2Lines_homo(Mat Line1, Mat Line2, Mat &intpt);	
	void hmat_fmatOnePtAndOneLine(Mat fmat, Mat line1, Mat line2, Mat pt1, Mat pt2, Mat &hmat);
	void get_asift_description(Mat img, Mat pts, vector<Mat> &vDes);
	float descriptorDistance(Mat des1, Mat des2, int ncand = 1);
	bool isTwoLineOverlap(Point2f pt1, Point2f pt2, Point2f pt3, Point2f pt4, float &dist, float radius=5);
	bool isConsistentWithHMat(Point2f pt1, Point2f pt2, Mat hmat, float hDistThr);	
	void calcLocalHomography_2lineFMat(strFanSection strfan1, strFanSection strfan2, Mat fmat, Mat &hmat);
	void lineMatches2Mat(Mat &mline);
	bool isLineIntersectRectangle(float linePointX1, float linePointY1, float linePointX2, float linePointY2, float rectangleLeftTopX, 
		float rectangleLeftTopY, float rectangleRightBottomX, float rectangleRightBottomY);
	float distPairPt2HMat(Point2f pt1, Point2f pt2, Mat hmat);
	void descriptorEvaluationUniquePtMatches();	
	Mat genContinuousMat(int _floor, int _ceil, int step = 1)
	{
		Mat outMat;	
		for (int i = _floor; i < _ceil; i += step)
		{
			outMat.push_back(i);		
		}
		return outMat.t();
	}	
	float fast_mod(float th)
	{		
		while(th < 0) th += 2*CV_PI ;
		while(th > 2*CV_PI) th -= 2*CV_PI ;
		return th ;		
	}
	int sign(float fval)
	{
		if(fval > 0)
			return 1;
		else if(fval < 0)
			return -1;
		else
			return 0;
	}

public:
	CLineMatching();
	CLineMatching(Mat rimg,  Mat rline, Mat rnode, Mat qimg,  Mat qline, Mat qnode, Mat colorImg1, Mat colorImg2, Mat &mlines,
		bool isVerb, bool isBuildingImagePyramids, float nAvgDesDist, bool isProvidedJunctions, bool isTwoLineHomog,
		int nOctave, int nOctaveLayer, float desDistThr, float fmatThr, float fmatDesThrProg, float hmatThr, int nNeighborPts, int nEnterGroup,
		float rotAngleThr, float sameSideRatio, float regionHeight, float junctionDistThr, float intensityProfileWidth, float radiusPointMatchUnique, float difAngThr,
		float rcircle, float truncateThr, float fanThr, string outFileName);
		~CLineMatching();	
};

#endif  // LINEMATCHING_H
