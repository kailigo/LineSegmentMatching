#ifndef PARTIALLYRECOVERCONNECTIVITY_H
#define PARTIALLYRECOVERCONNECTIVITY_H
#include <cv.h>
#include <highgui.h>
using namespace cv;
using namespace std;
class CPartiallyRecoverConnectivity
{
public:
	Mat gImg;
	CPartiallyRecoverConnectivity();
	~CPartiallyRecoverConnectivity();
	CPartiallyRecoverConnectivity(Mat mLines, float radius, Mat &fans, Mat img, float fanThr);
	void ptsDropInRotatedRect(Mat pts, RotatedRect rotRect, vector<int> &vDropInPts);
	bool isPtInRotatedRect(Point2f pt, RotatedRect rotRect);
	void intersectionOfLines(Point2f pt1, Point2f pt2, Point2f pt3, Point2f pt4, Point2f &intpt/*, int rLoc[]*/);
	int   relativeLocationOfIntersection(Point2f pt1, Point2f pt2, Point2f intpt);
};
#endif 