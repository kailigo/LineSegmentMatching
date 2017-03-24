#include "PartiallyRecoverConnectivity.h"
#include "IO.h"
#include "MatOperation.h"
CPartiallyRecoverConnectivity::CPartiallyRecoverConnectivity()
{

}

CPartiallyRecoverConnectivity::~CPartiallyRecoverConnectivity()
{

}

CPartiallyRecoverConnectivity::CPartiallyRecoverConnectivity(Mat mLines, float radius, Mat  &fans, Mat img, float fanThr)
{	
	gImg = img;		
	int rows = mLines.rows;	
	int cols = mLines.cols;
	int imgRows = img.rows;
	int imgCols = img.cols;
	RotatedRect  rotRect;
	
	Mat mPts;
	mLines.colRange(0, 2).copyTo(mPts);
	mPts.push_back(mLines.colRange(2,4).clone());
	CMatOperation *pMatOperation = new CMatOperation;

	///vector<Point2f> vPts;	
	for (int i = 0; i < rows; i++)
	{
		float *pdat = mLines.ptr<float>(i);
		Point2f cenpt;
		cenpt.x =( pdat[0] + pdat[2] ) / 2;
		cenpt.y =( pdat[1] + pdat[3] ) / 2;
		float dy = pdat[3] - pdat[1];
		float dx = pdat[2] - pdat[0];
		float degAng = fastAtan2(dy, dx);
		float arcAng  = degAng / 180 * CV_PI;		
		float length = abs(tan(arcAng)) > 1 ? abs(dy) : abs(dx);		
		CvSize tsize;
		tsize.height = radius * 2;
		tsize.width  = length + 2 * radius;
		rotRect = RotatedRect(cenpt, tsize, degAng);
		vector<int> vDropInPts;		
		ptsDropInRotatedRect(mPts, rotRect, vDropInPts);

		int nKeptPt = vDropInPts.size();
		for (int j = 0; j < nKeptPt; j++ )
		{
			int  curSer = vDropInPts[j] >= rows ? vDropInPts[j] - rows : vDropInPts[j];
			if (curSer == i)
				continue;

			float *pdat1 = mLines.ptr<float>(curSer);

 			float dy1 = pdat1[3] - pdat1[1];
 			float dx1 = pdat1[2] - pdat1[0];
 			float degAng1 = fastAtan2(dy1, dx1);
 			float arcAng1  = degAng1 / 180 * CV_PI;		
 			float tmp = CV_PI;
 			float tmpa = fmod( abs(arcAng-arcAng1), tmp);
 			if(tmpa < fanThr || CV_PI - tmpa < fanThr)			
 				continue;			

			Point2f intpt;
			intersectionOfLines(Point2f(pdat[0], pdat[1]), Point2f(pdat[2], pdat[3]), Point2f(pdat1[0], pdat1[1]), Point2f(pdat1[2], pdat1[3]), intpt/*rLoc*/);
			
			if (isPtInRotatedRect(intpt, rotRect) && (intpt.x >= 4 && intpt.x < imgCols-4 && intpt.y >= 4 && intpt.y < imgRows-4) )
			{		
				float fi = (float) i;
				float fCurSer = (float) curSer;
				Mat fanElem = (Mat_<float>(1, 4) << intpt.x, intpt.y, fi, fCurSer);
				fans.push_back(fanElem);
				//vPts.push_back(Point2f(intpt));
				/* 
				if (rLoc[0] == -1)
				{
					pdat[0] = intpt.x;
					pdat[1] = intpt.y;
				}else if (rLoc[0] == 0)
				{					
					;
				}else
				{
					pdat[2] = intpt.x;
					pdat[3] = intpt.y;
				}
				
				if (rLoc[1] == -1)
				{
					pdat1[0] = intpt.x;
					pdat1[1] = intpt.y;
				}else if (rLoc[1] == 0)
				{
					;				
				}
				else
				{
					pdat1[2] = intpt.x;
					pdat1[3] = intpt.y;
				}				
				*/
			}			
		}		
	}
	Mat tfans, tmat = fans.colRange(2, 4);
	int nfan = fans.rows;
	for (int i = 0; i < nfan; i++)
	{
		float *pdat = tmat.ptr<float>(i);
		int ser1 = pdat[0];
		int ser2 = pdat[1];
		bool flag = 1;
		for (int j = i+1; j < nfan; j++)
		{
			float *pdat1 = tmat.ptr<float>(j);
			int ser3 = (int) pdat1[0];
			int ser4 = (int) pdat1[1];
			if ( (ser1 == ser3 && ser2 == ser4) || (ser1 == ser4 && ser2 == ser3) )
			{
				flag = 0;
				break;
			}
		}
		if (flag)
		{
			tfans.push_back(fans.row(i));
		}
	}	
	fans = tfans;
}

bool CPartiallyRecoverConnectivity::isPtInRotatedRect(Point2f pt, RotatedRect rotRect)
{
	bool tbool = false;
	float hafWidth = rotRect.size.width / 2;
	float hafHeg    = rotRect.size.height / 2;
	float angle = rotRect.angle * CV_PI / 180;
	Point2f cenpt = rotRect.center;
	float dsin = sin(angle);
	float dcos = cos(angle);	
	float fposx = dcos * ( pt.x - cenpt.x) +  dsin  * (pt.y - cenpt.y);
	float fposy = dsin  * ( pt.x - cenpt.x) - dcos * (pt.y  - cenpt.y);	
	if (-hafWidth <= fposx && fposx < hafWidth && -hafHeg <= fposy && fposy < hafHeg)	
			tbool = true;	

	return tbool;
}

void CPartiallyRecoverConnectivity::ptsDropInRotatedRect(Mat pts, RotatedRect rotRect, vector<int> &vDropInPts)
{
	float hafWidth = rotRect.size.width / 2;
	float hafHeg    = rotRect.size.height / 2;
	float angle = rotRect.angle * CV_PI / 180;
	Point2f cenpt = rotRect.center;
	float dsin = sin(angle);
	float dcos = cos(angle);	
	Mat fposx = dcos * (pts.col(0) - cenpt.x) +  dsin * (pts.col(1) - cenpt.y);	
	Mat fposy = dsin  * (pts.col(0) - cenpt.x) - dcos * (pts.col(1) - cenpt.y);
//	cout<<fposy<<endl;
	int npts = pts.rows;
	fposx = fposx.t();
	fposy = fposy.t();
	float* pdat1 = fposx.ptr<float>(0);
	float* pdat2 = fposy.ptr<float>(0);
//	vector<Point2f> vpts;
 //	Point2f tpt;

	for (int i = 0; i < npts; i++)
	{
		if (-hafWidth <= pdat1[i] && pdat1[i] < hafWidth && -hafHeg <= pdat2[i]	 && pdat2[i] < hafHeg)
		{
			vDropInPts.push_back(i);	
	//		tpt.x = pts.at<float>(i, 0);
	//		tpt.y = pts.at<float>(i, 1);
	//	    vpts.push_back(tpt);
		}
	}
	
	int nPts = vDropInPts.size();
	if ( ! nPts )
		return;
	/*
	int R = 255, G = 0, B = 255;
	Point2f vertices[4];
	rotRect.points(vertices);
	line(gImg, vertices[0], vertices[1], cvScalar(R,G,B), 2 );
	line(gImg, vertices[1], vertices[2], cvScalar(R,G,B), 2 );
	line(gImg, vertices[2], vertices[3], cvScalar(R,G,B), 2 );
	line(gImg, vertices[3], vertices[0], cvScalar(R,G,B), 2 );
	 
	for (int i = 0; i < nPts; i++ )
	{  
		circle(gImg,  vpts[i], 3, Scalar(0, 0, 0), -1);
	} 
	imshow("tmp", gImg);
 	waitKey();	
	*/
}

int CPartiallyRecoverConnectivity::relativeLocationOfIntersection(Point2f pt1, Point2f pt2, Point2f intpt)
{
	float dx1 = pt2.x - pt1.x;
	float dx2 = intpt.x - pt1.x;
	float dy1 = pt2.y - pt1.y;
	float dy2 = intpt.y - pt1.y;
	float ratio;

	ratio = dx1 != 0 ? dx2 / dx1 : dy2 / dy1;
	if (ratio >= 1)	
	{
		return 1;
	}
	else if (ratio < 1 && ratio > 0)		
	{
		return 0;
	}
	else
	{
		return -1;
	}	
}

void CPartiallyRecoverConnectivity::intersectionOfLines(Point2f pt1, Point2f pt2, Point2f pt3, Point2f pt4, Point2f &intpt /*, int rLoc[]*/)
{
	float A1 = pt1.y - pt2.y;
	float B1 = pt2.x - pt1.x;
	float C1 = pt2.y * pt1.x - pt1.y * pt2.x;

	float A2 = pt3.y - pt4.y;
	float B2 = pt4.x - pt3.x;
	float C2 = pt4.y * pt3.x - pt3.y * pt4.x;
	Mat_<float>tmat1 = ( Mat_<float>(2,2) << A1, B1, A2, B2 );	
	Mat_<float>tmat2 = ( Mat_<float>(2,2) << -C1, B1, -C2, B2 );	
	Mat_<float>tmat3 = ( Mat_<float>(2,2) << A1, -C1, A2, -C2);	

	float D   = determinant(tmat1);
	float X   = determinant(tmat2) / D;
	float Y   = determinant(tmat3) / D;

	intpt = Point2f(X, Y);
	 
//	rLoc[0] = relativeLocationOfIntersection(pt1, pt2, intpt);
//	rLoc[1] = relativeLocationOfIntersection(pt3, pt4, intpt);	
}
