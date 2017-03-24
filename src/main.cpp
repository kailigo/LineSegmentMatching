
extern "C"
{
#include "lsd.h"
};


#include <stdio.h>
#include <highgui.h>
#include <cv.h>
#include <math.h>
#include <iostream>
#include "IO.h"
#include "LineMatching.h"
#include "PartiallyRecoverConnectivity.h"
#include "CmdLine.h"

using namespace std;  
using namespace cv;


void detectLine(char* imfile, Mat &mLines, float minLineLength)
{
	IplImage* im = cvLoadImage(imfile,CV_LOAD_IMAGE_GRAYSCALE);
	image_double image = new_image_double(im->width, im->height);
	unsigned char* im_src = (unsigned char*)im->imageData;
	int xsize = image->xsize;
	int ysize = image->ysize;
	int y,x;
	for (y = 0;y < ysize;y++)
	{
		for (x = 0;x < xsize;x++)
		{
			image->data[y * xsize + x] = im_src[y * im->widthStep + x];
		}
	}
	ntuple_list detected_lines = lsd(image);
	free_image_double(image);

	int nLines = detected_lines->size;	
	int nCount = 0;
	int i,j;
	int dim = detected_lines->dim;
	
	Mat tmat;
	for (i = 0;i < nLines;i++)
	{				
		float a1 = detected_lines->values[i*dim+0];
		float a2 = detected_lines->values[i*dim+1];
		float a3 = detected_lines->values[i*dim+2];
		float a4 = detected_lines->values[i*dim+3];
		if (sqrt( (a3-a1)*(a3-a1)+ (a4-a2)*(a4-a2) ) >= minLineLength)
		{
			tmat = (Mat_<float>(1, 4)<<a1, a2, a3, a4);
			mLines.push_back(tmat);
		}		
	}	
}

void drawDectectedLine(Mat Img, Mat mLines, string imgName )
{	
	int color[6][3];
	color[0][0]=255;color[0][1]=0;  color[0][2]=0;
	color[1][0]=0;  color[1][1]=255;color[1][2]=0;
	color[2][0]=0;  color[2][1]=0;  color[2][2]=255;
	color[3][0]=255;color[3][1]=255;color[3][2]=0;
	color[4][0]=0;color[4][1]=255; color[4][2]=255;
	color[5][0]=255;color[5][1]=0; color[5][2]=255;

	int nline = mLines.rows;
	for (int i = 0; i < nline; i++)
	{
		float *pdat = mLines.ptr<float>(i);
		int color_num=rand()%6;
		int R=color[color_num][0];
		int G=color[color_num][1];
		int B=color[color_num][2];
		
		Point2f spt1 = Point2f( pdat[0], pdat[1]);
		Point2f ept1 = Point2f( pdat[2], pdat[3] ); 
		line(Img, spt1, ept1, cvScalar(R,G,B), 2 );		
	}	

	imshow(imgName, Img);
//	imwrite(imgName, Img);
	waitKey(20);	
}

void drawPartiallyConnectedLine(Mat Img, Mat mLines, string imgName, Mat fans)
{	
	int color[6][3];
	color[0][0]=255;color[0][1]=0;  color[0][2]=0;
	color[1][0]=0;  color[1][1]=255;color[1][2]=0;
	color[2][0]=0;  color[2][1]=0;  color[2][2]=255;
	color[3][0]=255;color[3][1]=255;color[3][2]=0;
	color[4][0]=0;color[4][1]=255; color[4][2]=255;
	color[5][0]=255;color[5][1]=0; color[5][2]=255;

	int nline = mLines.rows;
	for (int i = 0; i < nline; i++)
	{
		float *pdat = mLines.ptr<float>(i);
		int color_num=rand()%6;
		int R=color[color_num][0];
		int G=color[color_num][1];
		int B=color[color_num][2];

		Point2f spt1 = Point2f( pdat[0], pdat[1] );
		Point2f ept1 = Point2f( pdat[2], pdat[3] ); 
		line(Img, spt1, ept1, cvScalar(R,G,B), 2 );		
	}	
	int nFan = fans.rows;
	for (int i = 0; i < nFan; i++ )
	{
		int color_num=rand()%6;
		int R=color[color_num][0];
		int G=color[color_num][1];
		int B=color[color_num][2];
		float *pdat = fans.ptr<float>(i);
		circle(Img, Point2f(pdat[0], pdat[1]), 5, cvScalar(R,G,B), -1);
	}
	imshow(imgName, Img);
	waitKey(20);	
}

int main(int argc, char** argv)
{		
	Torch::CmdLine cmd;	

	cmd.addText("\n**********************************************************************");
	cmd.addText("**            PROGRAM:   Line Matching                              **");
	cmd.addText("**                                                                  **");
	cmd.addText("**             Author:   Kai LI                                     **");
	cmd.addText("**                       School of Remote Sensing & Inf. Eng.       **");
	cmd.addText("**                       Wuhan University, Hubei, P.R. China        **");
	cmd.addText("**             Emails:   kaili@whu.edu.cn                           **");
	cmd.addText("**       Created Date:   January 1, 2015                            **");
	cmd.addText("**               Date:   January 23, 2015                           **");

	cmd.addText("\nArguments:");	

 	char*  imgName1;
 	char*  imgName2;
	
	cmd.addSCmdArg("First Image",		&imgName1, "the filename of first image");
	cmd.addSCmdArg("Second Image", &imgName2, "the filename of second image");

	cmd.addText("\nOptions:");	
	real minLineLength = 0;
	bool  isVerbose = true;
	real  expandWidth =20.0;	
	bool isBuildingImagePyramids = true;	
	int   nAvgDesDist = 2;
	bool isScaleChangeExisting  = false;
	bool isProvideJunc = 0;
	bool isProvideLines = 0;
	bool isTwoLineHomography = true;
	char* outLineMatchesFileName = "result.txt";		
	int nOctave = 4;
	int nOctaveLayer = 2;
 	real  desDistThrEpi = 0.4;
	real desDistThrProg = 0.5;
 	real  fmatThr = 3.0;
 	real  hmatThr = 5.0;
 	int   nNeighborPts = 10;	
	int  nEnterGroup = 4;
	real rotAngThr = 30*CV_PI/180;
	real sameSideRatio = 0.8;
	real regionHeight = 4;
	real junctionDistThr =5.0;
	char* providedLines1;
	char* providedLines2;
	char* providedJunc1;
	char* providedJunc2;
	real intensityProfileWidth = 3;
	real radiusPointMatchUnique = 0;
	real difAngThr = 20*CV_PI/180;	
	real rcircle = 10;
	real truncateThr = 0.3;
	real fanThr = 1.0/4 * CV_PI;
	
	cmd.addRCmdOption("-ew", &expandWidth, expandWidth, "the paramter that controls the size of the affect region a line segment");
	cmd.addRCmdOption("-rc", &rcircle, rcircle, "the radius of the smaller circle used constructing LJL descriptor");
	cmd.addBCmdOption("-vb", &isVerbose, isVerbose, "does show intermidiate result? [true]");		
	cmd.addBCmdOption("-bip", &isBuildingImagePyramids, isBuildingImagePyramids, "does build Gaussian image pyramids? [true]");
	cmd.addICmdOption("-noct", &nOctave, nOctave, "the number of octaves");
	cmd.addICmdOption("-nlay", &nOctaveLayer, nOctaveLayer, "the number of layers in each octave");
	cmd.addICmdOption("-nadd", &nAvgDesDist,	 nAvgDesDist, "the number of sub-descriptor distance values used for calculating mean value");
	cmd.addSCmdOption("-olmfn", &outLineMatchesFileName, "result.txt", "output line match file name");
	cmd.addICmdOption("-nnp", &nNeighborPts,	 nNeighborPts, "the number of neighbor matched points for  topological constraints");	
	cmd.addRCmdOption("-tt", &truncateThr, truncateThr, "the threshold for truncate while constructing the descriptor");
	cmd.addRCmdOption("-dat", &difAngThr, difAngThr, "the threshold for the angle difference of an accept LJL match");

	cmd.addBCmdOption("-pj",   &isProvideJunc,  isProvideJunc, "is providing junctions? [false]");
	cmd.addSCmdOption("-pj1", &providedJunc1, "nodes1.txt", "file name of the provided junctions in the first image");
	cmd.addSCmdOption("-pj2", &providedJunc2,  "nodes2.txt", "file name of the provided junctions in the second image");
	cmd.addBCmdOption("-pl",   &isProvideLines, isProvideLines, "is providing lines? [false]");
	cmd.addSCmdOption("-pl1", &providedLines1, "lines1.txt",  "file name of the  provided lines in the first image");
	cmd.addSCmdOption("-pl2", &providedLines2,  "lines2.txt", "file name of the provided lines in the second image");
	/*
	cmd.addBCmdOption("-tlh", &isTwoLineHomography, isTwoLineHomography, "if use two lines to calulate local homograpy? [false]");

	cmd.addRCmdOption("-ddte", &desDistThrEpi, desDistThrEpi, "the threshold for the distance for the description vectors for fundamental matrix estimation");
	cmd.addRCmdOption("-ddtp", &desDistThrProg, desDistThrProg, "the threshold for the distance for the description vectors for propagation");
	cmd.addRCmdOption("-fmt", &fmatThr, fmatThr, "the threshold for the distance according to fundamental matrix");
	cmd.addRCmdOption("-hmt", &hmatThr,	 hmatThr, "the threshold for the distance of homography");
	
	cmd.addICmdOption("-neg", &nEnterGroup, nEnterGroup, "the number of groups to which each single line or junction are distributed");
	cmd.addRCmdOption("-rat", &rotAngThr, rotAngThr, "rotation angle threshold difference for an accepted line match to its near line matches");
	cmd.addRCmdOption("-ssr", &sameSideRatio, sameSideRatio, "the ratio of neighbor matched points lying on the same sides of an accepted RJ match");
	cmd.addRCmdOption("-rh", &regionHeight, regionHeight, "the height of the affected region of line segments");
	cmd.addRCmdOption("-mll", &minLineLength, minLineLength, "the minimal length for an accepted line segment used for matching");
	cmd.addRCmdOption("-jdt", &junctionDistThr, junctionDistThr, "the threshold for the distance of the junctions according to the homography");
	cmd.addRCmdOption("-ipw", &intensityProfileWidth, intensityProfileWidth, "the width of the profile calculating intensity direction");
	cmd.addRCmdOption("-rpmu", &radiusPointMatchUnique, radiusPointMatchUnique, "the radius controlling the region with unique point match");	
	cmd.addRCmdOption("-ft", &fanThr, fanThr, "the threshold for contructing fans");
	*/


	cmd.read(argc, argv);	
	
   Mat colorImg1= imread(imgName1, 3);    
   Mat colorImg2= imread(imgName2, 3);    
   Mat img1, img2;
   cvtColor(colorImg1, img1, CV_RGB2GRAY);
   img1.convertTo(img1, CV_32FC1);
   cvtColor(colorImg2, img2, CV_RGB2GRAY);
   img2.convertTo(img2, CV_32FC1);

   Mat nodes1, nodes2, lines1, lines2;
   Mat keyPoints1, keyPoints2;
   if (isProvideLines)
   {
	   CIO io;
	   io.loadData(providedLines1, lines1);
	   io.loadData(providedLines2, lines2);

	   if (minLineLength != 0)
	   {
		   Mat tlines1, tlines2;
		  Mat ta = lines1.row(2) - lines1.row(0);
		  Mat tb = lines1.row(3) - lines1.row(1);
		  Mat tc = ta.mul(ta) + tb.mul(tb);
		  float *pdat = tc.ptr<float>(0);
		  int nline1 = lines1.rows;
		  for (int i = 0; i < nline1; i++ )
		  {
			  if (pdat[i] < minLineLength)
				  continue	;

			tlines1.push_back(lines1.row(i));  
		 }
		 lines1 = tlines1;

		 ta = lines2.row(2) - lines2.row(0);
		 tb = lines2.row(3) - lines2.row(1);
		 tc = ta.mul(ta) + tb.mul(tb);
		 pdat = tc.ptr<float>(0);
		int nline2 = lines2.rows;
		 for (int i = 0; i < nline2; i++)
		 {
			 if (pdat[i] < minLineLength)			 
				 continue;

			 tlines2.push_back(lines2.row(i));			 
		 }
		lines2 = tlines2;
	   }
   }
   else
   {
	   detectLine(imgName1, lines1, minLineLength);
	   detectLine(imgName2, lines2, minLineLength);
	}
   if (isVerbose)
   {
	   drawDectectedLine(colorImg1.clone(), lines1, "Detected lines in the first image");
	   drawDectectedLine(colorImg2.clone(), lines2, "Detected lines in the second image");
   }
   
   
   if (isProvideJunc)
	{
		CIO io;
		io.loadData(providedJunc1, nodes1);	   
		io.loadData(providedJunc2, nodes2);
	}
	else
	{
		CPartiallyRecoverConnectivity  p1(lines1, expandWidth, nodes1, colorImg1, fanThr);
		CPartiallyRecoverConnectivity  p2(lines2, expandWidth, nodes2, colorImg2, fanThr);	   
	}	
	cout<< "Line segments detected in the two images: ("<<lines1.rows<<","<< lines2.rows<<")"<< endl;	
   if (isVerbose)
   {
	   drawPartiallyConnectedLine(colorImg1.clone(), lines1, "Detected lines and generated junctions in the first image", nodes1);		   
	   drawPartiallyConnectedLine(colorImg2.clone(), lines2, "Detected lines and generated junctions in the second image", nodes2);			   
   }	   

   Mat mlines;
   CLineMatching *pLineMatching  = new CLineMatching(img1, lines1, nodes1, img2, lines2, nodes2, colorImg1, colorImg2, mlines, 
														isVerbose, isBuildingImagePyramids,  nAvgDesDist, isProvideJunc, isTwoLineHomography,
														nOctave, nOctaveLayer, desDistThrEpi, desDistThrProg, fmatThr, hmatThr, nNeighborPts, nEnterGroup,
														rotAngThr, sameSideRatio, regionHeight, junctionDistThr, intensityProfileWidth, radiusPointMatchUnique, difAngThr,
														rcircle, truncateThr, fanThr, outLineMatchesFileName);

   return 0;

   CIO *pIO = new CIO;
   pIO->writeData(outLineMatchesFileName, mlines);

   delete pIO;
   pIO = NULL;  
   delete pLineMatching;
   pLineMatching = NULL;
   return 0;
}