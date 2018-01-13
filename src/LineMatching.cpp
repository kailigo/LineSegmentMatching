#include "LineMatching.h" 
#include "MatOperation.h"
#include "IO.h"

CLineMatching::CLineMatching(Mat img1,  Mat line1, Mat tnode1, Mat img2,  Mat line2, Mat tnode2, Mat tcolorImg1, Mat tcolorImg2, Mat &mlines, bool isVerbose,
	 bool isBuildingImagePyramids, float nAvgDesDist, bool isProvidedJunctions, bool isTwoLineHomog, int nOctave, int nOctaveLayer, float desDistThrEpi, float desDistThrProg, 
	 float fmatThr, float hmatThr, int nNeighborPts, int nEnterGroup, float rotAngleThr, float sameSideRatio, float regionHeight, float junctionDistThr,
	 float intensityProfileWidth, float radiusPointMatchUnique, float difAngThr, float rcircle, float truncateThr, float fanThr, string outFileName)
{	
	_outFileName = outFileName;
	_isTwoLineHomog = isTwoLineHomog;
	_nOctave = nOctave;
	_nOctaveLayer = nOctaveLayer;	
	_fmatThr = fmatThr;
	_hmatThr = hmatThr;
	_nNeighborPts = nNeighborPts;
	_nEnterGroups = nEnterGroup;	
	_desDistThrEpi = desDistThrEpi;
	_rotAngThr = rotAngleThr;
	_sameSideRatio = sameSideRatio;
	_regionHeight = regionHeight;
	_junctionDistThr = junctionDistThr;
	_intensityProfileWidth = intensityProfileWidth;
	_radiusPointMatchUnique = radiusPointMatchUnique;
	_difAngThr = difAngThr;
	_rcircle = rcircle;
	_truncateThr = truncateThr;
	_fanThr = fanThr;

	colorImg1 = tcolorImg1;
	colorImg2 = tcolorImg2;
	nodes1 = tnode1;
	nodes2 = tnode2;
	extendIntensityValue(img1, img2);		
	Mat tline1 = line1.colRange(0, 4);
	Mat tline2 = line2.colRange(0, 4);		
	nline1 = tline1.rows;	
	strline1 = new strLine[nline1];
	nline2 = tline2.rows;
	strline2 = new strLine[nline2];	
	
	for(int i = 0; i < nline1; i++)
	{
		Point2f tpt;
		const float* dat = tline1.ptr<float>(i);
		tpt.x = dat[0];
		tpt.y = dat[1];
		strline1[i].ps = tpt;
		tpt.x = dat[2];
		tpt.y = dat[3];
		strline1[i].pe = tpt;
		strline1[i].serial = i;
	}
	for(int i = 0; i < nline2; i++)
	{
		Point2f tpt;
		const float* dat = tline2.ptr<float>(i);
		tpt.x = dat[0];
		tpt.y = dat[1];
		strline2[i].ps = tpt;
		tpt.x = dat[2];
		tpt.y = dat[3];
		strline2[i].pe = tpt;
		strline2[i].serial = i;
	}
	
	CTimer mtimer;
	char msg[1024];
	mtimer.Start();

	if (isProvidedJunctions)
	{
		initialize_providedJunctions(img1, strline1, nline1,  nodes1, vstrFanSection1);
		initialize_providedJunctions(img2, strline2, nline2,  nodes2, vstrFanSection2);
	}
	else
	{
		initialize_selfGenJunctions(img1, strline1, nline1,  nodes1, vstrFanSection1);
		initialize_selfGenJunctions(img2, strline2, nline2,  nodes2, vstrFanSection2);	
	}	
	cout<<"LJLs  constructed in the first image: " << vstrFanSection1.size()<<endl;
	cout<<"LJLs constructed in the second image: " << vstrFanSection2.size()<<endl;

	cout<<"Describing and matching LJLs (may last for a while)..." << endl;

	calcGrad(img1, gMag1, gDir1);
	calcGrad(img2, gMag2, gDir2);
	float scales[2] = {1, 1};	
	Mat affineTransMat = (Mat_<float>(2, 3) << 1, 0, 0, 1, 0, 0); 
	_nAvgDescriptorDistance = 1;

 	if (isBuildingImagePyramids)
	{	
		_nAvgDescriptorDistance = nAvgDesDist;
		affineTransformEstimation(img1, nodes1, strline1, vstrFanSection1, img2, nodes2, strline2, vstrFanSection2, vstrFanMatch);			
	}
	else
	{	
		description_fans(vstrFanSection1, gMag1, gDir1);	
		description_fans(vstrFanSection2, gMag2, gDir2);
		fanMatch(vstrFanSection1, vstrFanSection2, desDistThrEpi, vstrFanMatch);
	}
	vector<int> vtmp;
	float t1;

	vstrPointMatch.clear();
	vector<Point2f> vpts1, vpts2;
	strPointMatch tstrPointMatch;
	int nptsMatches = vstrFanMatch.size();
	for(int i = 0; i < nptsMatches; i++)
	{
		int serial1 = vstrFanMatch[i].fserial1;
		Point2f tpt = vstrFanSection1[serial1].intsection;
		tstrPointMatch.point1 = tpt;		
		int serial2 = vstrFanMatch[i].fserial2;
		tpt = vstrFanSection2[serial2].intsection;
		tstrPointMatch.point2 = tpt;		
		vstrPointMatch.push_back(tstrPointMatch);
	}

	int num = 0;
	nptsMatches = vstrFanMatch.size();
	Vec3i regulation = Vec3i(0, 1, 2);
	Mat tmat = Mat(nptsMatches, 3, CV_32F);
	for (int i = 0; i < nptsMatches; i++)
	{
		float* pdat = tmat.ptr<float>(i);
		pdat[0] = vstrFanMatch[i].fserial1;
		pdat[1] = vstrFanMatch[i].fserial2;
		pdat[2] = vstrFanMatch[i].dist;
	}	
	vector<int> vKeptIdx;

	uniqueChk(tmat, regulation, vKeptIdx);	

	int nKeptIdx = vKeptIdx.size();
	vector<strFanMatch>	  tvstrFanMatch;
	vector<strPointMatch> tvstrPointMatch;
	for (int i = 0; i < nKeptIdx; i++)				
	{		 
		int curIdx = vKeptIdx[i];
		tvstrFanMatch.push_back(vstrFanMatch[curIdx]);
		tvstrPointMatch.push_back(vstrPointMatch[curIdx]);					
	}
	vstrPointMatch = tvstrPointMatch;
	vstrFanMatch = tvstrFanMatch;	
	cout<<"Putative LJL matches: "<<tvstrPointMatch.size()<< endl;
	if (isVerbose) plotPointMatches(colorImg1.clone(), colorImg2.clone(), vstrPointMatch, "Putative LJL matches (show the junctions)");		
		
	nptsMatches = vstrPointMatch.size();
	for(int i = 0; i < nptsMatches; i++)
	{		
		Point2f tpt = vstrPointMatch[i].point1;	
		vpts1.push_back(tpt);		
		tpt = vstrPointMatch[i].point2;		
		vpts2.push_back(tpt);				
	}

	bool *pbIsKept = new bool[nptsMatches];	
	findRobustFundamentalMat(vpts1, vpts2, FMat, pbIsKept);	

	CIO io;
	io.writeData("fundamentalMatrix.txt", FMat);

	if (isVerbose) drawEpipolarline(colorImg1.clone(), colorImg2.clone(), vpts1, vpts2, FMat);	
	for (int i = 0; i < nptsMatches; i++)
	{	
		if (pbIsKept[i] == 0)									
		{
			vstrFanMatch.erase(vstrFanMatch.begin()+(i-num));			
			vstrPointMatch.erase(vstrPointMatch.begin()+(i-num));
			num++;
		}
	}	
	if (isVerbose) plotPointMatches(colorImg1.clone(), colorImg2.clone(), vstrPointMatch,"LJL matches filtered by epipolar line contraint");		
//	cout<<"LJL matches after estimating fundamental matirx: " << vstrPointMatch.size()<<endl;
	fanMatch2LineMatch(vstrFanMatch, vstrLineMatch);
		
	uniqueLineMatch(vstrLineMatch,  vtmp);
	if (isVerbose) plotLineMatches(colorImg1.clone(), colorImg2.clone(), vstrLineMatch, "Line matches filtered by epipolar line constraint");
	cout<<"LJL matches filtered by epipolar line constraint: " << vstrFanMatch.size()<<endl;
	cout<<"Line matches filtered by epipolar line constraint: " << vstrLineMatch.size()<<endl;	
	float desDistThr = desDistThrProg;
	float fDistThr = 1.0;
	vector<strPointMatch> vEndPtMatch;
	int preNum = 0, curNum;
	
	mtimer.Stop();
	mtimer.PrintElapsedTimeMsg(msg);	
	t1 = mtimer.GetElapsedSeconds();

	printf("\nElapsed time for the first stage: %s.\n", msg);

	mtimer.Start();

	cout<<"\n"<<"Begin LJL match propagation"<<endl;
	int nitr = 1;
	while (1)
	{
		cout<<"*********Itreration " << nitr<<"**********"<<endl;
		updatePointMatchFromFanMatches();					
		//cout<<"point matches after expanding: "<<vstrPointMatch.size()<<endl;
		topoFilter(vstrFanMatch, vstrPointMatch); 		
		cout<<"LJL matches filtered by topological constraint: " << vstrFanMatch.size()<<endl;
		fanMatch2LineMatch(vstrFanMatch, vstrLineMatch);
		uniqueLineMatch(vstrLineMatch, vtmp);				
		cout<<"Line matches filtered by topological constraint: " << vstrLineMatch.size()<<endl;	
		updatePointMatchFromFanMatches();
		//cout<<"point matches after topological filtering: "<<vstrPointMatch.size()<<endl;
		curNum = vstrFanMatch.size();
		if (nitr > 3 || curNum <= preNum)		
			break;		

		// waitKey();

		preNum = curNum;	
 		addFansMatch(desDistThr, fDistThr);

		cout<<"LJL matches after propagation: " << vstrFanMatch.size()<<endl;
 		updatePointMatchFromFanMatches();		
		//cout<<"point matches after expanding: " << vstrPointMatch.size()<<endl;
		fanMatch2LineMatch(vstrFanMatch, vstrLineMatch);
		uniqueLineMatch(vstrLineMatch, vtmp);
		cout<<"Line matches after propagation: " << vstrLineMatch.size()<<endl;	
		topoFilter(vstrFanMatch, vstrPointMatch);
		//cout<<"LJL matches after topological filtering: " << vstrFanMatch.size()<<endl;
		updatePointMatchFromFanMatches();		
		//cout<<"point matches after expanding: "<<vstrPointMatch.size()<<endl;
		addFansMatch(desDistThr, fDistThr);
		//cout<<"LJL matches after propagation: " << vstrFanMatch.size()<<endl;
		fDistThr += 2.0;
		nitr++;
		cout<<"\n";
	}
	
	if (isVerbose) plotPointMatches(colorImg1.clone(), colorImg2.clone(), vstrPointMatch, "Point matches after propagation");
	if (isVerbose) plotLineMatches(colorImg1.clone(), colorImg2.clone(), vstrLineMatch, "Line matches after propagation");

	preNum = 0;
	curNum = 0;	
	mtimer.Stop();
	mtimer.PrintElapsedTimeMsg(msg);
	printf("\nElapsed time for the second stage: %s.\n", msg);
	float t2 = mtimer.GetElapsedSeconds();

	cout<<"\nBegin matching line segments in individuals"<<endl;
	nitr = 1;
	int prenum = vstrLineMatch.size();

	while (1)
	{					
		//cout<<"*********itreration " << nitr<<"**********"<<endl;
	    matchSingleLines_homography(hmatThr);
		//cout<<"fan matches after single line propagation:" << vstrFanMatch.size()<<endl;
		//cout<<"line matches after single line propagation:" << vstrLineMatch.size()<<endl;
		//cout<<"point matches after single line propagation:" << vstrPointMatch.size()<<endl;

	//	if (isVerbose) plotLineMatches(colorImg1.clone(), colorImg2.clone(), vstrLineMatch, "tmp1.jpg");
		
		int newnum = vstrLineMatch.size();
		if (newnum <= prenum)
		{
			break;
		}
		prenum = newnum;
		nitr++;	
	}
	mtimer.Stop();
	mtimer.PrintElapsedTimeMsg(msg);
	printf("\nElapsed time for the third stage: %s.\n", msg);
	float t3 = mtimer.GetElapsedSeconds();
	float totTime = t1 + t2 + t3;
	cout<<"Final line matches:" << vstrLineMatch.size()<<endl;	
	//cout<<"\nTotal time: "<<totTime<<" seconds";

	lineMatches2Mat(mlines);
	if (isVerbose) plotPointMatches(colorImg1.clone(), colorImg2.clone(), vstrPointMatch, "Final point matches");
	if (isVerbose)
	{ 
		plotLineMatches(colorImg1.clone(), colorImg2.clone(), vstrLineMatch, "Final line matches");	
		waitKey();
	}
	// 
};

void CLineMatching::descriptorEvaluationUniquePtMatches()
{

}

void CLineMatching::lineMatches2Mat(Mat &mline)
{
	string fileName = _outFileName;
	ofstream outFile(fileName.c_str(), ios_base::out);  //按新建或覆盖方式写入  	

	vector<int> vser;
	int nmatch = vstrLineMatch.size();	
	for (int i = 0; i < nmatch; i++)
	{
		int ser1 = vstrLineMatch[i].serLine1;		
		int ser2 = vstrLineMatch[i].serLine2;		
		Mat tmat = (Mat_<float>(1, 8)<<strline1[ser1].ps.x, strline1[ser1].ps.y,
			strline1[ser1].pe.x, strline1[ser1].pe.y, 
			strline2[ser2].ps.x, strline2[ser2].ps.y,
			strline2[ser2].pe.x, strline2[ser2].pe.y);
		mline.push_back(tmat);
		vser.push_back(ser1);
		//	outFile<< ser1 << ' ' << ser2 <<endl;   //每列数据用 tab 隔开  			
	}		
	vector<int> vsidex;
	sortIdx(vser, vsidex, SORT_EVERY_ROW);

	for (int i  = 0; i < nmatch; i++)
	{
		int ser = vsidex.at(i);
		int ser1 = vstrLineMatch[ser].serLine1;		
		int ser2 = vstrLineMatch[ser].serLine2;		
		outFile<< ser1 << ' ' << ser2 <<endl;   //每列数据用 tab 隔开  			
	}
}

void CLineMatching::topoFilterLine(vector<strFanMatch> &vStrFanMatch, vector<strPointMatch> vStrPointMatch, vector<strLineMatch> &vStrLineMatch)
{	
	int nCandNum = _nNeighborPts;
	Mat pointMatch;
	CMatOperation *pMatOperation = new CMatOperation;
	vpointMatch2Mat(vStrPointMatch, pointMatch);	

	int nFanMatch = vStrFanMatch.size();		
	vector<strFanMatch> tvStrFanMatch;	
	for (int i = 0; i < nFanMatch; i++)
	{		
		int ser1 = vStrFanMatch[i].fserial1;		
		strFanSection curFan1 = vstrFanSection1[ser1];
		int ser2 = vStrFanMatch[i].fserial2;		
		strFanSection curFan2 = vstrFanSection2[ser2];		
		if ( ! isConsistentWithNeighborPointMatches(curFan1, curFan2, pointMatch))
			continue;

		tvStrFanMatch.push_back(vStrFanMatch[i]);			
	}
	vStrFanMatch = tvStrFanMatch;

	if ( ! vStrLineMatch.size() )
		vStrLineMatch.clear();
	
	fanMatch2LineMatch(vStrFanMatch, vStrLineMatch);
	vector<int> vKeptIdx;
	for (int i = 0; i < vStrLineMatch.size(); i++)
	{
		int ser2 = vStrLineMatch[i].serLine2;
	}
	uniqueLineMatch(vStrLineMatch, vKeptIdx);

	vector<strLine> vMatchedLines1, vMatchedLines2;
	int nLineMatch = vStrLineMatch.size();					
	for (int i = 0; i < nLineMatch; i++)
	{
		int ser1 = vStrLineMatch[i].serLine1;
		int ser2 = vStrLineMatch[i].serLine2;
		vMatchedLines1.push_back(strline1[ser1]);
		vMatchedLines2.push_back(strline2[ser2]);					
	}
	Mat nearestPts1, nearestPts2;
	nearestMatchedPointsToLine(vMatchedLines1, pointMatch.colRange(0, 2), nCandNum, nearestPts1);	
	nearestMatchedPointsToLine(vMatchedLines2, pointMatch.colRange(2, 4), nCandNum, nearestPts2);
	
	vector<strLineMatch> tvStrLineMatch;
	for (int i = 0; i < nLineMatch; i++)
	{
		strLine tline1 = vMatchedLines1[i];
		strLine tline2 = vMatchedLines2[i];
		Mat mNPts1, mNPts2;
		mNPts1 = nearestPts1.row(i);
		mNPts2 = nearestPts2.row(i);
		bool tbool = topoFilter_singleLine(tline1, tline2, mNPts1, mNPts2, pointMatch);
		if (tbool) 			
			tvStrLineMatch.push_back(vStrLineMatch[i]);			
	}
	vStrLineMatch = tvStrLineMatch;

	delete pMatOperation;
	pMatOperation = NULL;	
}

void CLineMatching::fanMatch2LineMatch(vector<strFanMatch> vStrFanMatch, vector<strLineMatch> &vStrLineMatch)
{
	vStrLineMatch.clear();
	int nFanMatches = vStrFanMatch.size();
	strLineMatch tstrLineMatch;
	for(int i = 0; i < nFanMatches; i++)
	{
		int serial1 = vStrFanMatch[i].fserial1;		
		int serial2 = vStrFanMatch[i].fserial2;		
		tstrLineMatch.serLine1 = vstrFanSection1[serial1].strbranch1.lineSerial;
		tstrLineMatch.serLine2 = vstrFanSection2[serial2].strbranch1.lineSerial;
		tstrLineMatch.dist = vStrFanMatch[i].dist;		
		vStrLineMatch.push_back(tstrLineMatch);
		tstrLineMatch.serLine1 = vstrFanSection1[serial1].strbranch2.lineSerial;
		tstrLineMatch.serLine2 = vstrFanSection2[serial2].strbranch2.lineSerial;
		
		tstrLineMatch.dist = vStrFanMatch[i].dist;		
		vStrLineMatch.push_back(tstrLineMatch);		
	}		
}

void CLineMatching::uniqueLineMatch(vector<strLineMatch> &vStrLineMatch, vector<int> &vKeptIdx)
{
	Mat tmat = vStrLineMatch2Mat(vStrLineMatch);
	//vector<int> vKeptIdx;
	if ( !vKeptIdx.empty() )
		vKeptIdx.clear();
	Vec3i regulation = Vec3i(0, 1, 2);	
	uniqueChk(tmat, regulation, vKeptIdx);	
	int nKeptIdx = vKeptIdx.size();
	vector<strLineMatch>	  tvstrLineMatch;
	for (int i = 0; i < nKeptIdx; i++)				
	{		
		int curIdx = vKeptIdx[i];
		tvstrLineMatch.push_back(vStrLineMatch[curIdx]);
	}	
	vStrLineMatch = tvstrLineMatch;
}

Mat CLineMatching::vStrLineMatch2Mat(vector<strLineMatch> vStrLineMatch)
{
	int nLineMatch = vStrLineMatch.size();
	Mat tmat = Mat(nLineMatch, 3, CV_32F);
	for (int i = 0; i < nLineMatch; i++)
	{
		float* pdat = tmat.ptr<float>(i);
		pdat[0] = vStrLineMatch[i].serLine1;
		pdat[1] = vStrLineMatch[i].serLine2;
		pdat[2] = vStrLineMatch[i].dist;
	}	
	return tmat;
}

void CLineMatching::uniqueFanMatch(vector<strFanMatch> &vStrFanMatch)
{
	Mat tmat = vStrFanMatch2Mat(vStrFanMatch);
	vector<int> vKeptIdx;
	Vec3i regulation = Vec3i(0, 1, 2);	
	uniqueChk(tmat, regulation, vKeptIdx);	
	int nKeptIdx = vKeptIdx.size();
	vector<strFanMatch>	  tvstrFanMatch;	
	for (int i = 0; i < nKeptIdx; i++)				
	{		
		int curIdx = vKeptIdx[i];
		tvstrFanMatch.push_back(vStrFanMatch[curIdx]);
	}	
	vStrFanMatch = tvstrFanMatch;
}

Mat CLineMatching::vStrFanMatch2Mat(vector<strFanMatch> vStrFanMatch)
{
	int nFanMatch = vStrFanMatch.size();
	Mat tmat = Mat(nFanMatch, 3, CV_32F);
	for (int i = 0; i < nFanMatch; i++)
	{
		float* pdat = tmat.ptr<float>(i);
		pdat[0] = vStrFanMatch[i].fserial1;
		pdat[1] = vStrFanMatch[i].fserial2;
		pdat[2] = vStrFanMatch[i].dist;
	}	
	return tmat;
}

void CLineMatching::homoLines(Mat inLines, Mat &hoLines)
{
	int nlines = inLines.rows;
	for (int i = 0; i < nlines; i++)
	{
		float *pdat = inLines.ptr<float>(i);
		Mat mat1 = (Mat_<float>(1,3) << pdat[0], pdat[1], 1);
		Mat mat2 = (Mat_<float>(1,3) << pdat[2], pdat[3], 1);
		hoLines.push_back(mat1.cross(mat2));
	}		
	hoLines = hoLines.t();
}

void CLineMatching::intersectionOf2Lines_homo(Mat line1, Mat line2, Mat &intpt)
{
	intpt = line1.cross(line2);
	intpt /= intpt.at<float>(2);
	intpt = intpt.t();
}

void CLineMatching::calcCorrespondence(Mat pts, Mat lines, Mat fmat, Mat &corPts)
{
	Mat transLines = fmat * pts;
	int nlines = transLines.cols;
	for (int i = 0; i < nlines; i++)
	{
		Mat curLine1 = lines.col(i);
		Mat curLine2 = transLines.col(i);
		Mat intpt;
		intersectionOf2Lines_homo(curLine1, curLine2, intpt);		
		corPts.push_back(intpt);
	}
	corPts = corPts.t();
}

void CLineMatching::hmat_fmatOnePtAndOneLine(Mat fmat, Mat line1, Mat line2, Mat pt1, Mat pt2, Mat &hmat)
{
	Mat tline2 = line2.t();
	float *pdat = tline2.ptr<float>(0);
	Mat skew = (Mat_<float>(3, 3)<<0, -pdat[2], pdat[1], pdat[2], 0, -pdat[0], -pdat[1], pdat[0], 0);
	SVD svd(fmat);
	Mat er = svd.u.col(2);
	Mat crossx2e2 = pt2.cross(er);	
	float lengthCrossx2e2 = pow(norm(crossx2e2), 2);
	Mat Fx = fmat * pt1;
	Mat crossx2Fx = pt2.cross(Fx);
	Mat fcross = crossx2Fx.cross(line2);
	Mat ltx = line1.t() * pt1;
	float val_ltx = ltx.at<float>(0);
	Mat e2t = er * line1.t();
	Mat tmat = crossx2e2.t() * fcross;
	float val = tmat.at<float>(0);
	hmat = skew * fmat  + 1/lengthCrossx2e2 * val * e2t / val_ltx;    
	hmat = hmat / hmat.at<float>(2, 2);   
}

void CLineMatching::calcLocalHomography_2lineFMat(strFanSection strfan1, strFanSection strfan2, Mat fmat, Mat &hmat)
{	
	int ser1 = strfan1.strbranch1.lineSerial;
	int ser2 = strfan1.strbranch2.lineSerial;
	strLine curLine1 = strline1[ser1];
	strLine curLine2 = strline1[ser2];
	ser1 = strfan2.strbranch1.lineSerial;
	ser2 = strfan2.strbranch2.lineSerial;
	strLine curLine3 = strline2[ser1];
	strLine curLine4 = strline2[ser2];

	Mat bPts = (Mat_<float>(3, 4)<< curLine1.ps.x, curLine1.pe.x, curLine2.ps.x,  curLine2.pe.x,
													     curLine1.ps.y, curLine1.pe.y, curLine2.ps.y,  curLine2.pe.y,
														 1 , 1, 1, 1);
	Mat lines = (Mat_<float>(2, 4)<< curLine3.ps.x, curLine3.ps.y, curLine3.pe.x,  curLine3.pe.y,
													     curLine4.ps.x, curLine4.ps.y, curLine4.pe.x,  curLine4.pe.y);
	
	Mat hoLines;	
	homoLines(lines, hoLines);
	
	SVD svd(fmat);
	Mat er = svd.u.col(2);
	Mat ter = er.t();
	float *pdat = ter.ptr<float>(0);
	Mat skew = (Mat_<float>(3, 3)<<0, -pdat[2], pdat[1], pdat[2], 0, -pdat[0], -pdat[1], pdat[0], 0);	
	Mat A = skew * fmat;
	Mat a, b;
	for (int i = 0; i < 2; i++)
	{		
		Mat curLine = hoLines.col(i).t();
		for (int j = 0; j < 2; j++)
		{
			Mat curPt = bPts.col(i*2+j).t();
			Mat l2tA = curLine * A;
			float val1 = curPt.dot(l2tA);
			float val2 = curLine.t().dot(er);
			float tb = val1 / val2;
			b.push_back (tb);
			a.push_back(curPt);
		}
	}
	Mat ata = a.t() * a;
	Mat inv_ata = ata.inv();
	Mat atb = a.t() * b;
	Mat v = inv_ata * atb;	
	hmat = A - er * v.t();
	hmat = hmat / hmat.at<float>(2,2);
}

void CLineMatching::calcLocalHomography(strFanSection strfan1, strFanSection strfan2, Mat fmat, Mat &hmat)
{	
	Mat bPts1 = (Mat_<float>(3, 2)<<strfan1.strbranch1.bpt.x, strfan1.strbranch2.bpt.x,
														  strfan1.strbranch1.bpt.y, strfan1.strbranch2.bpt.y, 1 , 1);
	Mat lines1 = (Mat_<float>(2, 4)<<strfan1.intsection.x, strfan1.intsection.y,
														  strfan1.strbranch1.bpt.x, strfan1.strbranch1.bpt.y,
														  strfan1.intsection.x, strfan1.intsection.y,
														  strfan1.strbranch2.bpt.x, strfan1.strbranch2.bpt.y);
	Mat lines2 = (Mat_<float>(2, 4)<<strfan2.intsection.x, strfan2.intsection.y,
														  strfan2.strbranch1.bpt.x, strfan2.strbranch1.bpt.y,
														  strfan2.intsection.x, strfan2.intsection.y,
														  strfan2.strbranch2.bpt.x, strfan2.strbranch2.bpt.y);
	Mat hoLines1, hoLines2;
	homoLines(lines1, hoLines1);
	homoLines(lines2, hoLines2);
	Mat corPts;
	calcCorrespondence(bPts1, hoLines2, fmat, corPts);

	Mat hmat1, hmat2;
	hmat_fmatOnePtAndOneLine(fmat, hoLines1.col(1), hoLines2.col(1), bPts1.col(0), corPts.col(0), hmat1);
	Mat homopt = hmat1 * bPts1.col(1);
	homopt = homopt / homopt.at<float>(2);
	float dist1 = norm(homopt - corPts.col(1));       
	hmat_fmatOnePtAndOneLine(fmat, hoLines1.col(0), hoLines2.col(0), bPts1.col(1), corPts.col(1), hmat2);
	homopt = hmat2 * bPts1.col(0);
	homopt = homopt / homopt.at<float>(2);
	float dist2 = norm(homopt - corPts.col(0));
	hmat = dist1 > dist2 ? hmat2 : hmat1;	

	Mat tint = (Mat_<float>(3, 1)<<strfan1.intsection.x, strfan1.intsection.y, 1);
	Mat ttint = (Mat_<float>(3, 1)<<strfan2.intsection.x, strfan2.intsection.y, 1);
	Mat homoTint = hmat1 * tint;
	homoTint /=  homoTint.at<float>(2);
	float dist3 = norm(homoTint - ttint);

	homoTint = hmat2 * tint;
	homoTint /=  homoTint.at<float>(2);
	float dist4 = norm(homoTint - ttint);
}

void CLineMatching::matchSingleLines_homography(float homDistThr)
{
	int nEnterGroups = _nEnterGroups;
	float rotAngleThr = _rotAngThr;
	int nFanMatch = vstrLineMatch.size();	
	Mat serMatchedLines1, serMatchedLines2;	
	float minDesDist = FLT_MAX;

	vector<strLine> vMatchedLines1, vMatchedLines2, vUnmatchedLines1, vUnmatchedLines2;
	bifurcateLines(strline1, strline2, vstrLineMatch, vMatchedLines1, vMatchedLines2, vUnmatchedLines1, vUnmatchedLines2);

	Mat pointMatches;
	vpointMatch2Mat(vstrPointMatch, pointMatches);
	vector<strFanSection> vMatchedFans1, vMatchedFans2, vUnmatchedFans1, vUnmatchedFans2;
	bifurcateFans(vstrFanSection1, vstrFanSection2, vstrFanMatch, vMatchedFans1, vMatchedFans2, vUnmatchedFans1, vUnmatchedFans2);
	vector<vector<vector<int> > > vEnteredSingleLines1, vEnteredSingleLines2;
	groupSingleLines(vUnmatchedLines1, vMatchedFans1, nEnterGroups, vEnteredSingleLines1);
	groupSingleLines(vUnmatchedLines2, vMatchedFans2, nEnterGroups, vEnteredSingleLines2);
	vector<strFanMatch> vNewFanMatch;
	vector<strLineMatch> vNewLineMatch;
	vector<strPointMatch> vNewPtMatch;
	vector<strFanSection> vNewStrFan1, vNewStrFan2;
	vector<int> vFanSer;
	int ngroup = vEnteredSingleLines1.size();
	int cols1 = colorImg1.cols;
	int nfan1 = vstrFanSection1.size();
	int nfan2 = vstrFanSection2.size();

	int i = 0;
	for (; i < ngroup; i++)
	{		
		strFanSection curFan1, curFan2;
		curFan1 = vMatchedFans1[i];
		curFan2 = vMatchedFans2[i];
		float rotAng = estimateLocalRotateAngle(curFan1, curFan2);
		Mat hmat;		
		if (_isTwoLineHomog)
		{
			calcLocalHomography_2lineFMat(curFan1, curFan2, FMat, hmat);
			float tdist = distPairPt2HMat(curFan1.intsection, curFan2.intsection, hmat);
			if (tdist  > _hmatThr)
				continue;
		}
		else
		{
			calcLocalHomography(curFan1, curFan2, FMat, hmat);
		}		

		for (int j = 0; j < 4; j++)
		{
			vector<int> serLine1 = vEnteredSingleLines1[i].at(j);
			vector<int> serLine2 = vEnteredSingleLines2[i].at(j);
			int nserLine1 = serLine1.size();
			int nserLine2 = serLine2.size();
			if (nserLine1 == 0 || nserLine2 == 0)			
				continue;

			for (int m = 0; m < nserLine1; m++)
			{				
				bool flag = 0;
				int ser1 = serLine1[m];
				strLine curLine1 = vUnmatchedLines1[ser1];	
				Mat hopt1 = (Mat_<float>(3, 1)<<curLine1.ps.x, curLine1.ps.y, 1);
				Mat hopt2 = (Mat_<float>(3, 1)<<curLine1.pe.x, curLine1.pe.y, 1);
				Mat hoLine1 = hopt1.cross(hopt2);
				Mat corPt1 = hmat * hopt1;
				corPt1 /= corPt1.at<float>(2);
				Mat corPt2 = hmat * hopt2;
				corPt2 /= corPt2.at<float>(2);

				Mat symDist = Mat::ones(nserLine2, 1, CV_32F) * homDistThr;
				vector<int> vser;
				vector<strPointMatch> tvstrPtMatch(2*nserLine2);
				vector<strFanSection> tstrFan1(2*nserLine2), tstrFan2(2*nserLine2);				
				for (int n = 0; n < nserLine2; n++)
				{	
					int ser2 = serLine2[n];					
					strLine curLine2 = vUnmatchedLines2[ser2];
					Mat inv_hmat = hmat.inv();
					vser.push_back(curLine2.serial);
					float ang1 = curLine1.direction;
					float ang2 = curLine2.direction;

					if ( (ang1 < 0.5*CV_PI && ang2  > 1.5 * CV_PI && ang2-ang1<=1.5*CV_PI) ||  (ang1 > 1.5*CV_PI && ang2 < 0.5 * CV_PI && ang1-ang2 <= 1.5*CV_PI) )
					{
						Point2f tpt = curLine2.ps;
						curLine2.ps = curLine2.pe;
						curLine2.pe = tpt;
						curLine2.direction = fast_mod(curLine2.direction+CV_PI);
						ang2 = curLine2.direction;
						curLine2.intensityDir *= (-1);
					}
					float dang = ang2 - ang1;	
					if (dang >= 3/2 * CV_PI)
						dang -= 2*CV_PI;
					if (dang <= -3/2 * CV_PI)
						dang += 2 * CV_PI;

					float dAng = abs(dang - rotAng);
					if ( dAng > rotAngleThr)
						continue;

					if ( (curLine1.intenRatio > 1.3 || curLine2.intenRatio > 1.3) && curLine1.intensityDir != curLine2.intensityDir)
						continue;

					Mat hopt3 = (Mat_<float>(3, 1)<<curLine2.ps.x, curLine2.ps.y, 1);
					Mat hopt4 = (Mat_<float>(3, 1)<<curLine2.pe.x, curLine2.pe.y, 1);
					Mat hoLine2 = hopt3.cross(hopt4);
					Mat corPt3 = inv_hmat * hopt3;
					corPt3 /= corPt3.at<float>(2);
					Mat corPt4 = inv_hmat * hopt4;
					corPt4 /= corPt4.at<float>(2);

					float dist1, dist2;
					bool tb1 = isTwoLineOverlap(curLine2.ps, curLine2.pe, Point2f(corPt1.at<float>(0), corPt1.at<float>(1)), Point2f(corPt2.at<float>(0), corPt2.at<float>(1)), dist1, _regionHeight);
					bool tb2 = isTwoLineOverlap(curLine1.ps, curLine1.pe, Point2f(corPt3.at<float>(0), corPt3.at<float>(1)), Point2f(corPt4.at<float>(0), corPt4.at<float>(1)), dist2, _regionHeight);
					if (!tb1 || !tb2)
						continue;

					symDist.at<float>(n) = (dist1 + dist2) / 2;			
				}
				double minVal;
				Point minIdx;
				minMaxLoc(symDist, &minVal, NULL, &minIdx);

				if (minVal < homDistThr)
				{
					strLineMatch tstrLineMatch;					
					tstrLineMatch.serLine1 = curLine1.serial;
					tstrLineMatch.serLine2 = vser[minIdx.y];
					tstrLineMatch.dist = minVal;
					vNewLineMatch.push_back(tstrLineMatch);

					vFanSer.push_back(i);
				}
			}
		}
	}
	vector<int> vkept;
	if ( ! vNewLineMatch.size() )
	{
		return;
	}
	uniqueLineMatch(vNewLineMatch, vkept);	
	int num = vNewLineMatch.size();	
	vector<int> tvfanSer;
	for (int i = 0; i < num; i++)		
	{
		vstrLineMatch.push_back(vNewLineMatch[i]);		
		int curIdx = vkept[i];
		tvfanSer.push_back(vFanSer[curIdx]);
	}
	vFanSer = tvfanSer;

	for (int i = 0; i < num; i++)
	{
		int curSer = vFanSer[i];
		strFanSection curFan1 = vMatchedFans1[curSer];
		strFanSection curFan2 = vMatchedFans2[curSer];

		Mat hmat;
		if (_isTwoLineHomog)
		{
			calcLocalHomography_2lineFMat(curFan1, curFan2, FMat, hmat);
			float tdist = distPairPt2HMat(curFan1.intsection, curFan2.intsection, hmat);
			if (tdist  > _hmatThr)
				continue;
		}
		else
		{
			calcLocalHomography(curFan1, curFan2, FMat, hmat);
		}	

		int lineSer1 = vNewLineMatch[i].serLine1;
		int lineSer2 = vNewLineMatch[i].serLine2;
		strLine curLine1 = strline1[lineSer1];
		strLine curLine2 = strline2[lineSer2];

		float ang1 = curLine1.direction;
		float ang2 = curLine2.direction;
		if ( (ang1 < 0.5*CV_PI && ang2  > 1.5 * CV_PI && ang2-ang1<=1.5*CV_PI) ||  (ang1 > 1.5*CV_PI && ang2 < 0.5 * CV_PI && ang1-ang2 <= 1.5*CV_PI) )
		{
			Point2f tpt = curLine2.ps;
			curLine2.ps = curLine2.pe;
			curLine2.pe = tpt;			
			curLine1.direction = fast_mod(curLine2.direction + CV_PI);
		}

		Point2f intpt1 = intersectionOfLines(curLine1.ps, curLine1.pe, curFan1.intsection, curFan1.strbranch1.bpt);
		Point2f intpt2 = intersectionOfLines(curLine1.ps, curLine1.pe, curFan1.intsection, curFan1.strbranch2.bpt);
		Point2f intpt3 = intersectionOfLines(curLine2.ps, curLine2.pe, curFan2.intsection, curFan2.strbranch1.bpt);
		Point2f intpt4 = intersectionOfLines(curLine2.ps, curLine2.pe, curFan2.intsection, curFan2.strbranch2.bpt);					
		Mat mintpt1 = (Mat_<float>(3, 1)<<intpt1.x, intpt1.y, 1);
		Mat mintpt2 = (Mat_<float>(3, 1)<<intpt2.x, intpt2.y, 1);
		Mat mintpt3 = (Mat_<float>(3, 1)<<intpt3.x, intpt3.y, 1);
		Mat mintpt4 = (Mat_<float>(3, 1)<<intpt4.x, intpt4.y, 1);

		if( (! isConsistentWithFMat(intpt1, intpt3,  FMat,  1.5*_fmatThr) ) || (! isConsistentWithFMat(intpt2, intpt4,  FMat,  1.5*_fmatThr)) )
		{
			continue;
		}

		Mat inv_hmat = hmat.inv();
		Mat corMIntpt1 = hmat * mintpt1;
		corMIntpt1 = corMIntpt1 / corMIntpt1.at<float>(2);
		Mat corMIntpt2 = hmat * mintpt2;
		corMIntpt2 = corMIntpt2 / corMIntpt2.at<float>(2);
		Mat corMIntpt3 = inv_hmat * mintpt3;				  
		corMIntpt3 = corMIntpt3 / corMIntpt3.at<float>(2);
		Mat corMIntpt4 = inv_hmat * mintpt4;				  
		corMIntpt4 = corMIntpt4 / corMIntpt4.at<float>(2);

		float hdist1 = (norm(corMIntpt1-mintpt3) + norm(corMIntpt3-mintpt1)) / 2; 
		float hdist2 = (norm(corMIntpt2-mintpt4) + norm(corMIntpt4-mintpt2)) / 2; 

		if (hdist1>1.5*_hmatThr || hdist2>1.5*_hmatThr)
			continue;

		float dist1 = (intpt1.x- curFan1.intsection.x) * (intpt1.x- curFan1.intsection.x) + (intpt1.y- curFan1.intsection.y) * (intpt1.y- curFan1.intsection.y);
		float dist2 = (intpt3.x- curFan2.intsection.x) * (intpt3.x- curFan2.intsection.x) + (intpt3.y- curFan2.intsection.y) * (intpt3.y- curFan2.intsection.y);
		if (dist1 < 3 || dist2 < 3)
			continue;

		float difAng = fmod(abs(curLine1.direction - curFan1.strbranch2.ang), CV_PI);
		if (difAng < _fanThr || (CV_PI - difAng) < _fanThr)		
			continue;

		strFanSection tstrFan1, tstrFan2;
		if (hdist1 < 2*_hmatThr)
		{		
			strPointMatch tstrPtMatch;
			tstrPtMatch.point1 = intpt1;
			tstrPtMatch.point2 = intpt3;
			vstrPointMatch.push_back(tstrPtMatch);

			strBranch strbranch1, strbranch2;
			strbranch1.bpt = curLine1.pe;
			strbranch1.lineSerial= curLine1.serial;
			float dx = curLine1.pe.x - intpt1.x;
			float dy = curLine1.pe.y - intpt1.y;
			float ang1 = fast_mod(atan2(dy, dx)); 
			strbranch1.ang = ang1;
			strbranch2.bpt =	curFan1.intsection;						
			strbranch2.lineSerial = curFan1.strbranch1.lineSerial;
			float ang2 = fast_mod(curFan1.strbranch1.ang+CV_PI);
			strbranch2.ang = ang2;

			strBranch strbranch3, strbranch4;
			strbranch3.bpt = curLine2.pe;
			strbranch3.lineSerial = curLine2.serial;
			dx = curLine2.pe.x - intpt3.x;
			dy = curLine2.pe.y - intpt3.y;						
			strbranch3.ang = fast_mod(atan2(dy, dx));
			strbranch4.bpt = curFan2.intsection;
			strbranch4.lineSerial = curFan2.strbranch1.lineSerial;
			strbranch4.ang = fast_mod(curFan2.strbranch1.ang+CV_PI);

			if (ang1 > ang2)
			{
				strBranch  tstrbranch;
				tstrbranch = strbranch1;
				strbranch1 = strbranch2;
				strbranch2 = tstrbranch;

				tstrbranch = strbranch3;
				strbranch3 = strbranch4;
				strbranch4 = tstrbranch;
			}
			strFanSection tstrFan1, tstrFan2;
			tstrFan1.intsection = intpt1;
			tstrFan2.intsection = intpt3;
			tstrFan1.fanSerial = nfan1;
			tstrFan2.fanSerial = nfan2;			
			tstrFan1.strbranch1 = strbranch1;
			tstrFan1.strbranch2 = strbranch2;
			tstrFan2.strbranch1 = strbranch3;
			tstrFan2.strbranch2 = strbranch4;			
			vstrFanSection1.push_back(tstrFan1);
			vstrFanSection2.push_back(tstrFan2);
			strFanMatch tstrFanMatch;
			tstrFanMatch.fserial1 = nfan1;
			tstrFanMatch.fserial2 = nfan2;
			vstrFanMatch.push_back(tstrFanMatch);
			nfan1++; 
			nfan2++;				
		}

		difAng = fmod(abs(curLine1.direction - curFan1.strbranch2.ang), CV_PI);
		if (difAng < _fanThr || (CV_PI - difAng) < _fanThr)		
			continue;

		if (hdist2 <2*_hmatThr)
		{		
			strPointMatch tstrPtMatch;
			tstrPtMatch.point1 = intpt2;
			tstrPtMatch.point2 = intpt4;
			vstrPointMatch.push_back(tstrPtMatch);

			strBranch strbranch1, strbranch2;
			strbranch1.bpt = curLine1.pe;
			strbranch1.lineSerial= curLine1.serial;
			float dx = curLine1.pe.x - intpt2.x;
			float dy = curLine1.pe.y - intpt2.y;
			float ang1 = fast_mod(atan2(dy, dx)); 
			strbranch1.ang = ang1;
			strbranch2.bpt =	curFan1.intsection;						
			strbranch2.lineSerial = curFan1.strbranch2.lineSerial;
			float ang2 = fast_mod(curFan1.strbranch2.ang+CV_PI);
			strbranch2.ang = ang2;

			strBranch strbranch3, strbranch4;
			strbranch3.bpt = curLine2.pe;
			strbranch3.lineSerial = curLine2.serial;
			dx = curLine2.pe.x - intpt4.x;
			dy = curLine2.pe.y - intpt4.y;						
			strbranch3.ang = fast_mod(atan2(dy, dx));
			strbranch4.bpt = curFan2.intsection;
			strbranch4.lineSerial = curFan2.strbranch2.lineSerial;
			strbranch4.ang = fast_mod(curFan2.strbranch2.ang+CV_PI);

			if (ang1 > ang2)
			{
				strBranch  tstrbranch;
				tstrbranch = strbranch1;
				strbranch1 = strbranch2;
				strbranch2 = tstrbranch;

				tstrbranch = strbranch3;
				strbranch3 = strbranch4;
				strbranch4 = tstrbranch;
			}
			strFanSection tstrFan1, tstrFan2;
			tstrFan1.intsection = intpt2;
			tstrFan2.intsection = intpt4;
			tstrFan1.fanSerial = nfan1;
			tstrFan2.fanSerial = nfan2;			
			tstrFan1.strbranch1 = strbranch1;
			tstrFan1.strbranch2 = strbranch2;
			tstrFan2.strbranch1 = strbranch3;
			tstrFan2.strbranch2 = strbranch4;	
			vstrFanSection1.push_back(tstrFan1);
			vstrFanSection2.push_back(tstrFan2);
			strFanMatch tstrFanMatch;
			tstrFanMatch.fserial1 = nfan1;
			tstrFanMatch.fserial2 = nfan2;
			vstrFanMatch.push_back(tstrFanMatch);
			nfan1++; 
			nfan2++;	
		}

		uniquePointMatch(vstrPointMatch);	
	}
}

float CLineMatching::distPairPt2HMat( Point2f pt1, Point2f pt2, Mat hmat)
{
	Mat coPt1 = (Mat_<float>(3, 1) << pt1.x, pt1.y, 1);
	Mat coPt2 = (Mat_<float>(3, 1) << pt2.x, pt2.y, 1);
	Mat tranPt1 = hmat * coPt1;
	tranPt1 = tranPt1 / tranPt1.at<float>(2);
	Mat inv_hmat = hmat.inv();
	Mat tranPt2 = inv_hmat * coPt2;
	tranPt2 = tranPt2 / tranPt2.at<float>(2);
	float dist = norm(tranPt1-coPt2) + norm(tranPt2-coPt1);
	return dist/2;
}

bool CLineMatching::isTwoLineOverlap(Point2f pt1, Point2f pt2, Point2f pt3, Point2f pt4, float &dist, float radius)
{
	Point2f cenpt;
	cenpt.x =( pt1.x + pt2.x ) / 2;
	cenpt.y =( pt1.y + pt2.y ) / 2;
	float dy = pt2.y - pt1.y;
	float dx = pt2.x - pt1.x;
	float degAng = fastAtan2(dy, dx);
	float arcAng  = degAng / 180 * CV_PI;
	float dsin = sin(arcAng);
	float dcos = cos(arcAng);
	float length = abs(tan(arcAng)) > 1 ? abs(dy) : abs(dx);			
	float hafWidth = length / 2; //+ radius;
	float hafHeg    = radius;
	
	float fposx1 = dcos * (pt3.x - cenpt.x) + dsin *  (pt3.y - cenpt.y);	
	float fposy1 = dsin  * (pt3.x - cenpt.x) -  dcos * (pt3.y - cenpt.y);
	float fposx2 = dcos * (pt4.x - cenpt.x) + dsin *  (pt4.y - cenpt.y);	
	float fposy2 = dsin  * (pt4.x - cenpt.x) -  dcos * (pt4.y - cenpt.y);

	dist = (abs(fposy1) + abs(fposy2)) / 2;
	return isLineIntersectRectangle(fposx1, fposy1, fposx2, fposy2, -hafWidth, -hafHeg, hafWidth, hafHeg); 
}

bool CLineMatching::isLineIntersectRectangle(float linePointX1, float linePointY1, float linePointX2, float linePointY2, float rectangleLeftTopX, 
	float rectangleLeftTopY, float rectangleRightBottomX, float rectangleRightBottomY) 
{
	float  lineHeight = linePointY1 - linePointY2;
	float  lineWidth = linePointX2 - linePointX1;		
	float  c = linePointX1 * linePointY2 - linePointX2 * linePointY1;

	if ((lineHeight * rectangleLeftTopX + lineWidth * rectangleLeftTopY + c >= 0 && lineHeight * rectangleRightBottomX + lineWidth * rectangleRightBottomY + c <= 0)
		|| (lineHeight * rectangleLeftTopX + lineWidth * rectangleLeftTopY + c <= 0 && lineHeight * rectangleRightBottomX + lineWidth * rectangleRightBottomY + c >= 0)
		|| (lineHeight * rectangleLeftTopX + lineWidth * rectangleRightBottomY + c >= 0 && lineHeight * rectangleRightBottomX + lineWidth * rectangleLeftTopY + c <= 0)
		|| (lineHeight * rectangleLeftTopX + lineWidth * rectangleRightBottomY + c <= 0 && lineHeight * rectangleRightBottomX + lineWidth * rectangleLeftTopY + c >= 0)) {
			if (rectangleLeftTopX > rectangleRightBottomX) {
				float temp = rectangleLeftTopX;
				rectangleLeftTopX = rectangleRightBottomX;
				rectangleRightBottomX = temp;
			}
			if (rectangleLeftTopY < rectangleRightBottomY) {
				float temp = rectangleLeftTopY;
				rectangleLeftTopY = rectangleRightBottomY;
				rectangleRightBottomY = temp;
			}
			if ((linePointX1 < rectangleLeftTopX && linePointX2 < rectangleLeftTopX) 
				|| (linePointX1 > rectangleRightBottomX && linePointX2 > rectangleRightBottomX) 
				|| (linePointY1 > rectangleLeftTopY && linePointY2 > rectangleLeftTopY) 
				|| (linePointY1 < rectangleRightBottomY && linePointY2 < rectangleRightBottomY)) {
					return false;
			} else {
				return true;
			}
	} else {
		return false;
	}
}

void CLineMatching::matchSingleLines(float desDistThr, float fDistThr)
{
	int nEnterCluster = 4;	
	int nNearestMatchedPts = 8;
	int nEnterGroups = 8;
	float rotAngleThr = 20 * CV_PI / 180;
	float endPtDist = 7;
	int nFanMatch = vstrLineMatch.size();	
	Mat serMatchedLines1, serMatchedLines2;	
	float minDesDist = FLT_MAX;
	
	vector<strLine> vMatchedLines1, vMatchedLines2, vUnmatchedLines1, vUnmatchedLines2;
	bifurcateLines(strline1, strline2, vstrLineMatch, vMatchedLines1, vMatchedLines2, vUnmatchedLines1, vUnmatchedLines2);

	Mat pointMatches;
	vpointMatch2Mat(vstrPointMatch, pointMatches);
	Mat nearestPts1, nearestPts2;
	nearestMatchedPointsToLine(vUnmatchedLines1, pointMatches.colRange(0, 2), 8, nearestPts1);
	nearestMatchedPointsToLine(vUnmatchedLines2, pointMatches.colRange(2, 4), 8, nearestPts2);

	vector<strFanSection> vMatchedFans1, vMatchedFans2, vUnmatchedFans1, vUnmatchedFans2;
	bifurcateFans(vstrFanSection1, vstrFanSection2, vstrFanMatch, vMatchedFans1, vMatchedFans2, vUnmatchedFans1, vUnmatchedFans2);
	vector<vector<vector<int> > > vEnteredSingleLines1, vEnteredSingleLines2;
	groupSingleLines(vUnmatchedLines1, vMatchedFans1, nEnterGroups, vEnteredSingleLines1);
	groupSingleLines(vUnmatchedLines2, vMatchedFans2, nEnterGroups, vEnteredSingleLines2);
	vector<strFanMatch> vNewFanMatch;
	vector<strLineMatch> vNewLineMatch;
	vector<strFanSection> vStrFan1, vStrFan2;
	int ngroup = vEnteredSingleLines1.size();
	for (int i = 0; i < ngroup; i++)
	{		
		strFanSection curFan1, curFan2;
		curFan1 = vMatchedFans1[i];
		curFan2 = vMatchedFans2[i];
		float rotAng = estimateLocalRotateAngle(curFan1, curFan2);

		for (int j = 0; j < 4; j++)
		{
			vector<int> serLine1 = vEnteredSingleLines1[i].at(j);
			vector<int> serLine2 = vEnteredSingleLines2[i].at(j);
			int nserLine1 = serLine1.size();
			int nserLine2 = serLine2.size();
			if (nserLine1 == 0 || nserLine2 == 0)			
				continue;

			for (int m = 0; m < nserLine1; m++)
			{				
				bool flag = 0;
				int ser1 = serLine1[m];
				strLine curLine1 = vUnmatchedLines1[ser1];						
				Mat mDesDist;
				vector<int> vtSer;		
				vector<strFanSection> tvStrFan1, tvStrFan2;
				for (int n = 0; n < nserLine2; n++)
				{				
					int ser2 = serLine2[n];					
					strLine curLine2 = vUnmatchedLines2[ser2];
					float dAng = abs(curLine2.direction - curLine1.direction - rotAng);
					if ( dAng > rotAngleThr && 2*CV_PI - dAng > rotAngleThr)
						continue;

					if(dAng > 3/2 * CV_PI)
						curLine2.intensityDir *= (-1);

					if ( (curLine1.intenRatio > 1.2 || curLine2.intenRatio > 1.2) && curLine1.intensityDir != curLine2.intensityDir)
						continue;

					float ang1 = curLine1.direction;
					float ang2 = curLine2.direction;
					float offset = abs(ang1 - ang2);
					if (offset > 1.5*CV_PI)
					{
						Point2f tpt = curLine2.ps;
						curLine2.ps = curLine2.pe;
						curLine2.pe = tpt;
						curLine2.direction = fast_mod(curLine2.direction+CV_PI);
					}
					if ( (! isConsistentWithFMat(curLine1.pe, curLine2.pe, FMat, endPtDist)) && 
						(! isConsistentWithFMat(curLine1.ps, curLine2.ps, FMat, endPtDist)) )
						continue;

					Mat mNPts1 = nearestPts1.row(ser1); 					
					Mat mNPts2 = nearestPts2.row(ser2); 
					if ( ! topoFilter_singleLine(curLine1, curLine2, mNPts1, mNPts2, pointMatches))
						continue;

					strFanSection tstrFan1, tstrFan2;
					float desDist;
					formAndMatchNewFans(curLine1,  curLine2, curFan1, curFan2, pointMatches, FMat, tstrFan1, tstrFan2, desDist, desDistThr, fDistThr);
					if (desDist == FLT_MAX)					
						continue;

					mDesDist.push_back(desDist);
					vtSer.push_back(curLine2.serial);
					tvStrFan1.push_back(tstrFan1);
					tvStrFan2.push_back(tstrFan2);
				}
				if (mDesDist.empty())
					continue;

				Mat sortedIdx, sortedDist;
				sortIdx(mDesDist, sortedIdx, CV_SORT_EVERY_COLUMN);
				cv::sort(mDesDist, sortedDist, CV_SORT_EVERY_COLUMN);

				strLineMatch tstrLineMatch;					
				tstrLineMatch.serLine1 = curLine1.serial;
				int tser = sortedIdx.at<int>(0, 0);
				tstrLineMatch.serLine2 = vtSer[tser];
				tstrLineMatch.dist = sortedDist.at<float>(0, 0);
				vNewLineMatch.push_back(tstrLineMatch);

				vStrFan1.push_back(tvStrFan1[tser]);
				vStrFan2.push_back(tvStrFan2[tser]);
			}
		}
	}
	vector<int> vkept;
	if ( ! vNewLineMatch.size() )
	{
		return;
	}
	uniqueLineMatch(vNewLineMatch, vkept);
	int num = vkept.size();
	int nFan1 = vstrFanSection1.size();
	int nFan2 = vstrFanSection2.size();
	strFanMatch tstrFanMatch;	
	for (int i = 0; i < num; i++)
	{	
		int tser = vkept[i];
		vStrFan1[tser].fanSerial = nFan1; 
		vStrFan2[tser].fanSerial = nFan2; 
		vstrFanSection1.push_back(vStrFan1[tser]);
		vstrFanSection2.push_back(vStrFan2[tser]);
		tstrFanMatch.fserial1 = nFan1;
		tstrFanMatch.fserial2 = nFan2;
		tstrFanMatch.dist  =  vNewLineMatch[i].dist;
		vstrFanMatch.push_back(tstrFanMatch);
		vstrLineMatch.push_back(vNewLineMatch[i]);

		nFan1++;
		nFan2++;
	}
}

void CLineMatching::bifurcateLines(strLine* pStrLine1, strLine* pStrLine2, vector<strLineMatch> vStrLineMatch, vector<strLine> &vMatchedLines1, 
													   vector<strLine> &vMatchedLines2, vector<strLine> &vUnmatchedLines1, vector<strLine> &vUnmatchedLines2)
{
	int nLineMatch = vStrLineMatch.size();
	Mat matchSerial1, matchSerial2;
	for (int i = 0; i < nLineMatch; i++)
	{
		int ser1 = vStrLineMatch[i].serLine1;
		int ser2 = vStrLineMatch[i].serLine2;
		vMatchedLines1.push_back(pStrLine1[ser1]);
		vMatchedLines2.push_back(pStrLine2[ser2]);
		matchSerial1.push_back(ser1);
		matchSerial2.push_back(ser2);
	}
	matchSerial1 = matchSerial1.t();
	matchSerial2 = matchSerial2.t();
	CMatOperation *pMatOperation = new CMatOperation;
	Mat fullSer1 = genContinuousMat(0, nline1);
	Mat fullSer2 = genContinuousMat(0, nline2);
	Mat unmatchedSer1, unmatchedSer2;
	pMatOperation->diffSet(fullSer1, matchSerial1, unmatchedSer1);	
	pMatOperation->diffSet(fullSer2, matchSerial2, unmatchedSer2);	

	int cols1 = unmatchedSer1.cols;	
	int* pdat1 = unmatchedSer1.ptr<int>(0);
	for (int i = 0; i < cols1; i++)
	{
		int tser = pdat1[i];				
		vUnmatchedLines1.push_back((pStrLine1[tser]));		
	}	
	int cols2 = unmatchedSer2.cols;	
	int* pdat2 = unmatchedSer2.ptr<int>(0);
	for (int i = 0; i < cols2; i++)
	{		
		int ser = pdat2[i];		
		vUnmatchedLines2.push_back(pStrLine2[ser]);
	}

	delete pMatOperation;
	pMatOperation = NULL;
}

void CLineMatching::formAndMatchNewFans(strLine line1,  strLine line2, strFanSection fan1, strFanSection fan2, Mat pointMatches, 
																		Mat FMat, strFanSection &strFan1, strFanSection &strFan2, float &desDist, float desDistThr, float fDistThr)
{
	strFanSection tstrFan1, tstrFan2;
	Vec4f branch1 = Vec4f(fan1.intsection.x, fan1.intsection.y, fan1.strbranch1.bpt.x, fan1.strbranch1.bpt.y);
	Vec4f branch2 = Vec4f(fan2.intsection.x, fan2.intsection.y, fan2.strbranch1.bpt.x, fan2.strbranch1.bpt.y);	
	float desDist1;
	intersectWithOnePairBranches(line1,  line2, branch1, branch2, pointMatches, FMat, tstrFan1, tstrFan2, desDist1, desDistThr, fDistThr);					
	strFanSection tstrFan3, tstrFan4;
	float desDist2;
	branch1 = Vec4f(fan1.intsection.x, fan1.intsection.y, fan1.strbranch2.bpt.x, fan1.strbranch2.bpt.y);
	branch2 = Vec4f(fan2.intsection.x, fan2.intsection.y, fan2.strbranch2.bpt.x, fan2.strbranch2.bpt.y);
	intersectWithOnePairBranches(line1, line2, branch1, branch2, pointMatches, FMat, tstrFan3, tstrFan4, desDist2, desDistThr, fDistThr);	
	if (desDist1 == FLT_MAX && desDist2 == FLT_MAX )
	{
		desDist = FLT_MAX;
		return;
	}
	if (desDist2 > desDist1)
	{
		desDist = desDist1;
		tstrFan1.strbranch1.lineSerial = fan1.strbranch1.lineSerial;
		tstrFan1.strbranch2.lineSerial = line1.serial;
		tstrFan2.strbranch1.lineSerial = fan2.strbranch1.lineSerial;
		tstrFan2.strbranch2.lineSerial = line2.serial;
		strFan1 = tstrFan1;
		strFan2 = tstrFan2;
	}
	else
	{
		desDist = desDist2;
		tstrFan3.strbranch1.lineSerial = fan1.strbranch2.lineSerial;
		tstrFan3.strbranch2.lineSerial = line1.serial;
		tstrFan4.strbranch1.lineSerial = fan2.strbranch2.lineSerial;
		tstrFan4.strbranch2.lineSerial = line2.serial;

		strFan1 = tstrFan3;
		strFan2 = tstrFan4;
	}
}

void CLineMatching::intersectWithOnePairBranches(strLine line1,  strLine line2, Vec4f branch1, Vec4f branch2, Mat pointMatches, 
																				Mat FMat, strFanSection &strFan1, strFanSection &strFan2, float &desDist, float desDistThr, float fDistThr)
{
	Point2f junction1 = Point2f(branch1(0), branch1(1));
	Point2f junction2 = Point2f(branch2(0), branch2(1));
	Point2f tpt1 = Point2f(branch1(2), branch1(3));;
	Point2f tpt2 = Point2f(branch2(2), branch2(3));;
	Point2f endpt1		= line1.ps;
	Point2f endpt2		= line2.ps;
	Point2f intersection1 = intersectionOfLines(line1.ps, line1.pe, junction1, tpt1);	
	Point2f intersection2 = intersectionOfLines(line2.ps, line2.pe, junction2, tpt2);

	strFanSection tstrFan1, tstrFan2, tstrFan3, tstrFan4;
	float desDist1,  desDist2;
	if (   intersection1.x < 0 || intersection1.x > colorImg1.cols || intersection1.y < 0 || intersection1.y > colorImg1.rows
		|| intersection2.x < 0 || intersection2.x > colorImg2.cols || intersection2.y < 0 || intersection2.y > colorImg2.rows )
	{		
		desDist = FLT_MAX;		
		return;
	}
	formNewFanPair(intersection1, intersection2, junction1, junction2, endpt1, endpt2, tstrFan1, tstrFan2);	
	desDist1 = matchNewlyFormedFanPair(tstrFan1, tstrFan2, pointMatches, FMat, desDistThr, fDistThr);
		
	endpt1 = line1.pe;
	endpt2 = line2.pe;		
	formNewFanPair( intersection1, intersection2, junction1, junction2, endpt1, endpt2, tstrFan3, tstrFan4);		
	desDist2 =  matchNewlyFormedFanPair(tstrFan3, tstrFan4, pointMatches, FMat, desDistThr, fDistThr);

	if (desDist2 > desDist1)
	{
		desDist = desDist1;
		strFan1 = tstrFan1;
		strFan2 = tstrFan2;
	}
	else
	{
		desDist = desDist2;
		strFan1 = tstrFan3;
		strFan2 = tstrFan4;
	}
}

void CLineMatching::formNewFanPair(Point2f intersection1, Point2f intersection2, Point2f junction1, Point2f junction2, Point2f endpt1, 
	Point2f endpt2, strFanSection &tstrFan1, strFanSection &tstrFan2)
{
	strBranch tstrBranch1, tstrBranch2;
	tstrBranch1.bpt = junction1;
	float ang = fast_mod(atan2(junction1.y-intersection1.y, junction1.x-intersection1.x));
	tstrBranch1.ang = ang;
	tstrBranch2.bpt = endpt1;
	ang = fast_mod(atan2(endpt1.y-intersection1.y, endpt1.x-intersection1.x));
	tstrBranch2.ang = ang;	
	tstrFan1.intsection   = intersection1;
	tstrFan1.strbranch1 = tstrBranch1;
	tstrFan1.strbranch2 = tstrBranch2;

	tstrBranch1.bpt = junction2;
	ang = fast_mod(atan2(junction2.y-intersection2.y, junction2.x-intersection2.x));
	tstrBranch1.ang = ang;
	tstrBranch2.bpt = endpt2;
	ang = fast_mod(atan2(endpt2.y-intersection2.y, endpt2.x-intersection2.x));
	tstrBranch2.ang = ang;	
	tstrFan2.intsection   = intersection2;
	tstrFan2.strbranch1 = tstrBranch1;
	tstrFan2.strbranch2 = tstrBranch2;
}

float CLineMatching::matchNewlyFormedFanPair(strFanSection tstrFan1, strFanSection  tstrFan2, Mat pointMatches, Mat FMat, float desDistThr, float fDistThr)
{
	if ( ! isConsistentWithFMat(tstrFan1.intsection, tstrFan2.intsection, FMat, fDistThr) )
		return FLT_MAX;

	if (! isConsistentWithNeighborPointMatches(tstrFan1, tstrFan2, pointMatches))
		return FLT_MAX;

	strFanMatch tstrFanMatch;
	bool flag = 0;		
	Mat des1, des2;
	vector<strFanSection> tv1, tv2;
	tv1.push_back(tstrFan1);
	tv2.push_back(tstrFan2);
	description(tv1, gMag1, gDir1);
	description(tv2, gMag2, gDir2);
	des1 = tv1[0].mDes;
	des2 = tv2[0].mDes;
	float desDist = norm((des1 - des2), NORM_L2);
	if (desDist > desDistThr)		
		return FLT_MAX;
	
	return desDist;
}


Point2f CLineMatching::intersectionOfLines(Point2f pt1, Point2f pt2, Point2f pt3, Point2f pt4)
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

	float D   =  determinant(tmat1);
	float X   = determinant(tmat2) / D;
	float Y   = determinant(tmat3) / D;

	return  Point2f(X, Y);
}

bool CLineMatching::topoFilter_singleLine(strLine curLine1, strLine curLine2, Mat mNPts1, Mat mNPts2, Mat pointMatches)
{
	int nCandNum = 8;
	CMatOperation *pMatOperation = new CMatOperation;
	Mat interSet, keptIdx;	
	pMatOperation->intersectSet(mNPts1, mNPts2, interSet, keptIdx);
	if (((float)(interSet.rows))/nCandNum < 0.5)
		return false;

	float ang1 = curLine1.direction;
	float ang2 = curLine2.direction;
	float offset = abs(ang1 - ang2);
	if (offset > 1.5*CV_PI)
	{
		Point2f tpt = curLine2.ps;
		curLine2.ps = curLine2.pe;
		curLine2.pe = tpt;
		curLine2.direction = fast_mod(curLine2.direction+CV_PI);
	}

	Mat uniSet;
	pMatOperation->unionSet(mNPts1, mNPts2, uniSet);
	int nuni = uniSet.cols;
	int dircons = 0;
	int *pUnisetDat = uniSet.ptr<int>(0); 
	for (int k = 0; k < nuni; k++)
	{
		float *pCurPoint   = pointMatches.ptr<float>(pUnisetDat[k]);
		Vec2f vect1		    = Vec2f(curLine1.pe.x - curLine1.ps.x, curLine1.pe.y - curLine1.ps.y);
		Vec2f vect2          = Vec2f(pCurPoint[0] - curLine1.ps.x,  pCurPoint[1] - curLine1.ps.y);
		float sign1	        = vect1(0)*vect2(1) - vect1(1)*vect2(0);
		Mat_<float> tline1 = (Mat_<float>(1,3) <<curLine1.ps.x, curLine1.ps.y, curLine1.direction);
		float tdist1     = distPt2Line(Point2f(pCurPoint[0], pCurPoint[1]), tline1);
		vect1(0) = curLine2.pe.x - curLine2.ps.x;
		vect1(1) = curLine2.pe.y - curLine2.ps.y;
		vect2(0) = pCurPoint[2]  - curLine2.ps.x;
		vect2(1) =  pCurPoint[3] - curLine2.ps.y;							
		float sign2	  = vect1(0)*vect2(1) - vect1(1)*vect2(0);
		Mat_<float> tline2 = (Mat_<float>(1,3) <<curLine2.ps.x, curLine2.ps.y, curLine2.direction);
		float tdist2	  = distPt2Line(Point2f(pCurPoint[2], pCurPoint[3]), tline2);
		int signcom	  = sign(sign1*sign2);
		if ( signcom == 1 ||  (tdist1<3 && tdist2<3))
			dircons = dircons + 1;
	}
	if (dircons / nuni < 0.7)
		return false;

	return true;
}

float CLineMatching::estimateLocalRotateAngle(strFanSection strFan1, strFanSection strFan2)
{
	float ang1 = strFan1.strbranch1.ang;
	float ang2 = strFan1.strbranch2.ang;
	float ang3 = strFan2.strbranch1.ang;
	float ang4 = strFan2.strbranch2.ang;
	float dang1 = ang3 - ang1;	
	if (dang1 >= 3/2 * CV_PI)
		dang1 -= 2*CV_PI;
	if (dang1 <= -3/2 * CV_PI)
		dang1 += 2 * CV_PI;
	
	float dang2 = ang4 - ang2;
	if (dang2 >= 3/2 * CV_PI)
		dang2 -= 2*CV_PI;
	if (dang2 <= -3/2 * CV_PI)
		dang2 += 2 * CV_PI;

	return (dang1 + dang2) / 2;	
}

void CLineMatching::groupSingleLines(vector<strLine> vUnmatchedLines, vector<strFanSection> vMatchedFans, 
	int nEnterGroups, vector<vector<vector<int> > > &vEnteredSingleLines)
{
	Mat matchedJunctions, nearestJunctions;			
	int nMatchedFans = vMatchedFans.size();
	matchedJunctions.create(nMatchedFans, 2, CV_32F);	 
	for (int i = 0; i < nMatchedFans; i++)
	{
		float* pdat = matchedJunctions.ptr<float>(i);
		pdat[0] = vMatchedFans[i].intsection.x;
		pdat[1] = vMatchedFans[i].intsection.y;
	}
	nearestMatchedPointsToLine(vUnmatchedLines, matchedJunctions, nEnterGroups, nearestJunctions);
	int nunLines = vUnmatchedLines.size();
	vector<vector<int> > vGroup(nMatchedFans);

	for (int i = 0; i < nunLines; i++)
	{
		int* pNearestJunc =  nearestJunctions.ptr<int>(i);
		for (int j = 0; j < nEnterGroups; j++)
		{
			int ser = pNearestJunc[j];
			vGroup[ser].push_back(i);
		}
	}	
	vEnteredSingleLines.resize(nMatchedFans);
	for (int i = 0; i < nMatchedFans; i++)
	{
		vEnteredSingleLines[i].resize(4);
	}

	for (int i = 0; i < nMatchedFans; i++)
	{		
		vector<int> curGroupLines = vGroup[i];
		int ncurGLines = curGroupLines.size();
		if (!ncurGLines)             
			continue;		

		strFanSection curFan = vMatchedFans[i];
		Mat_<float> pline1 = (Mat_<float>(1, 3)<<curFan.intsection.x, curFan.intsection.y, curFan.strbranch1.ang);
		float refDirection1 = curFan.strbranch2.ang;
		Mat_<float> pline2 = pline1.clone();
		pline2(0, 2) = curFan.strbranch2.ang;
		float refDirection2 = curFan.strbranch1.ang;

		for (int m = 0; m < ncurGLines; m++)
		{
			int serCurline = curGroupLines[m];
			strLine	curLine = vUnmatchedLines[serCurline];
			int isSameSide1 = determinePtLinePos(curLine.ps, pline1, refDirection1);
			int isSameSide2 = determinePtLinePos(curLine.pe, pline1, refDirection1); 
			int isSameSide3 = determinePtLinePos(curLine.ps, pline2, refDirection2);                
			int isSameSide4 = determinePtLinePos(curLine.pe, pline2, refDirection2);
			if ( (isSameSide1 >= 0 && isSameSide3 >= 0) || (isSameSide2 >= 0 && isSameSide4 >= 0) ) 
				vEnteredSingleLines[i].at(0).push_back(serCurline);

			if ( (isSameSide1 >= 0 && isSameSide3 <= 0) || (isSameSide2 >= 0 && isSameSide4 <= 0) )
				vEnteredSingleLines[i].at(1).push_back(serCurline);

			if ( (isSameSide1 <= 0 && isSameSide3 >= 0) || (isSameSide2 <= 0 && isSameSide4 >= 0) )
				vEnteredSingleLines[i].at(2).push_back(serCurline);

			if ( (isSameSide1 <= 0 && isSameSide3 <= 0) || (isSameSide2 <= 0 && isSameSide4 <= 0) )
				vEnteredSingleLines[i].at(3).push_back(serCurline);
		}	
	}
}

void CLineMatching::nearestMatchedPointsToLine(vector<strLine> vUnmatchedLines, Mat matchedPoints, int nNeighborMPts, Mat &nearestPts)
{
	int nunLine = vUnmatchedLines.size();
	nearestPts = Mat(nunLine, nNeighborMPts, CV_32S);	
	for (int i = 0; i < nunLine; i++)
	{		
		int *pNearestPt = nearestPts.ptr<int>(i);				
		strLine curLine = vUnmatchedLines[i];
		Point2f midpt;
		midpt.x = (curLine.pe.x + curLine.ps.x) / 2;
		midpt.y = (curLine.pe.y + curLine.ps.y) / 2;
		float	length = 10;		
		Mat dist1 = matchedPoints.col(0).t() - midpt.x;
		Mat dist2 = matchedPoints.col(1).t() - midpt.y;
		Mat squeDist1, squeDist2;
		pow(dist1, 2., squeDist1);
		pow(dist2, 2., squeDist2);
		Mat dist = squeDist1 + squeDist2;
		Mat sortedDist, sortedIdx;
		cv::sort(dist, sortedDist, SORT_EVERY_ROW);
		sortIdx(dist, sortedIdx, SORT_EVERY_ROW);		
		Mat tmat;		
		int nnonzero = 0;

		do 
		{
			compare(sortedDist, length, tmat, CMP_LE);	
			nnonzero = countNonZero(tmat);
			length *= 2;
		} while (nnonzero < nNeighborMPts);
		
		Mat serNearPt = sortedIdx.colRange(0, nnonzero);
		int *pdat = serNearPt.ptr<int>(0);				
		Mat dists2line;
		for (int j = 0; j < nnonzero; j++)
		{
			int curSer = pdat[j];
			float* curPt = matchedPoints.ptr<float>(curSer);
			Mat_<float> tline = (Mat_<float>(1, 3) <<curLine.ps.x, curLine.ps.y, curLine.direction);
			float tdist = distPt2Line(Point2f(curPt[0], curPt[1]),  tline);
			dists2line.push_back(tdist);
		}
		Mat sortedIdx1;	 
		sortIdx(dists2line.t(), sortedIdx1, CV_SORT_EVERY_ROW);	
		int* pSortedIdx = sortedIdx1.ptr<int>(0);
		for (int j = 0; j < nNeighborMPts; j++)
		{
			int curSer = pSortedIdx[j];
			int oriSer = serNearPt.at<int>(0, curSer);			
			pNearestPt[j] = oriSer;
		}		
	}
}

void CLineMatching::bifurcateFans(vector<strFanSection> vStrFan1, vector<strFanSection> vStrFan2, vector<strFanMatch> vStrFanMatch, 
	vector<strFanSection> &vMatchedFans1, vector<strFanSection> &vMatchedFans2, vector<strFanSection> &vUnmatchedFans1, vector<strFanSection> &vUnmatchedFans2)
{
	int nfanMatch = vStrFanMatch.size();
	Mat matchSerial1, matchSerial2;
	for (int i = 0; i < nfanMatch; i++)
	{
		int ser1 = vstrFanMatch[i].fserial1;
		int ser2 = vstrFanMatch[i].fserial2;
		vMatchedFans1.push_back(vStrFan1[ser1]);
		vMatchedFans2.push_back(vStrFan2[ser2]);
		matchSerial1.push_back(ser1);
		matchSerial2.push_back(ser2);
	}
	matchSerial1 = matchSerial1.t();
	matchSerial2 = matchSerial2.t();
	CMatOperation *pMatOperation = new CMatOperation;
	Mat fullSer1 = genContinuousMat(0, vstrFanSection1.size());
	Mat fullSer2 = genContinuousMat(0, vstrFanSection2.size());
	Mat unmatchedSer1, unmatchedSer2;

	pMatOperation->diffSet(fullSer1, matchSerial1, unmatchedSer1);	
	pMatOperation->diffSet(fullSer2, matchSerial2, unmatchedSer2);


	int cols1 = unmatchedSer1.cols;	
	int* pdat1 = unmatchedSer1.ptr<int>(0);
	for (int i = 0; i < cols1; i++)
	{
		int tser = pdat1[i];				
		vUnmatchedFans1.push_back((vstrFanSection1[tser]));		
	}	
	int cols2 = unmatchedSer2.cols;	
	int* pdat2 = unmatchedSer2.ptr<int>(0);
	for (int i = 0; i < cols2; i++)
	{		
		int ser = pdat2[i];		
		vUnmatchedFans2.push_back(vstrFanSection2[ser]);
	}

	delete pMatOperation;
	pMatOperation = NULL;
}


void CLineMatching::updatePointMatchFromSingleLineMatch(vector<strFanMatch> vStrFanMatch, vector<strLineMatch> vStrLineMatch, const Mat FMat, vector<strPointMatch> &vPointMatches)
{	
	vPointMatches.clear();

	int nFan = vStrFanMatch.size();
	strPointMatch tstrPointMatch;
	for (int i = 0; i < nFan; i++)
	{
		//tstrPointMatch.dist = vStrFanMatch[i].dist;
		tstrPointMatch.point1 = vstrFanSection1[vStrFanMatch[i].fserial1].intsection;
		tstrPointMatch.point2 = vstrFanSection2[vStrFanMatch[i].fserial2].intsection;
	//	cout<<vStrFanMatch[i].fserial1<<","<<vStrFanMatch[i].fserial2<<endl;
		vPointMatches.push_back(tstrPointMatch);
	}

	float desDistThr = 0.4;
	float FDistThr = 3;
	adjustLineMatchEndpoints(vStrLineMatch);
	int nLineMatch = vStrLineMatch.size();
	for (int i = 0; i < nLineMatch; i++)
	{
		int fserial1 = vStrLineMatch[i].serLine1;	
		Vec3f pt1;
		pt1[0] = strline1[fserial1].ps.x;
		pt1[1] = strline1[fserial1].ps.y;
		pt1[2] = strline1[fserial1].direction;
		
		int fserial2 = vStrLineMatch[i].serLine2;
		Vec3f pt2;
		pt2[0] = strline2[fserial2].ps.x;
		pt2[1] = strline2[fserial2].ps.y;
		pt2[2] = strline2[fserial2].direction;

		float dist;
		bool tbool = isAcceptedToBePointMatch(pt1, pt2, desDistThr, FDistThr, dist);
		if (tbool)
		{			
			tstrPointMatch.point1.x = pt1[0];
			tstrPointMatch.point1.y = pt1[1];
			tstrPointMatch.point2.x = pt2[0];
			tstrPointMatch.point2.y = pt2[1];
//			tstrPointMatch.dist = dist;
			vPointMatches.push_back(tstrPointMatch);
		}

		pt1[0] = strline1[fserial1].pe.x;
		pt1[1] = strline1[fserial1].pe.y;
		pt1[2] = strline1[fserial1].direction;		
		pt2[0] = strline2[fserial2].pe.x;
		pt2[1] = strline2[fserial2].pe.y;
		pt2[2] = strline2[fserial2].direction;		
		tbool = isAcceptedToBePointMatch(pt1, pt2, desDistThr, FDistThr, dist);
		if (tbool)
		{
			strPointMatch tstrPointMatch;
			tstrPointMatch.point1.x = pt1[0];
			tstrPointMatch.point1.y = pt1[1];
			tstrPointMatch.point2.x = pt2[0];
			tstrPointMatch.point2.y = pt2[1];
//			tstrPointMatch.dist = dist;
			vPointMatches.push_back(tstrPointMatch);
		}
	}		
	uniquePointMatch(vPointMatches);
}

void CLineMatching::adjustLineMatchEndpoints(vector<strLineMatch> vStrLineMatch)
{
	int nlineMatch = vStrLineMatch.size();
	for (int i = 0; i < nlineMatch; i++)
	{
		strLineMatch curLineMatch = vStrLineMatch[i];
		int ser1 = curLineMatch.serLine1;
		int ser2 = curLineMatch.serLine2;
		float ang1 = strline1[ser1].direction;
		float ang2 = strline2[ser2].direction;
		float offset = abs(ang1 - ang2);
		if (offset > 1.5*CV_PI)
		{
			Point2f tpt = strline2[ser2].ps;
			strline2[ser2].ps = strline2[ser2].pe;
			strline2[ser2].pe = tpt;
		}
	}
}

void CLineMatching::nearestMatchedPtsToFan(vector<strFanSection> vFans, Mat matchedPts, int nNeighborMPts, Mat &nearestMPts)
{
	int nfans = vFans.size();
	int nCandNum = min(matchedPts.rows, nNeighborMPts);
	for (int i = 0; i < nfans; i++)
	{
		strFanSection curFan = vFans[i];		
		Point2f cenPt1 = curFan.intsection;	
		Mat xdiff= matchedPts.col(0) - cenPt1.x;	
		Mat xdiff2, ydiff2;
		pow(xdiff, 2., xdiff2);		
		Mat ydiff= matchedPts.col(1) - cenPt1.y;		
		pow(ydiff, 2., ydiff2);
		Mat dist = xdiff2 + ydiff2;		
		Mat sortedIdx;
		sortIdx(dist, sortedIdx, SORT_EVERY_COLUMN);		
		Mat candIdx = sortedIdx.rowRange(0, nCandNum).t();		
		nearestMPts.push_back(candIdx);
	}	
}

bool CLineMatching::isConsistentWithLocalRotateAngle(strFanSection fan1, strFanSection fan2, float rotAngle)
{
	float angleConsistentThr = _rotAngThr;
	float ang1 = fan1.strbranch1.ang;
	float ang2 = fan1.strbranch2.ang;
	float ang3 = fan2.strbranch1.ang;
	float ang4 = fan2.strbranch2.ang;
	float dang1 = ang3 - ang1;	
	if (dang1 >= 3/2 * CV_PI)
		dang1 -= 2*CV_PI;
	if (dang1 <= -3/2 * CV_PI)
		dang1 += 2 * CV_PI;

	float dang2 = ang4 - ang2;
	if (dang2 >= 3/2 * CV_PI)
		dang2 -= 2*CV_PI;
	if (dang2 <= -3/2 * CV_PI)
		dang2 += 2 * CV_PI;

	if (abs((dang1 + dang2) / 2 - rotAngle) > angleConsistentThr)	
		return false;
	
	return true;
}

void CLineMatching::addFansMatch(float desDistThr, float fDistThr)
{
	int nEnterCluster = 4;	
	int nNearPts = 8;
	Mat pointMatches;

	vpointMatch2Mat(vstrPointMatch, pointMatches);	
	
	vector<strFanSection> vMatchedFans1, vMatchedFans2, vUnmatchedFans1, vUnmatchedFans2;

	bifurcateFans(vstrFanSection1, vstrFanSection2, vstrFanMatch, vMatchedFans1, vMatchedFans2, vUnmatchedFans1, vUnmatchedFans2);

	vector<vector<vector<int> > >  vGroupedUnFans1, vGroupedUnFans2;
	groupFans(vMatchedFans1, vUnmatchedFans1, nEnterCluster, vGroupedUnFans1);
	groupFans(vMatchedFans2, vUnmatchedFans2, nEnterCluster, vGroupedUnFans2);


	Mat nearestMPts1, nearestMPts2;
	nearestMatchedPtsToFan(vUnmatchedFans1, pointMatches.colRange(0, 2), nNearPts, nearestMPts1);
	nearestMatchedPtsToFan(vUnmatchedFans2, pointMatches.colRange(2, 4), nNearPts, nearestMPts2);

	vector<strFanMatch> tvstrFanMatch;
	int ngroup = vGroupedUnFans2.size();
	for (int i = 0; i < ngroup; i++)
	{		
		strFanSection curMFan1 = vMatchedFans1[i];
		strFanSection curMFan2 = vMatchedFans2[i];
		float refRotAng = estimateLocalRotateAngle(curMFan1, curMFan2);

		for (int j = 0; j < 4; j++)
		{
			vector<int> serFan1 = vGroupedUnFans1[i].at(j);
			vector<int> serFan2 = vGroupedUnFans2[i].at(j);
			int nserFan1 = serFan1.size();
			int nserFan2 = serFan2.size();
			if (nserFan1 == 0 || nserFan2 == 0)			
				continue;
			
			for (int m = 0; m < nserFan1; m++)
			{
				float minDesDist = 100;
				strFanMatch tstrFanMatch;
				bool flag = 0;
				int ser1 = serFan1[m];
				strFanSection curFan1 = vUnmatchedFans1[ser1];
				Mat curNMPts1 = nearestMPts1.row(ser1);
				Mat curDes1 = curFan1.mDes;
				float difAng1 = fmod(2*CV_PI+curFan1.strbranch2.ang - curFan1.strbranch1.ang, 2*CV_PI);

				for (int n = 0; n < nserFan2; n++)
				{						
					int ser2 = serFan2[n];
					strFanSection curFan2 = vUnmatchedFans2[ser2];
					float difAng2 = fmod(2*CV_PI+curFan2.strbranch2.ang - curFan2.strbranch1.ang, 2*CV_PI);

					if (abs(difAng1- difAng2) > _difAngThr)			
						continue;

					if ( ! isConsistentWithLocalRotateAngle(curFan1, curFan2, refRotAng))
						continue;
					
					Mat curDes2 = curFan2.mDes;
					float desDist = descriptorDistance(curDes1, curDes2, _nAvgDescriptorDistance);						
					if (desDist > desDistThr)		
						continue;
					
					if ( ! isConsistentWithFMat(curFan1.intsection, curFan2.intsection, FMat, fDistThr) )
						continue;

					Mat curNMPts2 = nearestMPts2.row(ser2);
 					if ( ! isCurPairFanConsistentWithNearMPts(curFan1, curFan2, curNMPts1, curNMPts2, pointMatches) )
 						continue;
	
					if (desDist < minDesDist)
					{
						flag = 1;
						tstrFanMatch.dist = desDist;
						tstrFanMatch.fserial1 = vUnmatchedFans1[ser1].fanSerial;
						tstrFanMatch.fserial2 = vUnmatchedFans2[ser2].fanSerial;
						minDesDist = desDist;						
					}
				}
				if (flag)
				{
					tvstrFanMatch.push_back(tstrFanMatch);
				}
			}
		}
	}

	if (tvstrFanMatch.empty())
		return;

	uniqueFanMatch(tvstrFanMatch);


	cout<<"tvstrFanMatch:" <<tvstrFanMatch.size();

	int nNewFanMatch = tvstrFanMatch.size();
	for (int i = 0; i < nNewFanMatch; i++)	
		vstrFanMatch.push_back(tvstrFanMatch[i]);		
}

bool CLineMatching::isCurPairFanConsistentWithNearMPts(strFanSection strFan1, strFanSection strFan2, Mat candIdx1, Mat candIdx2, Mat pointMatch)
{	
	Mat intstMat, keepIdx;	
	CMatOperation* pMatOperation = new CMatOperation;
	pMatOperation->intersectSet(candIdx1, candIdx2, intstMat, keepIdx);
	if (((float)(intstMat.rows)) / (candIdx1.cols)< 0.7)
		return false;

	Mat unionIdx;
	pMatOperation->unionSet(candIdx1, candIdx2, unionIdx);

	int nuni = unionIdx.cols;
	int cons = 0;
	int* pdat = unionIdx.ptr<int>(0);
	for (int m = 0; m<nuni; m++)
	{
		int curIdx = pdat[m];
		float* pCurPtMatch = pointMatch.ptr<float>(curIdx);			
		Point2f tpt1 = Point2f(pCurPtMatch[0], pCurPtMatch[1]);
		Point2f tpt2 = Point2f(pCurPtMatch[2], pCurPtMatch[3]);
		Mat_<float> curLine1 = (Mat_<float>(1, 3) << strFan1.intsection.x, strFan1.intsection.y, strFan1.strbranch1.ang);
		Mat_<float> curLine2 = (Mat_<float>(1, 3) << strFan2.intsection.x, strFan2.intsection.y, strFan2.strbranch1.ang);
		float refDirection1 = strFan1.strbranch2.ang;
		float refDirection2 = strFan2.strbranch2.ang;

		int isSameSide1 = determinePtLinePos(tpt1, curLine1, refDirection1);
		int isSameSide2 = determinePtLinePos(tpt2, curLine2, refDirection2);        
		if (isSameSide1 == isSameSide2 || (isSameSide1 == 0 || isSameSide2 == 0))
			cons = cons + 1;

		curLine1(0, 2) = strFan1.strbranch2.ang;
		curLine2(0, 2) = strFan2.strbranch2.ang;
		refDirection1 = strFan1.strbranch1.ang;
		refDirection2 = strFan2.strbranch1.ang;

		float dist1, dist2;
		isSameSide1 = determinePtLinePos(tpt1, curLine1, refDirection1);
		isSameSide2 = determinePtLinePos(tpt2, curLine2, refDirection2);        
		if (isSameSide1 == isSameSide2 || (isSameSide1 == 0 || isSameSide2 == 0))
			cons = cons + 1;
	}
	float fcons = (float) cons;
   	if ( fcons / (2* nuni) < _sameSideRatio)
		return false;		

	return true;
}

void CLineMatching::groupFans(vector<strFanSection> vMatchedFans, vector<strFanSection> vUnmatchedFans,  int nEnterCluster, 
												  vector<vector<vector<int> > >  &vGroupedUnjunc)
{
	int nunFans = vUnmatchedFans.size();
	int nMatchedFans = vMatchedFans.size();
	vector<vector<int> >   vEnteredunFans(nMatchedFans);	
	Mat enteringFans;

	for (int i = 0; i < nunFans; i++)
	{
		Point2f intersection = vUnmatchedFans[i].intsection;		
		Mat dist1, dist2;
		for (int j = 0; j < nMatchedFans; j++)
		{
			float ft1 = vMatchedFans[j].intsection.x - intersection.x;
			dist1.push_back(ft1);
			float ft2 = vMatchedFans[j].intsection.y - intersection.y;
			dist2.push_back(ft2);
		}		
		Mat squeDist1, squeDist2;
		pow(dist1, 2., squeDist1);
		pow(dist2, 2., squeDist2);
		Mat dist = squeDist1 + squeDist2;
		Mat sortedIdx;
		dist = dist.t();
		sortIdx(dist, sortedIdx, SORT_EVERY_ROW);
		Mat enteredIdx = sortedIdx.colRange(0, nEnterCluster);
		enteringFans.push_back(enteredIdx);

		int* pdat1 = enteredIdx.ptr<int>(0);
		vector<int> tvint;
		for (int j = 0; j < nEnterCluster; j++)
		{
			int ser = pdat1[j];
			vEnteredunFans[ser].push_back(i);
		}
	}
	vGroupedUnjunc.resize(nMatchedFans);
	for (int i = 0; i < nMatchedFans; i++)
	{
		vGroupedUnjunc[i].resize(4);
	}
	
	for (int i = 0; i < nMatchedFans; i++)
	{
		strFanSection curFan = vMatchedFans[i];		
		vector<int> curunFans = vEnteredunFans[i];
		int nunjunc = curunFans.size();
		for (int j = 0; j < nunjunc; j++)
		{
			int tser = curunFans[j];
			strFanSection tFan = vUnmatchedFans[tser];
			Point2f unJunc = tFan.intsection;

			Mat_<float> curLine = (Mat_<float>(1, 3) << curFan.intsection.x,
																					curFan.intsection.y,
																					curFan.strbranch1.ang);			
			float refDirection = curFan.strbranch2.ang;
			int isSameSide1 = determinePtLinePos(unJunc, curLine, refDirection);
			curLine(0, 2) = curFan.strbranch2.ang;						
			refDirection = curFan.strbranch1.ang;
			int isSameSide2 = determinePtLinePos(unJunc, curLine, refDirection);
			if (isSameSide1 >= 0 && isSameSide2 >= 0)
				vGroupedUnjunc[i].at(0).push_back(tser);
			if (isSameSide1 >= 0 && isSameSide2 <= 0)
				vGroupedUnjunc[i].at(1).push_back(tser);
			if (isSameSide1 <= 0 && isSameSide2 >= 0)
				vGroupedUnjunc[i].at(2).push_back(tser);
			if (isSameSide1 <= 0 && isSameSide2 <= 0)
				vGroupedUnjunc[i].at(3).push_back(tser);
		}
	}
}

void CLineMatching::topoFilter(vector<strFanMatch> &vStrFanMatch, vector<strPointMatch> vStrPointMatch)
{		
	Mat pointMatch;
	CMatOperation *pMatOperation = new CMatOperation;
	vpointMatch2Mat(vStrPointMatch, pointMatch);

 	int nFanMatch = vStrFanMatch.size();	
	vector<strFanMatch> tvStrFanMatch;	
	for (int i = 0; i < nFanMatch; i++)
	{
		int ser1 = vStrFanMatch[i].fserial1;		
		strFanSection curFan1 = vstrFanSection1[ser1];
		int ser2 = vStrFanMatch[i].fserial2;		
		strFanSection curFan2 = vstrFanSection2[ser2];		
		if (isConsistentWithNeighborPointMatches(curFan1, curFan2, pointMatch))
		{
			tvStrFanMatch.push_back(vStrFanMatch[i]);			
		}
	}
	vStrFanMatch = tvStrFanMatch;

	delete pMatOperation;
	pMatOperation = NULL;
}

bool CLineMatching::isConsistentWithNeighborPointMatches(strFanSection strFan1, strFanSection strFan2, Mat pointMatch)
{		
	int nCandNum = min(pointMatch.rows, _nNeighborPts);
	Point2f cenPt1 = strFan1.intsection;	
	Mat xdiff= pointMatch.col(0) - cenPt1.x;	
	Mat xdiff2, ydiff2;
	pow(xdiff, 2., xdiff2);		
	Mat ydiff= pointMatch.col(1) - cenPt1.y;		
	pow(ydiff, 2., ydiff2);
	Mat dist = xdiff2 + ydiff2;		
	Mat sortedIdx;
	sortIdx(dist, sortedIdx, SORT_EVERY_COLUMN);		
	Mat candIdx1 = sortedIdx.rowRange(0, nCandNum).t();		

	Point2f cenPt2 = strFan2.intsection;		
	xdiff= pointMatch.col(2) - cenPt2.x;
	ydiff= pointMatch.col(3) - cenPt2.y;		
	pow(xdiff, 2., xdiff2);
	pow(ydiff, 2., ydiff2);

	dist = xdiff2 + ydiff2;		
	sortIdx(dist, sortedIdx, SORT_EVERY_COLUMN);
	Mat candIdx2 = sortedIdx.rowRange(0, nCandNum).t();

	Mat intstMat, keepIdx;	
	CMatOperation* pMatOperation = new CMatOperation;
	pMatOperation->intersectSet(candIdx1, candIdx2, intstMat, keepIdx);
	if (((float)(intstMat.rows))/nCandNum < 0.7)
		return false;

	Mat unionIdx;
	pMatOperation->unionSet(candIdx1, candIdx2, unionIdx);
	int nuni = unionIdx.cols;
	int cons = 0;
	int* pdat = unionIdx.ptr<int>(0);
	for (int m = 0; m<nuni; m++)
	{
		int curIdx = pdat[m];
		float* pCurPtMatch = pointMatch.ptr<float>(curIdx);			
		Point2f tpt1 = Point2f(pCurPtMatch[0], pCurPtMatch[1]);
		Point2f tpt2 = Point2f(pCurPtMatch[2], pCurPtMatch[3]);
		Mat_<float> curLine1 = (Mat_<float>(1, 3) << strFan1.intsection.x, strFan1.intsection.y, strFan1.strbranch1.ang);
		Mat_<float> curLine2 = (Mat_<float>(1, 3) << strFan2.intsection.x, strFan2.intsection.y, strFan2.strbranch1.ang);
		float refDirection1 = strFan1.strbranch2.ang;
		float refDirection2 = strFan2.strbranch2.ang;

		int isSameSide1 = determinePtLinePos(tpt1, curLine1, refDirection1);
		int isSameSide2 = determinePtLinePos(tpt2, curLine2, refDirection2);        
		if (isSameSide1 == isSameSide2 || (isSameSide1 == 0 || isSameSide2 == 0))
			cons = cons + 1;

		curLine1(0, 2) = strFan1.strbranch2.ang;
		curLine2(0, 2) = strFan2.strbranch2.ang;
		refDirection1 = strFan1.strbranch1.ang;
		refDirection2 = strFan2.strbranch1.ang;
	
		float dist1, dist2;
  		isSameSide1 = determinePtLinePos(tpt1, curLine1, refDirection1);
		isSameSide2 = determinePtLinePos(tpt2, curLine2, refDirection2);        
 		if (isSameSide1 == isSameSide2 || (isSameSide1 == 0 || isSameSide2 == 0))
			cons = cons + 1;
	} 
 	float fcons = (float) cons;
     if ( fcons / (2* nuni) < _sameSideRatio)
		return false;				

	return true;
}

int CLineMatching::determinePtLinePos(Point2f pt, Mat_<float> pline, float refDirection)
{
	Vec2f vect1 = Vec2f(pt.x - pline(0, 0), pt.y - pline(0, 1));
	Vec2f vect2 = Vec2f(cos(pline(0, 2)), sin(pline(0, 2)));
	Vec2f refv = Vec2f(cos(refDirection), sin(refDirection));
	float dist = distPt2Line(pt, pline);
	if (dist < 1)
	{
		return 0;		
	}	
	
	float cross1 = refv(1)   * vect2(0)	- refv(0)   * vect2(1);
	float cross2 = vect1(1) * vect2(0) - vect1(0) * vect2(1);    
	float tmp = cross1 * cross2;
	if (tmp > 0)
		return 1;
	else 
		return -1;		
}

float CLineMatching::distPt2Line(Point2f pt, Mat_<float> pline)
{	
	Vec2f vect1 = Vec2f(pt.x-pline(0,0), pt.y-pline(0,1));
	Vec2f vect2 = Vec2f(cos(pline(0, 2)), sin(pline(0, 2)));
	float dist = vect1(0) * vect2(1) - vect1(1) * vect2(0);
	dist = abs(dist);
	return dist;
}

bool CLineMatching::isConsistentWithHMat(Point2f pt1, Point2f pt2, Mat hmat, float hDistThr)
{
	Mat mpt1 = (Mat_<float>(3, 1)<<pt1.x, pt1.y, 1);
	Mat hopt1 = hmat * mpt1;
	hopt1 = hopt1 / hopt1.at<float>(2);
	Mat mpt2 = (Mat_<float>(3, 1)<<pt2.x, pt2.y, 1);
	Mat inv_hmat = hmat.inv();
	Mat hopt2 = inv_hmat * mpt2;
	hopt2 = hopt2 / hopt2.at<float>(2);
	float dist1 = norm(mpt2 - hopt1);
	float dist2 = norm(mpt1 - hopt2);	
	float dist = (dist1 + dist2) / 2;
	bool flag = ( dist < hDistThr) ? 1 : 0;
	return flag;	
}

void CLineMatching::updatePointMatchFromFanMatches()
{
	vstrPointMatch.clear();

	strPointMatch tstrPointMatch;
	int nptsMatches = vstrFanMatch.size();
	for(int i = 0; i < nptsMatches; i++)
	{
		int serial1 = vstrFanMatch[i].fserial1;
		Point2f tpt = vstrFanSection1[serial1].intsection;
		tstrPointMatch.point1 = tpt;		
		int serial2 = vstrFanMatch[i].fserial2;
		tpt = vstrFanSection2[serial2].intsection;
		tstrPointMatch.point2 = tpt;				
		vstrPointMatch.push_back(tstrPointMatch);
	}
	
//	adjustLineMatchEndpoints(vstrLineMatch);
	int nFanMatch = vstrFanMatch.size();
	Mat hmat;
	float hDistThr = _junctionDistThr;
	for (int i = 0; i < nFanMatch; i++)
	{		
		hmat.release();
		int fserial1 = vstrFanMatch[i].fserial1;	
		int fserial2 = vstrFanMatch[i].fserial2;			
		strFanSection curFan1 = vstrFanSection1[fserial1];
		strFanSection curFan2 = vstrFanSection2[fserial2];		
		if (_isTwoLineHomog)
		{
			calcLocalHomography_2lineFMat(curFan1, curFan2, FMat, hmat);
		}
		else
		{
			calcLocalHomography(curFan1, curFan2, FMat, hmat);
		}		

		strLine curLine1 = strline1[curFan1.strbranch1.lineSerial];
		strLine curLine2 = strline1[curFan1.strbranch2.lineSerial];
		strLine curLine3 = strline2[curFan2.strbranch1.lineSerial];
		strLine curLine4 = strline2[curFan2.strbranch2.lineSerial];
		float ang1 = curLine1.direction;
		float ang2 = curLine3.direction;
		float offset = abs(ang1 - ang2);
		if ( (ang1 < 0.5*CV_PI && ang2 > 1.5 * CV_PI && ang2-ang1<=1.5*CV_PI) ||  (ang1 > 1.5*CV_PI && ang2 < 0.5 * CV_PI && ang1-ang2 <= 1.5*CV_PI) )
		{
			Point2f tpt = curLine3.ps;
			curLine3.ps = curLine3.pe;
			curLine3.pe = tpt;
		}
		ang1 = curLine2.direction;
		ang2 = curLine4.direction;
		offset = abs(ang1 - ang2);
		if ( (ang1 < 0.5*CV_PI && ang2 > 1.5 * CV_PI && ang2-ang1<=1.5*CV_PI) ||  (ang1 > 1.5*CV_PI && ang2 < 0.5 * CV_PI && ang1-ang2 <= 1.5*CV_PI) )
		{
			Point2f tpt = curLine4.ps;
			curLine4.ps = curLine4.pe;
			curLine4.pe = tpt;
		}
		
		Point2f tpt[8] = { curLine1.ps, curLine1.pe, curLine2.ps, curLine2.pe, 
									curLine3.ps, curLine3.pe, curLine4.ps, curLine4.pe};

		for (int j = 0; j < 4; j++)
		{			
			Point2f pt1 = tpt[j];
			Point2f pt2 = tpt[j+4];
			float dx = pt1.x - curFan1.intsection.x;
			float dy = pt1.y - curFan1.intsection.y;
			float dist1 = sqrt(dx*dx+dy*dy);
			dx = pt2.x - curFan2.intsection.x;
			dy = pt2.y - curFan2.intsection.y;
			float dist2 = sqrt(dx*dx+dy*dy);

			
			Mat mpt1 = (Mat_<float>(3, 1)<<pt1.x, pt1.y, 1);
			Mat hopt1 = hmat * mpt1;
			hopt1 = hopt1 / hopt1.at<float>(2);
			Mat mpt2 = (Mat_<float>(3, 1)<<pt2.x, pt2.y, 1);
			Mat inv_hmat = hmat.inv();
			Mat hopt2 = inv_hmat * mpt2;
			hopt2 = hopt2 / hopt2.at<float>(2);
			float tdist1 = norm(mpt2 - hopt1);
			float tdist2 = norm(mpt1 - hopt2);	
			float tdist = (tdist1 + tdist2) / 2;
			bool flag = ( tdist < hDistThr) ? 1 : 0;			

			if (dist1 < 3 || dist2 < 3)
			{
				continue;
			}
			if(isConsistentWithHMat(pt1, pt2, hmat, hDistThr))
			{
				strPointMatch tstrPointMatch;
				tstrPointMatch.point1 = pt1;				
				tstrPointMatch.point2 = pt2;				
				vstrPointMatch.push_back(tstrPointMatch);
			}
		}
	}		
 	uniquePointMatch(vstrPointMatch);
}

void CLineMatching::vpointMatch2Mat(vector<strPointMatch> tvstrPointMatch, Mat &outMat)
{
	int nmatch = tvstrPointMatch.size();
	outMat.release();
	outMat.create(nmatch, 4, CV_32F);
	for (int i = 0; i < nmatch; i++)
	{
		float *pdat = outMat.ptr<float>(i);
		pdat[0] = tvstrPointMatch[i].point1.x;
		pdat[1] = tvstrPointMatch[i].point1.y;
		pdat[2] = tvstrPointMatch[i].point2.x;
		pdat[3] = tvstrPointMatch[i].point2.y;					
	}	
}

void CLineMatching::uniquePointMatch(vector<strPointMatch> &vStrPointMatch)
{
	int nmatch = vStrPointMatch.size();
	Mat pointMatch;
	vpointMatch2Mat(vStrPointMatch, pointMatch);

	int rows = pointMatch.rows;	
	Mat dist2Ori1 = pointMatch.col(0).mul(pointMatch.col(0)) + pointMatch.col(1).mul(pointMatch.col(1));	
	Mat tmat1, sortedIdx1;
	sortIdx(dist2Ori1,  sortedIdx1, CV_SORT_EVERY_COLUMN);							
	for (int i = 0; i < rows; i++)
	{
		int *pSer = sortedIdx1.ptr<int>(0);
		tmat1.push_back(pointMatch.row(pSer[i]));		
	}	
	sortedIdx1 = sortedIdx1.t();
	vector<int> vkeptIdx;
	int* pdat = sortedIdx1.ptr<int>(0);	
	vkeptIdx.push_back(pdat[0]);
	Mat tmat2;
	tmat2.push_back(tmat1.row(0));

	for (int i = 1; i < rows; i++)
	{				
		Mat rmat = tmat1.row(i) - tmat1.row(i-1);
		float tmp1 = norm(rmat.colRange(0, 2));
		float tmp2 = norm(rmat.colRange(2, 4));
		float tmp3 = norm(rmat);		
		if ( (tmp1 < _radiusPointMatchUnique && tmp2 > _radiusPointMatchUnique) || (tmp1>_radiusPointMatchUnique && tmp2<_radiusPointMatchUnique) || (tmp3 < _radiusPointMatchUnique*1.414) )
			continue;
		
		vkeptIdx.push_back(pdat[i]);		
		tmat2.push_back(tmat1.row(i));
	}

	Mat tmat3, sortedIdx2;
	Mat dist2Ori2 = tmat2.col(2).mul(tmat2.col(2)) + tmat2.col(3).mul(tmat2.col(3));		
	sortIdx(dist2Ori2,  sortedIdx2, CV_SORT_EVERY_COLUMN);						
	rows = tmat2.rows;
	for (int i = 0; i < rows; i++)
	{
		int *pSer = sortedIdx2.ptr<int>(0);
		tmat3.push_back(tmat2.row(pSer[i]));		
	}	

	sortedIdx2 = sortedIdx2.t();
	vector<int> vkeptIdx1;
	pdat = sortedIdx2.ptr<int>(0);	
	int newSer = vkeptIdx[pdat[0]];	
	vkeptIdx1.push_back(newSer);	
	for (int i = 1; i < rows; i++)
	{				
		Mat rmat = tmat3.row(i) - tmat3.row(i-1);
		float tmp1 = norm(rmat.colRange(0, 2));
		float tmp2 = norm(rmat.colRange(2, 4));
		float tmp3 = norm(rmat);		
		if ( (tmp1 < _radiusPointMatchUnique && tmp2 > _radiusPointMatchUnique) || (tmp1>_radiusPointMatchUnique && tmp2<_radiusPointMatchUnique) || (tmp3 <_radiusPointMatchUnique*1.414) )
			continue;

		newSer = vkeptIdx[pdat[i]];
		vkeptIdx1.push_back(newSer);				
	}

	vector<strPointMatch> tvstrPointMatch;	
	nmatch = vkeptIdx1.size();
	for (int i = 0; i < nmatch; i++)		
		tvstrPointMatch.push_back(vStrPointMatch[vkeptIdx1[i]]);		
	
	vStrPointMatch.clear();
	vStrPointMatch = tvstrPointMatch;

	Mat outMat;
	vpointMatch2Mat(vStrPointMatch, outMat);	
}

void CLineMatching::unique(Mat mat, vector<int> &keptIdx)
{
	Mat tmat, sortedIdx;
	sortrows(mat, tmat, sortedIdx, 0, 1);
	sortedIdx = sortedIdx.t();

	int rows = tmat.rows;
	int* pdat = sortedIdx.ptr<int>(0);
	keptIdx.push_back(pdat[0]);
	CMatOperation *pMatOperation = new CMatOperation;	
	for (int i = 1; i < rows; i++)
	{				
		Mat matSub = tmat.row(i) - tmat.row(i-1);		
		bool pSign;
		pMatOperation->any(matSub, &pSign);
		if ( pSign)		
			keptIdx.push_back(pdat[i]);
	}
	delete pMatOperation;
	pMatOperation = NULL;
}

bool CLineMatching::isAcceptedToBePointMatch(Vec3f pt1, Vec3f pt2, float desDistThr, float FDistThr, float &desDist)
{
	Mat des1, des2;
	describeSingleLine(pt1, gMag1, gDir1, des1);	
	describeSingleLine(pt2, gMag2, gDir2, des2);
	desDist = norm((des1 - des2), NORM_L2);
	desDist *= desDist;
	if (desDist > desDistThr)		
		return false;

	return isConsistentWithFMat(Point2f(pt1(0), pt1(1)), Point2f(pt2(0), pt2(1)), FMat, FDistThr);
/*
	FMat.convertTo(FMat, CV_32F);
	Mat_<float> homoPoint = (Mat_<float>(3,1) << pt1[0], pt1[1], 1);
	Mat_<float> lineCoeffs = FMat * homoPoint;		
	float a = lineCoeffs(0);
	float b = lineCoeffs(1);
	float c = lineCoeffs(2);
	float dist = fabs(a*pt2[0] + b*pt2[1] + c)/sqrt(a*a + b*b);
	if (dist > FDistThr)	
		return false;
	
	homoPoint = (Mat_<float>(3,1) << pt2[0], pt2[1], 1);
	lineCoeffs = (FMat.t()) * homoPoint;		
	a = lineCoeffs(0);
	b = lineCoeffs(1);
	c = lineCoeffs(2);
	dist = fabs(a*pt1[0] + b*pt1[1] + c)/sqrt(a*a + b*b);
	if (dist > FDistThr)	
		return false;
	*/
// 	Mat mpt1, mpt2;
// 	mpt1 = (Mat_<float>(1, 2) << pt1[0], pt1[1]);  
// 	mpt2 = (Mat_<float>(1, 2) << pt2[0], pt2[1]);  
// 	Mat_<float> residual;
// 	getResiduauls(mpt1, mpt2, FMat, residual);
// 	if (residual[0][0] > FDistThr)		
// 		return false;		
}

bool CLineMatching::isConsistentWithFMat(Point2f pt1, Point2f pt2, Mat FMat, float FDistThr)
{
	//FMat.convertTo(FMat, CV_32F);
	Mat_<float> homoPoint = (Mat_<float>(3,1) << pt1.x, pt1.y, 1);
	Mat_<float> lineCoeffs = FMat * homoPoint;		
	float a = lineCoeffs(0);
	float b = lineCoeffs(1);
	float c = lineCoeffs(2);
	float dist = fabs(a*pt2.x + b*pt2.y + c)/sqrt(a*a + b*b);
	if (dist > FDistThr)	
		return false;

	homoPoint = (Mat_<float>(3,1) << pt2.x, pt2.y, 1);
	lineCoeffs = (FMat.t()) * homoPoint;		
	a = lineCoeffs(0);
	b = lineCoeffs(1);
	c = lineCoeffs(2);
	dist = fabs(a*pt1.x + b*pt1.y + c)/sqrt(a*a + b*b);
	if (dist > FDistThr)	
		return false;

	return true;
}


void CLineMatching::uniqueChk(Mat inMat, Vec3i regulation, vector<int> &vkeptIdx, Mat &outMat)
{
	Mat tmat1, tmat2;
	Mat sortedIdx;
	sortrows(inMat, tmat1, sortedIdx, regulation[0], regulation[2]);
	sortedIdx = sortedIdx.t();
	int rows = inMat.rows;
	Mat toutMat;
	vector<int> vkeptIdx1;
	tmat2 = tmat1.col(regulation[0]).t();
	float *pcol = tmat2.ptr<float>(0);		
	int *pIdx = sortedIdx.ptr<int>(0);		
	vkeptIdx1.push_back(pIdx[0]);
	toutMat.push_back(tmat1.row(0).clone());		
	for (int i = 1; i < rows; i++)
	{			
		if (pcol[i] != pcol[i-1])
		{
			vkeptIdx1.push_back(pIdx[i]);		
			toutMat.push_back(tmat1.row(i).clone());
		}
	}

	tmat1.release();
	sortrows(toutMat, tmat1, sortedIdx, regulation[1], regulation[2]);	
	sortedIdx = sortedIdx.t();
	rows = toutMat.rows;
	tmat2 = tmat1.col(regulation[1]).t();
	pcol = tmat2.ptr<float>(0);				
	pIdx = sortedIdx.ptr<int>(0);
	int curIdx = pIdx[0];
	vkeptIdx.push_back(vkeptIdx1.at(curIdx));		
	outMat.push_back(tmat1.row(0).clone());
	for (int i = 1; i < rows; i++)
	{			
		if (pcol[i] != pcol[i-1])
		{
			curIdx = pIdx[i];
			vkeptIdx.push_back(vkeptIdx1.at(curIdx));		
			outMat.push_back(tmat1.row(i).clone());
		}
	}
}


void CLineMatching::uniqueChk(Mat inMat, Vec3i regulation, vector<int> &vkeptIdx)
{
	Mat tmat1, tmat2;
	Mat sortedIdx;
//	cout<<inMat<<endl;
	sortrows(inMat, tmat1, sortedIdx, regulation[0], regulation[2]);
	sortedIdx = sortedIdx.t();
//	cout<<sortedIdx<<endl;
	int rows = inMat.rows;
	Mat toutMat;
// 	if(inMat.rows == tmat2.rows)
// 	{
// 		toutMat = inMat;
// 		for (int i = 1; i < rows; i++)
// 				vkeptIdx.push_back(i);
// 	}
// 	else
// 	{		
		vector<int> vkeptIdx1;
		tmat2 = tmat1.col(regulation[0]).t();
		float *pcol = tmat2.ptr<float>(0);		
		int *pIdx = sortedIdx.ptr<int>(0);		
		vkeptIdx1.push_back(pIdx[0]);
		toutMat.push_back(tmat1.row(0).clone());		
		for (int i = 1; i < rows; i++)
		{			
			if (pcol[i] != pcol[i-1])
			{
				vkeptIdx1.push_back(pIdx[i]);		
				toutMat.push_back(tmat1.row(i).clone());
			}
		}
//		cout<<toutMat<<endl;
	// // }
	tmat1.release();
	sortrows(toutMat, tmat1, sortedIdx, regulation[1], regulation[2]);	
	sortedIdx = sortedIdx.t();
	//cout<<sortedIdx<<endl;
//	cout<<tmat1<<endl;
//	cout<<tmat1.rowRange(0, 10)<<endl;
// 	if(inMat.rows == tmat2.rows)
// 	{
// 		toutMat = toutMat;
// 		for (int i = 1; i < rows; i++)
// 			vkeptIdx.push_back(i);
// 	}
// 	else
// 	{
		rows = toutMat.rows;
	//	vkeptIdx.clear();
		tmat2 = tmat1.col(regulation[1]).t();
	//	cout<<tmat2<<endl;		
		pcol = tmat2.ptr<float>(0);				
		pIdx = sortedIdx.ptr<int>(0);
		int curIdx = pIdx[0];
		vkeptIdx.push_back(vkeptIdx1.at(curIdx));		
		// outMat.push_back(tmat1.row(0).clone());
		for (int i = 1; i < rows; i++)
		{			
			if (pcol[i] != pcol[i-1])
			{
				curIdx = pIdx[i];
				vkeptIdx.push_back(vkeptIdx1.at(curIdx));		
				// outMat.push_back(tmat1.row(i).clone());
			}
		}
}



void CLineMatching::sortrows(Mat inMat, Mat &outMat, Mat &sortedComIdx, int primiaryKey, int secondaryKey)
{
	int rows = inMat.rows;	
	Mat primColumn = inMat.col(primiaryKey).clone();
	Mat secColumn = inMat.col(secondaryKey).clone();
	Mat comCol = 100*primColumn + 0.01*secColumn;
	sortIdx(comCol,  sortedComIdx, CV_SORT_EVERY_COLUMN);		
	outMat = Mat(inMat.size(), inMat.type());
	int nrows = outMat.rows;
	for (int i = 0; i < nrows; i++)
	{
		int *pSer = sortedComIdx.ptr<int>(0);
		inMat.row(pSer[i]).copyTo(outMat.row(i));
	}	
}


void CLineMatching::plotLineMatches(Mat img1, Mat img2, vector<strLineMatch> vStrLineMatch, string imgName)
{
	Mat combinedImg;
	concatenateTwoImgs(img1, img2, combinedImg);
	int nlineMatches = vStrLineMatch.size();
	int cols1 = img1.cols;

	int color[6][3];
	color[0][0]=255;color[0][1]=0;  color[0][2]=0;
	color[1][0]=0;  color[1][1]=255;color[1][2]=0;
	color[2][0]=0;  color[2][1]=0;  color[2][2]=255;
	color[3][0]=255;color[3][1]=255;color[3][2]=0;
	color[4][0]=0;color[4][1]=255; color[4][2]=255;
	color[5][0]=255;color[5][1]=0; color[5][2]=255;


	for (int i = 0; i < nlineMatches; i++)
	{
		int color_num=rand()%6;
		int R=color[color_num][0];
		int G=color[color_num][1];
		int B=color[color_num][2];	
		int ser1 = vStrLineMatch[i].serLine1;		
		Point2f spt1 = strline1[ser1].ps;
		Point2f ept1 = strline1[ser1].pe; 
		line(combinedImg, spt1, ept1, cvScalar(R,G,B), 2 );

		int ser2 = vStrLineMatch[i].serLine2;		
		Point2f spt2 = strline2[ser2].ps;
		spt2.x += cols1;
		Point2f ept2 = strline2[ser2].pe;
		ept2.x += cols1;
		line(combinedImg, spt2, ept2, cvScalar(R,G,B), 2 );
		stringstream ss;		
		string str;
		ss<<i;
		ss>>str;
		Point2i midpt1, midpt2, tpt;
		tpt = spt1 + ept1;
		midpt1.x = (int) tpt.x/2;
		midpt1.y = (int) tpt.y/2;		
		putText(combinedImg, str, midpt1, FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(R,G,B), 1);
		tpt = spt2 + ept2;
		midpt2.x = (int) tpt.x/2;
		midpt2.y = (int) tpt.y/2;		
		putText(combinedImg, str, midpt2, FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(R,G,B), 1);		
	}	

	imshow(imgName, combinedImg);
	waitKey(20);
//	imwrite(imgName, combinedImg);	
}

void CLineMatching::description(Mat pts, const Mat gMag, const Mat gDir, Mat &descriptors)
{	
	int nfan = pts.rows;	
	for (int i = 0; i < nfan; i += 3)
	{		
		Mat mDes;		
		float *pdat1 = pts.ptr<float>(i);
		float *pdat2 = pts.ptr<float>(i+1);
		float *pdat3 = pts.ptr<float>(i+2);

		float dx = pdat2[0] - pdat1[0];
		float dy = pdat2[1] - pdat1[1];
		float ang1 = fast_mod(atan2(dy, dx));
		dx = pdat3[0] - pdat1[0];
		dy = pdat3[1] - pdat1[1];
		float ang2 = fast_mod(atan2(dy, dx));
		float angs[2] = { ang1, ang2 };
		Vec3f pt;
		pt[0] = pdat1[0];
		pt[1] = pdat1[1];
		for (int j = 0; j < 2; j++)
		{									
			pt[2] = angs[j];
			Mat des;
			description_sift_single(pt, gMag, gDir, des);
			mDes.push_back(des);					
		}
		descriptors.push_back(mDes.reshape(1, 1));
	}
}

void CLineMatching::get_asift_description(Mat img, Mat pts, vector<Mat> &vDes)
{
	int nfan = pts.rows / 3;	
	int rows = img.rows, cols = img.cols;
	Mat A = (Mat_<float>(2, 3)<<1, 0, 0, 0, 1, 0);			
	int noctave = _nOctave;
	int noctaveLayer = _nOctaveLayer;
	float downSampleRatio = 0.5;
	vector<Vec2f> vparam;
	Vec2f tvec;
	pts = pts.t();
	Mat mones = Mat::ones(1, pts.cols, CV_32F);
	pts.push_back(mones);

	tvec(0) = 1.0;
	tvec(1) = 0.0;
	vparam.push_back(tvec);
	for (int i = 1; i < 1; i++)
	{
		float  tilt = powf(2.0, 0.5 * i);
		float phi = 0;
		while(phi <  180)
		{
			tvec(0) = tilt;
			tvec(1) = phi;
			vparam.push_back(tvec);
			phi += 72.0 / tilt;			
		}
	}

	int nparams = vparam.size();
	for (int i = 0; i < nparams; i++)
	{
		float tilt =  vparam[i].val[0];
		float phi =  vparam[i].val[1];		
		Mat ttransImg = img;
		if (phi != 0)
		{
			phi = phi * CV_PI / 180;
			float sinPhi = sin(phi), cosPhi = cos(phi);
			A = (Mat_<float>(2, 2) << cosPhi, -sinPhi, sinPhi, cosPhi);					
			vector<Point2f> vCor;
			Mat corners = (Mat_<float>(4, 2) << 0, 0, cols, 0, cols, rows, 0, rows);				
			Mat tcorners = corners * A.t();
			tcorners = tcorners.reshape(2);		
			Rect bRect = boundingRect(tcorners);				
			Mat tmpB = (Mat_<float>(1,2) << -bRect.x, -bRect.y);
			A = A.t();
			A.push_back(tmpB);				
			A = A.t();						
			warpAffine(ttransImg, ttransImg, A, cvSize(bRect.width, bRect.height), CV_INTER_LINEAR, BORDER_REPLICATE);			
		}
		if (tilt != 1.0)
		{
			float s = 1.0*sqrtf(tilt * tilt);			
			GaussianBlur(ttransImg, ttransImg, cvSize(0, 0), s, 0.01);
			resize(ttransImg, ttransImg, cvSize(0, 0), 1.0, 1.0/tilt, INTER_NEAREST);
			A.row(1) /= tilt;
		}
		Mat ttransPts = A * pts;
		Mat dst, dstPts, preDst = ttransImg, prePts = ttransPts;
		for (int j = 0; j < noctave; j++)
		{
			for (int k = 0; k < noctaveLayer; k++)
			{	
				if( j == 0  &&  k == 0 )
				{
					dst = preDst;			
					dstPts = prePts;
				}
				else if( k == 0 )
				{					
					resize(preDst, dst, Size(preDst.cols*downSampleRatio, preDst.rows*downSampleRatio), 0, 0, INTER_NEAREST);
					dstPts = prePts * downSampleRatio;
					preDst = dst;
					prePts = dstPts;
				}
				else
				{	
					float s = 1.6;		
					GaussianBlur(preDst, dst, Size(), s, s);
					preDst = dst;
					dstPts = prePts;
				}
				dstPts = dstPts.t();				
				Mat transDes, tGMag, tGDir;
				calcGrad(dst, tGMag, tGDir);						
				description_fanPts(dstPts, tGMag, tGDir, transDes);				
				vDes.push_back(transDes);
			}
		}
	}
}


void CLineMatching::description_fanPts(Mat pts,  Mat gMag,  Mat gDir, Mat &descriptors)
{
	Mat mDes; 
	int nfan = pts.rows;	
	for (int i = 0; i < nfan; i += 3)
	{		
		mDes.release();		
		float *pdat1 = pts.ptr<float>(i);
		float *pdat2 = pts.ptr<float>(i+1);
		float *pdat3 = pts.ptr<float>(i+2);

		float dx = pdat2[0] - pdat1[0];
		float dy = pdat2[1] - pdat1[1];
		float ang1 = fast_mod(atan2(dy, dx));
		dx = pdat3[0] - pdat1[0];
		dy = pdat3[1] - pdat1[1];
		float ang2 = fast_mod(atan2(dy, dx));
		float angs[2] = { ang1, ang2 };
		Vec4f pt(pdat1[0], pdat1[1], ang1, ang2);
		//pt[0] = pdat1[0];
		//pt[1] = pdat1[1];
		//for (int j = 0; j < 2; j++)
		//{									
		//	pt[2] = angs[j];
			Mat des;
			description_singleFan(pt, gMag, gDir, mDes);
			//mDes.push_back(des);					
	//	}
		descriptors.push_back(mDes);
		//descriptors.push_back(mDes.reshape(1,1));
	}
}


void CLineMatching::matchAffineSequence(Mat oriImg, vector<strFanSection> oriFan, Mat oriDes, Mat oriPts, Mat transImg, vector<strFanSection> transFan, Mat transPts,
																   vector<strFanMatch> &maxvMatches, Mat &maxGMag, Mat &maxGDir, vector<strFanSection> &vMaxFan, Mat &maxTransMat)
{			
	int nfan = transPts.rows / 3;	
	int rows = transImg.rows, cols = transImg.cols;
	Mat A = (Mat_<float>(2, 3)<<1, 0, 0, 0, 1, 0);		
	int maxMatches = 0;
	Mat maxDes, maxTransPts;
	vector<strFanMatch> tvFanMatches;
	strFanSection tstrFan;
	transPts = transPts.t();
	Mat mones = Mat::ones(1, transPts.cols, CV_32F);
	transPts.push_back(mones);
	
	int noctave = 3;
	int noctaveLayer = 2;
	vector<Vec2f> vparam;
	Vec2f tvec;
	for (int i = 1; i < 6; i++)
	{
		float  tilt = powf(2.0, 0.5 * i);
		float phi = 0;
		while(phi <  180)
		{
			tvec(0) = tilt;
			tvec(1) = phi;
			vparam.push_back(tvec);
			phi += 72.0 / tilt;			
		}
	}

	int nparams = vparam.size();
	for (int i = 0; i < nparams; i++)
	{
		float tilt =  vparam[i].val[0];
		float phi = vparam[i].val[1];		
		Mat ttransImg = transImg;
		if (phi != 0)
		{
			phi = phi * CV_PI / 180;
			float sinPhi = sin(phi), cosPhi = cos(phi);
			A = (Mat_<float>(2, 2) << cosPhi, -sinPhi, sinPhi, cosPhi);					
			vector<Point2f> vCor;
			Mat corners = (Mat_<float>(4, 2) << 0, 0, cols, 0, cols, rows, 0, rows);				
			Mat tcorners = corners * A.t();
			tcorners = tcorners.reshape(2);		
			Rect bRect = boundingRect(tcorners);				
			Mat tmpB = (Mat_<float>(1,2) << -bRect.x, -bRect.y);
			A = A.t();
			A.push_back(tmpB);				
			A = A.t();						
			warpAffine(ttransImg, ttransImg, A, cvSize(bRect.width, bRect.height), CV_INTER_LINEAR, BORDER_REPLICATE);			
		}
		if (tilt != 0)
		{
			float s = 0.8*sqrtf(tilt * tilt-1);			
			GaussianBlur(ttransImg, ttransImg, cvSize(0, 0), s, 0.01);
			resize(ttransImg, ttransImg, cvSize(0, 0), 1.0, 1.0/tilt, INTER_NEAREST);
			A.row(1) /= tilt;
		}
		Mat ttransPts = A * transPts;
		Mat dst;
		for (int j = 0; j < noctave; j++)
		{
			for (int k = 0; k < noctaveLayer; k++)
			{
				if( j == 0  &&  k == 0 )
				{
					dst = ttransImg;
				}
				else if( j == 0 )
				{					
					resize(ttransImg, dst, Size(ttransImg.cols*0.5, ttransImg.rows*0.5), 0, 0, INTER_NEAREST);
					ttransPts /= 2;
				}
				else
				{	
					float s = 0.8*sqrtf(tilt * tilt-1);		
					GaussianBlur(ttransImg, dst, Size(), s, s);
				}
				ttransPts = ttransPts.t();
				tvFanMatches.clear();
				Mat transDes, tGMag, tGDir;
				calcGrad(dst, tGMag, tGDir);						
				description(ttransPts, tGMag, tGDir, transDes);
			}
		}
	/*	
		stringstream ss;
		string str;
		ss<<i<<".jpg";
		ss>>str;
		Mat drawttransPts = ttransPts.t();
		int npts = ttransPts.cols/3;
		for (int j = 0; j < npts; j++)
		{			
			float *pdat1 = drawttransPts.ptr<float>(j);
			Point2f pt1 = Point2f(pdat1[0], pdat1[1]);
			float *pdat2 = drawttransPts.ptr<float>(j+npts);
			Point2f pt2 = Point2f(pdat2[0], pdat2[1]);
			float *pdat3 = drawttransPts.ptr<float>(j+2*npts);
			Point2f pt3 = Point2f(pdat3[0], pdat3[1]);
			circle(ttransImg, pt1, 5, cvScalar(0, 0, 0), -1);
		}
		imwrite(str, ttransImg); 	
		imshow(str, ttransImg);
		waitKey();

 		ttransPts = ttransPts.t();
		tvFanMatches.clear();
		Mat transDes, tGMag, tGDir;
		calcGrad(ttransImg, tGMag, tGDir);						
		description(ttransPts, tGMag, tGDir, transDes);
*/
// 		int ntm = descriptorsMatching(oriPts, ttransPts.rowRange(0, nfan), oriDes, transDes, 0.3, tvFanMatches);
// 		if (ntm > maxMatches)
// 		{
// 			maxvMatches = tvFanMatches;		
// 			maxTransMat = A;				
// 			maxGMag = tGMag;
// 			maxGDir = tGDir;
// 			maxMatches = ntm;
// 			maxTransPts = ttransPts;			
// 			maxDes = transDes;
// 		}
// 		
	}
/*
	for (int i = 1; i < 6; i++)
	{
		float  tilt = powf(2.0, 0.5 * i);
		float  phi = 0;
		
		phi = phi * CV_PI / 180;
		float sinPhi = sin(phi), cosPhi = cos(phi);
		Mat A = (Mat_<float>(2, 2) << -cosPhi, sinPhi, sinPhi, cosPhi);					
		vector<Point2f> vCor;
		Mat corners = (Mat_<float>(4, 2) << 0, 0, cols, 0, cols, rows, 0, rows);				
		Mat tcorners = corners * A.t();
		tcorners = tcorners.reshape(2);		
		Rect bRect = boundingRect(tcorners);				
		Mat tmpB = (Mat_<float>(1,2) << -bRect.x, -bRect.y);
		A = A.t();
		A.push_back(tmpB);				
		A = A.t();
		Mat ttransImg;
		warpAffine(transImg, ttransImg, A, cvSize(cols, rows), CV_INTER_LINEAR, BORDER_REPLICATE);			

		imshow("1.jpg", ttransImg);
		waitKey();

		Mat ttransPts = A * transPts;

		ttransPts.push_back(mones);
		while(phi <  180)
		{						
			float s = 0.8*sqrtf(tilt * tilt-1);
			Mat tttransImg;
			GaussianBlur(ttransImg, tttransImg, cvSize(0, 0), s, 0.01);
			resize(tttransImg, tttransImg, cvSize(0, 0), 1.0/tilt, 1.0, INTER_NEAREST);

			imshow("2.jpg", tttransImg);
			waitKey();

			A.row(0) /= tilt;
			Mat tttransPts = A * ttransPts;
				
			tttransPts = tttransPts.t();
			tvFanMatches.clear();
			Mat transDes, tGMag, tGDir;
			calcGrad(tttransImg, tGMag, tGDir);						
			description(tttransPts, tGMag, tGDir, transDes);
/*
			imshow("tGMag2.jpg", tGMag2);
			
 			Mat showImg1, showImg2;			
			double minVal, maxVal;
			minMaxLoc(oriImg, &minVal, &maxVal);
			//cout<<oriImg.row(0);
			showImg1 =  (oriImg - minVal) * 255 / maxVal;
		//	cout<<showImg1.row(0);
			showImg1.convertTo(showImg1, CV_8U);
		//	cout<<showImg1.row(0);
				
			minMaxLoc(tImg2, &minVal, &maxVal);
			showImg2 =  (tImg2 - minVal) * 255 / maxVal;
			showImg2.convertTo(showImg2, CV_8U);
				
			showImg1 = oriImg;
			showImg2 = tImg2;
			int nline1 = vOriFan.size();
			for (int i = 0; i < nline1; i++)
			{	
				Point2f intersection = vOriFan[i].intsection;
				Point2f branch1      = vOriFan[i].strbranch1.bpt;
				Point2f branch2      = vOriFan[i].strbranch2.bpt;
				circle(showImg1, intersection, 5, cvScalar(255, 255, 255), -1);
				line(showImg1, intersection, branch1, cvScalar(255,255,255), 2);		
				line(showImg1, intersection, branch2, cvScalar(255,255,255), 2);		
			}
			imshow("showImg1.jpg", showImg1);	
			waitKey();	

			int nline2 = tvPyrFans.size();
			for (int i = 0; i < nline2; i++)
			{	
				Point2f intersection = tvPyrFans[i].intsection;
				Point2f branch1      = tvPyrFans[i].strbranch1.bpt;
				Point2f branch2      = tvPyrFans[i].strbranch2.bpt;
				circle(showImg2, intersection, 5, cvScalar(255, 255, 255), -1);
				line(showImg2, intersection, branch1, cvScalar(255,255,255), 2);		
				line(showImg2, intersection, branch2, cvScalar(255,255,255), 2);		
			}
			imshow("showImg2.jpg", showImg2);					
			waitKey();	
						

		
			if (i == 1 && j == 0)
			{
 				Mat showImg1, showImg2;
				/*
				double minVal, maxVal;
				minMaxLoc(oriImg, &minVal, &maxVal);
				cout<<oriImg.row(0);
				showImg1 =  (oriImg - minVal) * 255 / maxVal;
				cout<<showImg1.row(0);
				showImg1.convertTo(showImg1, CV_8U);
				cout<<showImg1.row(0);
				
				minMaxLoc(tImg2, &minVal, &maxVal);
				showImg2 =  (tImg2 - minVal) * 255 / maxVal;
				showImg2.convertTo(showImg2, CV_8U);
				
				showImg1 = oriImg;
				showImg2 = tImg2;
				int nline1 = vOriFan.size();
				for (int i = 0; i < nline1; i++)
				{	
					Point2f intersection = vOriFan[i].intsection;
					Point2f branch1      = vOriFan[i].strbranch1.bpt;
					Point2f branch2      = vOriFan[i].strbranch2.bpt;
					circle(showImg1, intersection, 5, cvScalar(255, 255, 255), -1);
					line(showImg1, intersection, branch1, cvScalar(255,255,255), 2);		
					line(showImg1, intersection, branch2, cvScalar(255,255,255), 2);		
				}
				imshow("showImg1.jpg", showImg1);	
				waitKey();	

				int nline2 = tvPyrFans.size();
				for (int i = 0; i < nline2; i++)
				{	
					Point2f intersection = tvPyrFans[i].intsection;
					Point2f branch1      = tvPyrFans[i].strbranch1.bpt;
					Point2f branch2      = tvPyrFans[i].strbranch2.bpt;
					circle(showImg2, intersection, 5, cvScalar(255, 255, 255), -1);
					line(showImg2, intersection, branch1, cvScalar(255,255,255), 2);		
					line(showImg2, intersection, branch2, cvScalar(255,255,255), 2);		
				}
				imshow("showImg2.jpg", showImg2);					
				waitKey();	
			}
		

			int ntm = descriptorsMatching(oriPts, tttransPts.rowRange(0, nfan), oriDes, transDes, 0.3, tvFanMatches);
			if (ntm > maxMatches)
			{
				maxvMatches = tvFanMatches;		
				maxTransMat = A;				
				maxGMag = tGMag;
				maxGDir = tGDir;
				maxMatches = ntm;
				maxTransPts = tttransPts;			
				maxDes = transDes;
			}								
		}
	}	
*/
	maxTransPts = maxTransPts.rowRange(0, 2).clone().t();

	vMaxFan = transFan;
	for (int j = 0; j < nfan; j++)
	{					
		float *pdat1 = maxTransPts.ptr<float>(j);
		float *pdat2 = maxTransPts.ptr<float>(j+nfan);
		float *pdat3 = maxTransPts.ptr<float>(j+2*nfan);
		vMaxFan[j].mDes = maxDes.row(j).clone();
		vMaxFan[j].intsection = Point2f(pdat1[0], pdat1[1]);
		vMaxFan[j].strbranch1.bpt = Point2f(pdat2[0], pdat2[1]);
		vMaxFan[j].strbranch2.bpt = Point2f(pdat3[0], pdat3[1]);		
	}	
}

void CLineMatching::affineTransformEstimation(Mat &img1, Mat &node1, strLine* strline1, vector<strFanSection> &vStrFanSect1, Mat &img2, Mat &node2,
																		  strLine* strline2, vector<strFanSection> &vStrFanSect2, vector<strFanMatch> &vFanMatch)
{
	vector<Mat> vImgPyr1, vImgPyr2, vEndPts1, vEndPts2;	
	float desDistThr = _desDistThrEpi;
//	desDistThr = 0.4;

	int nv1 = vStrFanSect1.size();
	//Mat pts1, pts2, combPts1, combPts2;
	Mat combPts1, combPts2;
	Mat ttmat;	
	for (int i = 0; i < nv1; i++)
	{				
		ttmat = (Mat_<float>(1, 2)<< vStrFanSect1[i].intsection.x, vStrFanSect1[i].intsection.y);
		combPts1.push_back(ttmat);
		//pts1.push_back(ttmat);
		ttmat = (Mat_<float>(1, 2)<< vStrFanSect1[i].strbranch1.bpt.x, vStrFanSect1[i].strbranch1.bpt.y);
		combPts1.push_back(ttmat);
		ttmat = (Mat_<float>(1, 2)<< vStrFanSect1[i].strbranch2.bpt.x, vStrFanSect1[i].strbranch2.bpt.y);		
		combPts1.push_back(ttmat);
	}		

	int nv2 = vStrFanSect2.size();
	for (int i = 0; i < nv2; i++)
	{		
		ttmat = (Mat_<float>(1, 2)<<vStrFanSect2[i].intsection.x, vStrFanSect2[i].intsection.y);
		combPts2.push_back(ttmat);		
		ttmat = (Mat_<float>(1, 2)<<vStrFanSect2[i].strbranch1.bpt.x, vStrFanSect2[i].strbranch1.bpt.y);
		combPts2.push_back(ttmat);
        ttmat = (Mat_<float>(1, 2)<<vStrFanSect2[i].strbranch2.bpt.x, vStrFanSect2[i].strbranch2.bpt.y);
		combPts2.push_back(ttmat);
	}	
	
	vector<Mat> vDes1, vDes2;
 	get_asift_description(img1, combPts1, vDes1);
	get_asift_description(img2, combPts2, vDes2);

	Mat des1, des2;
	int nLevel = vDes1.size();
	for (int i = 0; i < nLevel; i++)
	{
		des1.push_back(vDes1[i]);
		des2.push_back(vDes2[i]);
	}

	Mat tdes;
	for (int i = 0; i < nv1; i++)
	{	
		tdes.release();
		for (int j = 0; j < nLevel; j++)
		{			
			tdes.push_back(des1.row(j*nv1+i));
		}
		vStrFanSect1[i].mDes = tdes;
	}
	for (int i = 0; i < nv2; i++)
	{	
		tdes.release();
		for (int j = 0; j < nLevel; j++)
		{
			tdes.push_back(des2.row(j*nv2+i));
		}
		vStrFanSect2[i].mDes = tdes;
	}

	Mat curDes1, curDes2;
	int ncand = _nAvgDescriptorDistance;
	strFanMatch tstrFanMatch;
	vector<float> vDist;
	for (int i = 0; i < nv1; i++)
	{
		curDes1 = vStrFanSect1[i].mDes;
		float minDist1 = FLT_MAX;
		int ser = 0;
		float difAng1 = fmod(2*CV_PI+vStrFanSect1[i].strbranch2.ang - vStrFanSect1[i].strbranch1.ang, 2*CV_PI);
		for (int j = 0; j < nv2; j++)
		{
			curDes2 = vStrFanSect2[j].mDes;
			float difAng2 = fmod(2*CV_PI+vStrFanSect2[j].strbranch2.ang - vStrFanSect2[j].strbranch1.ang, 2*CV_PI);

 			if (abs(difAng1- difAng2) > _difAngThr)			
 				continue;
			
		//	float tminDist1 = FLT_MAX;
			vDist.clear();
			for (int m = 0; m < nLevel; m++)
			{
				Mat currow1 = curDes1.row(m);				
				for (int n = 0; n < nLevel; n++)
				{
					Mat currow2 = curDes2.row(n);
					float tdist = norm(currow1-currow2);					
					vDist.push_back(tdist);
					/*
					if (tdist < tminDist1)
					{
						tminDist1 = tdist;
					}
					*/
				}
			}			
			cv::sort(vDist, vDist, CV_SORT_EVERY_ROW);	
			float tdist = 0;
			for (int m = 0; m < ncand; m++)
			{
				tdist += vDist[m] * vDist[m];
			}
			tdist = tdist / ncand;

			if (tdist < minDist1)
			{
				minDist1 = tdist;
				ser = j;
			}
		}		
		if (minDist1 < desDistThr)
		{
			tstrFanMatch.fserial1 = i;
			tstrFanMatch.fserial2 = ser;
			tstrFanMatch.dist = minDist1;
			vFanMatch.push_back(tstrFanMatch);
		}		
	}
/*
	Mat tpts1, tpts2;
	int nmatch = m_knnMatches.size();	
	strFanMatch tstrFanMatch;
	for (int i = 0; i < nv1; i++)  
	{  				
		float dist = FLT_MAX, distance;
		DMatch& tmatch = DMatch();
		for (int  j = 0; j < nLevel; j++)
		{	
			const DMatch& bestMatch = m_knnMatches[j*nv1+i][0];  			
			distance = bestMatch.distance;
			if(distance < dist)
			{
				dist = distance;
				tmatch = bestMatch;
			}			
		}
		distance *= distance;
		if (distance < desDistThr)
		{			
			int ser = tmatch.trainIdx % nv2;		
			tstrFanMatch.fserial1 = i;
			tstrFanMatch.fserial2 = ser;
			tstrFanMatch.dist = distance;
			vFanMatch.push_back(tstrFanMatch);
		}	
	} 	
*/

/*
	Mat des1, des2;
	description(vStrFanSect1, gmag1, gdir1, 1, des1);	
	description(vStrFanSect2, gmag2, gdir2, 1, des2);
	vector<Vec3f> vMatch;
	int nOriMatch = descriptorsMatching(pts1, pts2, des1, des2, 0.3, vFanMatch);

	vector<strFanMatch> vMaxMatches1, vMaxMatches2;
	vector<strFanSection> maxFanSect1, maxFanSect2; 	
	Mat maxGMag1, maxGDir1, maxGMag2, maxGDir2, maxTransPts1, maxTransPts2, maxTransMat1, maxTransMat2;
	matchAffineSequence(img1, vStrFanSect1, des1, pts1, img2, vStrFanSect2, combPts2, vMaxMatches1, maxGMag1, maxGDir1, maxFanSect1, maxTransMat1);
	matchAffineSequence(img2, vStrFanSect2, des2, pts2, img1, vStrFanSect1, combPts1, vMaxMatches2, maxGMag2, maxGDir2, maxFanSect2, maxTransMat2);

	int num2 = vMaxMatches1.size();
	int num3 = vMaxMatches2.size();

	if (num2 > max(nOriMatch, num3))
	{		
		for (int j = 0; j < nline2; j++)
		{
			Mat tmat = (Mat_<float>(3, 2)<<strline2[j].ps.x, strline2[j].pe.x,
																 strline2[j].ps.y, strline2[j].pe.y,
																 1, 1);
			tmat	 = tmat * maxTransMat1;
			float *pdat = tmat.ptr<float>(0);
			strline2[j].pe.x = pdat[0];
			strline2[j].pe.y = pdat[2];
			strline2[j].ps.x = pdat[1];
			strline2[j].ps.y = pdat[3];				
		}				
	    warpAffine(img2, img2, maxTransMat1, cvSize(0, 0), CV_INTER_LINEAR, BORDER_REPLICATE);	
		warpAffine(colorImg2, colorImg2, maxTransMat1, cvSize(0, 0), CV_INTER_LINEAR, BORDER_REPLICATE);	

 		vFanMatch = vMaxMatches1;
 		gdir2 = maxGDir1;
 		gmag2 = maxGMag1;
 		vStrFanSect2 = maxFanSect1;
	}
	else if(num3 > max(nOriMatch, num2))
	{
		for (int j = 0; j < nline2; j++)
		{
			Mat tmat = (Mat_<float>(3, 2)<<strline1[j].ps.x, strline1[j].pe.x,
				strline1[j].ps.y, strline1[j].pe.y,
				1, 1);
			tmat	 = tmat * maxTransMat1;
			float *pdat = tmat.ptr<float>(0);
			strline1[j].pe.x = pdat[0];
			strline1[j].pe.y = pdat[2];
			strline1[j].ps.x = pdat[1];
			strline1[j].ps.y = pdat[3];				
		}		

		warpAffine(img1, img1, maxTransMat2, cvSize(0, 0), CV_INTER_LINEAR, BORDER_REPLICATE);	
		warpAffine(colorImg1, colorImg1, maxTransMat2, cvSize(0, 0), CV_INTER_LINEAR, BORDER_REPLICATE);	

		int nfan = vMaxMatches2.size();	
		vFanMatch = vMaxMatches2;
		for(int j = 0;  j < nfan ; j++)
		{
			vFanMatch[j].fserial1 = vMaxMatches2[j].fserial2;
			vFanMatch[j].fserial2 = vMaxMatches2[j].fserial1;
		}				
 		gdir1 = maxGDir2;
 		gmag1 = maxGMag2;
 		vStrFanSect1 = maxFanSect2;
	}	
*/
}

float CLineMatching::descriptorDistance(Mat des1, Mat des2, int ncand)
{		
	float minDist1 = FLT_MAX;
	int ser = 0;
	int rows = des1.rows;	
	vector<float> vDist;
	for (int m = 0; m < rows; m++)
	{
		Mat currow1 = des1.row(m);				
		for (int n = 0; n < rows; n++)
		{
			Mat currow2 = des2.row(n);
			float tdist = norm(currow1-currow2);					
			vDist.push_back(tdist);					
		}
	}			
	cv::sort(vDist, vDist, CV_SORT_EVERY_ROW);	
	float tdist = 0;
	for (int m = 0; m < ncand; m++)
	{
		tdist += vDist[m] * vDist[m];
	}
	tdist = tdist / ncand;
	return tdist;
}

int CLineMatching::descriptorsMatching(Mat pts1, Mat pts2, Mat des1, Mat des2, float distThr, vector<strFanMatch> &vFanMatch)
{
	FlannBasedMatcher matcher;  	
	vector<vector<DMatch> > m_knnMatches;  	
	matcher.knnMatch(des1, des2, m_knnMatches, 1);  
	Mat tpts1, tpts2;
	int nmatch = m_knnMatches.size();
	strFanMatch tstrFanMatch;
	vFanMatch.clear();
	for (int i= 0; i<nmatch; i++)  
	{  		
		const DMatch& bestMatch = m_knnMatches[i][0];  
		float distance = bestMatch.distance;
		distance *= distance;
		if (distance < distThr)
		{
			tpts1.push_back(pts1.row(bestMatch.queryIdx));
			tpts2.push_back(pts2.row(bestMatch.trainIdx));
			tstrFanMatch.fserial1 = bestMatch.queryIdx;
			tstrFanMatch.fserial2 = bestMatch.trainIdx;
			tstrFanMatch.dist = distance;
			vFanMatch.push_back(tstrFanMatch);
		}	
	} 	
	int ninlier = 0;
	/*
	if (_isScaleChangeExisting)
	{
		if (tpts1.rows < 8)
		{
			return tpts1.rows;
		}
		vector<uchar> inliers;
		Mat fmat = findFundamentalMat(tpts1, tpts2, CV_RANSAC, _fmatThr, 0.99, inliers);
		ninlier = countNonZero(inliers);
	}
*/
	return ninlier;
}

void CLineMatching::fanMatch(vector<strFanSection> v1, vector<strFanSection> v2, float distThr, vector<strFanMatch> &vFanMatch)
{	
	int rows1  = v1.size();
	int rows2  = v2.size();
	strFanMatch tstrFanMatch;
	for (int i = 0; i < rows1; i++)
	{
		float difAng1 = fmod(v1[i].strbranch2.ang - v1[i].strbranch1.ang+2*CV_PI, 2*CV_PI);
		Mat curDes1 = v1[i].mDes;
		float minDist = FLT_MAX;
		int bestj = 0;
		for (int j = 0; j < rows2; j++)
		{						
			float difAng2 = fmod( v2[j].strbranch2.ang - v2[j].strbranch1.ang+2*CV_PI, 2*CV_PI);			
 			if (abs(difAng1- difAng2) > _difAngThr)			
 				continue;

			Mat curDes2 = v2[j].mDes;
			float tdist = norm((curDes1 - curDes2), NORM_L2);
			if (tdist < minDist )
			{
				minDist = tdist;			
				bestj = j;
			}				
		}
		float dminDist = minDist * minDist;
		if (dminDist < distThr)
		{
			tstrFanMatch.fserial1 = i;
			tstrFanMatch.fserial2 = bestj;
			tstrFanMatch.dist = dminDist;
			vFanMatch.push_back(tstrFanMatch);			
		}
	}	
}

void CLineMatching::description(vector<strFanSection> &vstrFanSection,  Mat gMag,  Mat gDir)
{
	Mat mDes; 
	// desMat.release();
	int nfan = vstrFanSection.size();
	for (int i = 0; i < nfan; i++)
	{
		mDes.release();
		float angs[2] = { vstrFanSection[i].strbranch1.ang, vstrFanSection[i].strbranch2.ang };
		Vec3f pt;
		pt[0] = vstrFanSection[i].intsection.x;
		pt[1] = vstrFanSection[i].intsection.y;
		for (int j = 0; j < 2; j++)
		{									
			pt[2] = angs[j];
			Mat des;
			describeSingleLine(pt, gMag, gDir, des);
			mDes.push_back(des.clone());					
		}
		vstrFanSection[i].mDes = mDes.reshape(1, 1).clone();				
	}
}

void CLineMatching::description(vector<strFanSection> &vstrFanSection,  Mat gMag,  Mat gDir, bool isOutputDesMat, Mat & desMat)
{
	Mat mDes; 
	desMat.release();
	int nfan = vstrFanSection.size();
	for (int i = 0; i < nfan; i++)
	{
		mDes.release();
		float angs[2] = { vstrFanSection[i].strbranch1.ang, vstrFanSection[i].strbranch2.ang };
		Vec3f pt;
		pt[0] = vstrFanSection[i].intsection.x;
		pt[1] = vstrFanSection[i].intsection.y;
		for (int j = 0; j < 2; j++)
		{									
			pt[2] = angs[j];
			Mat des;
			describeSingleLine(pt, gMag, gDir, des);
			mDes.push_back(des.clone());					
		}
		vstrFanSection[i].mDes = mDes.reshape(1, 1).clone();		
		if (isOutputDesMat)
		{
			desMat.push_back(mDes.reshape(1, 1).clone());
		}
	}
}

void CLineMatching::description_singleFan(Vec4f pt,  Mat gMag,  Mat gDir, Mat &des)
{
	int ncircle = 2;
	int rcircle = _rcircle;
	int nbo = 8;
	int mcircle = ncircle * rcircle;	
	float difAng = fmod(pt[3] - pt[2]+2*CV_PI, 2*CV_PI);
//cout<<pt[2]<<","<<pt[3]<<endl;
	float subdifAng = CV_PI - difAng;
	float ang = pt[2];
	float tsin = sin(ang);
	float tcos = cos(ang);

	int win = (int) mcircle * 1.414;
	int rows = gMag.rows;
	int cols = gMag.cols;
	Mat tdes = Mat::zeros(1, 192, CV_32F);
	float *pdat = tdes.ptr<float>(0);

	float fptx = pt[0];
	float fpty = pt[1];
	int iptx = (int) fptx;
	int ipty = (int) fpty;
	int bxlim = max(-win, 1-iptx);
	int uxlim = min(win, cols-2-iptx);
	int bylim = max(-win, 1-ipty);
	int uylim = min(win, rows-2-ipty);

	for (int i = bxlim; i < uxlim; i++)
	{
		int serx = iptx+i;
		for (int j = bylim; j < uylim; j++)
		{
			float dist = sqrtf(i*i+j*j);
			float binr = dist / rcircle;
			if( binr >=  ncircle) continue;
			float wei = (binr < 1.5 && binr > 1) ? 0.33333 : 1.0;			
			int ibinr = (int) (binr - 0.5);
			float rbinr = binr - ibinr - 0.5;			
		
			int sery = ipty+j;
			float mag = gMag.at<float>(sery, serx);
			float dir    = gDir.at<float>(sery, serx);
			float theta = fast_mod(-dir+pt[2]);
			float bint = nbo * theta / (2*CV_PI);

			int ibint = cvFloor(bint);
			float rbint = bint - ibint;
// 			float roti = tcos * j - tsin * i + pt[0];
// 			float rotj = tsin * j + tcos * i +pt[1];
// 			int posx = cvRound(roti);
// 			int posy = cvRound(rotj);
// 			if (posx < 0)  posx = 0;
// 			if (posy < 0)  posy = 0;
// 			if (posx >= cols)   posx = cols - 1;
// 			if (posy >= rows)  posy = rows - 1;	
			
			float orit = fastAtan2(j, i);
			orit = orit * CV_PI / 180;
			//fast_mod(-gDir.at<float>(posy, posx)+pt[2])
			float dang = fmod(orit - pt[2]+2*CV_PI, 2*CV_PI);
			int flag = 0;
			float coorAng = 0;
// 			if (dang < 0) 
// 			{
// 				dang += 2 * CV_PI;
// 			}
			if (dang < difAng)
			{
				flag = 0;
				coorAng = 0;
			}
			else if (dang >= difAng && dang < CV_PI)
			{
				flag = 1;
				coorAng = difAng;
			}
			else if (dang >= CV_PI && dang < difAng + CV_PI)
			{
				flag = 2;
				coorAng = CV_PI;
			}
			else
			{
				flag = 3;
				coorAng = difAng + CV_PI;
			}
			float bina = 3.0 * flag;			
			if (flag == 0 || flag == 2)
			{
				bina += 3 * (dang - coorAng) / difAng;
			} 
			else
			{
				bina += 3 * (dang - coorAng) / subdifAng;
			}

			int    ibina = (int) bina;
			float rbina = bina - ibina;
			float wincoef = exp(-dist*dist/(2*mcircle*mcircle));

			for (int dbinr = 0; dbinr < 2; dbinr++)
			{
				for (int dbina = 0; dbina < 2; dbina++ )
				{
					for (int dbint = 0; dbint < 2; dbint++)
					{		
						if (ibinr+dbinr >= 0 && ibinr+dbinr < ncircle)
						{
							float wmag = wei * wincoef * mag * abs(1 - dbinr - rbinr) * abs(1 - dbina - rbina) * abs(1-dbint-rbint);                 							
							//float wmag = wei * wincoef * mag * abs(1 - dbina - rbina) * abs(1-dbint-rbint);                 							
							int obin = (ibint+dbint) % nbo;
							int rbin = ibinr + dbinr;							
							//int rbin = ibinr;							
							int abin = (ibina + dbina) % 12;
							int ser = rbin*96+abin*8+obin;
							pdat[ser] = pdat[ser] + wmag;	
						}														
					}			
				}	
			}
		 }		
	}

// 	double desNorm = norm(tdes, NORM_L2);
// 	tdes = tdes / desNorm;
// 	threshold(tdes, tdes, 0.2, 0.2, THRESH_TRUNC);
// 	desNorm = norm(tdes, NORM_L2);
// 	tdes = tdes / desNorm;
// 	des = tdes;

	//cout<<tdes<<endl;
	Mat retdes = tdes.reshape(1, 24);
	//cout<<retdes<<endl;
	Mat tmat;// = retdes;// = Mat::zeros(16, 8, CV_32F);
	/*
	Mat matc;
	matc.push_back(tmat.rowRange(0, 3));
	matc.push_back(tmat.rowRange(12, 15));
	matc.push_back(tmat.rowRange(6, 9));
	matc.push_back(tmat.rowRange(18, 21));
	Mat matd;
	matd.push_back(tmat.rowRange(3, 6));
	matd.push_back(tmat.rowRange(15, 18));
	matd.push_back(tmat.rowRange(9, 12));
	matd.push_back(tmat.rowRange(21, 24));
	*/
	//cout<<retdes<<endl;
		
	Mat mata = retdes.rowRange(0, 12);
	for (int i = 0; i < 4; i++)
	{
		Mat matb = mata.row(i*3+0) + mata.row(i*3+1) + mata.row(i*3+2);
		tmat.push_back(matb);
	}
	tmat.push_back(retdes.rowRange(12, 24));

	Mat matc;
	matc.push_back(tmat.row(0));
	matc.push_back(tmat.rowRange(4, 7));
	matc.push_back(tmat.row(2));
	matc.push_back(tmat.rowRange(10, 13));
	Mat matd;
	matd.push_back(tmat.row(1));
	matd.push_back(tmat.rowRange(7, 10));
	matd.push_back(tmat.row(3));
	matd.push_back(tmat.rowRange(13, 16));
	
	double desNorm = norm(matc, NORM_L2);
	matc = matc / desNorm;
	threshold(matc, matc, _truncateThr, _truncateThr, THRESH_TRUNC);
	
//	cout<<matc<<endl;

	desNorm = norm(matc, NORM_L2);
	matc = matc / desNorm;

//	cout<<matc<<endl;

	
	desNorm = norm(matd, NORM_L2);
	matd = matd / desNorm;	
	threshold(matd, matd, _truncateThr, _truncateThr, THRESH_TRUNC);	
	desNorm = norm(matd, NORM_L2);
	matd = matd / desNorm;

	des.push_back(matc);
	des.push_back(matd);

	//cout<<des<<endl;

	des = des.reshape(1, 1);

//	cout<<des<<endl;
	//cout<<des<<endl;
}

void CLineMatching::description_fans(vector<strFanSection> &vstrFanSection,  Mat gMag,  Mat gDir, bool isOutputDesMat, Mat &desMat)
{
	desMat.release(); 
	int nfan = vstrFanSection.size();

	for(int i = 0; i < nfan; i++)
	{
		strFanSection curFan = vstrFanSection[i];
		Vec4f pt(curFan.intsection.x, curFan.intsection.y, curFan.strbranch1.ang, curFan.strbranch2.ang);
		Mat mDes;
		description_singleFan(pt, gMag, gDir, mDes);		
		vstrFanSection[i].mDes = mDes;			
	}

	if (isOutputDesMat)
	{
		for (int i = 0; i < nfan; i++)
		{
			desMat.push_back(vstrFanSection[i].mDes);
		}		
	}
}



void CLineMatching::description_fans(vector<strFanSection> &vstrFanSection,  Mat gMag,  Mat gDir)
{
	// desMat.release(); 
	int nfan = vstrFanSection.size();

	for(int i = 0; i < nfan; i++)
	{
		strFanSection curFan = vstrFanSection[i];
		Vec4f pt(curFan.intsection.x, curFan.intsection.y, curFan.strbranch1.ang, curFan.strbranch2.ang);
		Mat mDes;
		description_singleFan(pt, gMag, gDir, mDes);		
		vstrFanSection[i].mDes = mDes;			
	}
	// if (isOutputDesMat)
	// {
	// 	for (int i = 0; i < nfan; i++)
	// 	{
	// 		desMat.push_back(vstrFanSection[i].mDes);
	// 	}		
	// }
}



void CLineMatching::description_sift(vector<strFanSection> &vstrFanSection,  Mat gMag,  Mat gDir)
{	
	//desMat.release();
	int nfan = vstrFanSection.size();
	for (int i = 0; i < nfan; i++)
	{		
		float angs[2] = {vstrFanSection[i].strbranch1.ang, vstrFanSection[i].strbranch2.ang };
		Vec3f pt;
		pt[0] = vstrFanSection[i].intsection.x;
		pt[1] = vstrFanSection[i].intsection.y;
		Mat mDes; 
		for (int j = 0; j < 2; j++)
		{									
			pt[2] = angs[j];
			Mat des;
			description_sift_single(pt, gMag, gDir, des);
	//		cout<<des<<endl;
			mDes.push_back(des);					
		}
		vstrFanSection[i].mDes = mDes.reshape(1, 1);			
	//	cout<<i<<",";
	}
}

void CLineMatching::description_sift_single(Vec3f pt, const Mat gMag, const Mat gDir, Mat &des)
{
	int nbo = 8;
	int nbp = 4;
	int sbp = 4;
	int hnbp = 2;

	int rows = gMag.rows;
	int cols = gMag.cols;

	des = Mat::zeros(1, 128, CV_32F);	
	float *pdat = des.ptr<float>(0);

	int xp = int(pt(0) + 0.5);
	int yp = int(pt(1) + 0.5);

	float theta0 = pt(2);
	float sinth0 = sin(theta0) ;
	float costh0 = cos(theta0) ;
	float sigma = 1.6;
	float SBP = 3 * sigma;
	int W =  int( 0.8 * SBP * (nbp + 1) / 2.0 + 0.5);

	//descriptor = zeros(NBP, NBP, NBO);
	int dxi = max(-W, -xp);	
	int xlim = min(W, cols -1 - xp);	
	int ylim = min(W, rows-1-yp);
	float pi = (float)CV_PI;
	for ( ; dxi< xlim; dxi++)
	{
		int dyi = max(-W, -yp);
		for(; dyi < ylim; dyi++)
		{
			float mag = gMag.at<float>(yp + dyi, xp + dxi); 
			float angle = gDir.at<float>(yp + dyi, xp + dxi);
			angle = theta0-angle;
			angle = fmod(angle+2*pi, 2*pi);     
			float dx = xp + dxi - pt(0);            
			float dy = yp + dyi - pt(1);            

			float nx = ( costh0 * dx + sinth0 * dy) / SBP; 
			float ny = (-sinth0 * dx + costh0 * dy) / SBP; 
			float nt = nbo * angle / (2* pi) ;
			int wsigma = nbp / 2 ;
			float wincoef =  exp(-(nx*nx + ny*ny)/(2.0 * wsigma * wsigma)) ;

			int binx = cvFloor( nx - 0.5 ) ;
			int biny = cvFloor( ny - 0.5 )	;
			int bint = cvFloor( nt );
			float rbinx = nx - (binx+0.5) ;
			float rbiny = ny - (biny+0.5) ;
			float rbint = nt - bint ;
			
			for(int dbinx = 0; dbinx < 2; dbinx++) 
			{
				for(int dbiny = 0; dbiny<2; dbiny++) 
				{
					for (int dbint = 0; dbint<2; dbint++)
					{						
						if( binx+dbinx >= -2 && binx+dbinx <   2 && biny+dbiny >= -2 && biny+dbiny <  2 ) 
						{
							float	weight = wincoef * mag * abs(1 - dbinx - rbinx) * abs(1 - dbiny - rbiny) * abs(1 - dbint - rbint);
							int obin = (bint+dbint) % nbo;
							int xbin = binx+dbinx + 2;
							int ybin = biny+dbiny + 2;
							int ser = 32 * xbin + ybin*8 +obin;						
							pdat[ser] = pdat[ser] +  weight ;
						}
					}
				}
			}
		}
	}
	//cout<<des<<endl;
	double desNorm = norm(des, NORM_L2);
	des = des / desNorm;
	threshold(des, des, 0.2, 0.2, THRESH_TRUNC);
	desNorm = norm(des, NORM_L2);
	des = des / desNorm;
}

void CLineMatching::calcGrad(Mat img, Mat &gMag, Mat &gDir)
{		 
	int rows = img.rows;
	int cols = img.cols;	
	gMag = Mat(rows, cols, CV_32F);	
	gDir   = Mat(rows, cols, CV_32F);

	for (int i = 1; i < rows-1; i++ )
	{				
		float* dat1 = img.ptr<float>(i-1);
		float* dat2 = img.ptr<float>(i);
		float* dat3 = img.ptr<float>(i+1);
		float*   mag = gMag.ptr<float>(i);
		float*      dir = gDir.ptr<float>(i);
		for (int j = 1; j < cols-1; j++)
		{
			float dx = (dat2[j+1] - dat2[j-1]) / 2;			
			float dy = ( dat3[j] - dat1[j]) / 2 ;
			mag[j] = (sqrt(dx*dx + dy*dy));
			dir[j] = fast_mod(atan2(dy, dx));
		}
	}
	gMag.col(1).copyTo(gMag.col(0));
	gMag.col(cols-2).copyTo(gMag.col(cols-1));
	gMag.row(1).copyTo(gMag.row(0));
	gMag.row(rows-2).copyTo(gMag.row(rows-1));

	gDir.col(1).copyTo(gDir.col(0));
	gDir.col(cols-2).copyTo(gDir.col(cols-1));
	gDir.row(1).copyTo(gDir.row(0));
	gDir.row(rows-2).copyTo(gDir.row(rows-1));
}


void CLineMatching::describeSingleLine(Vec3f pt, const Mat gMag, const Mat gDir, Mat &des)
{
	int nbo = 8;
	int nbpy = 4;
	int sbp = 4;
	int height = nbpy * sbp/2;	
	int width = sbp;		
	int sigma = nbpy / 2;

	int rows = gMag.rows;
	int cols = gMag.cols;
	int channel = nbo * nbpy;
	des = Mat::zeros(1, channel, CV_32F);	
	float angle = pt[2];
	float dsin = sin(angle);
	float dcos = cos(angle);	
	float *pdat = des.ptr<float>(0);
	for (int m = -height; m <= height; m++)
	{
		float ny = ((float)m) / sbp;
		int biny = cvFloor(ny);
		float rbiny = ny - biny;
		float wincoef = exp(-ny*ny/(2*sigma*sigma));				
		for (int n = -width; n <= width; n++)
		{
			float fposx = dcos * n - dsin * m  + pt[0];
			float fposy = dsin * n + dcos * m + pt[1];
			int posx = cvRound(fposx);
			int posy = cvRound(fposy);
			if (posx < 0)  posx = 0;
			if (posy < 0)  posy = 0;
			if (posx >= cols)   posx  = cols - 1;
			if (posy >= rows)  posy = rows - 1;	

			float mag = gMag.at<float>(posy, posx);
			float dir    = fast_mod(-gDir.at<float>(posy, posx)+angle) ;
			float nt = nbo * dir / (2*CV_PI);
			int bint = cvFloor(nt);
			float rbint = nt - bint;
			
			for (int dbiny = 0; dbiny < 2; dbiny++ )
			{
				for (int dbint = 0; dbint < 2; dbint++)
				{
					if( biny+dbiny >= -(nbpy/2) && biny+dbiny <   (nbpy/2) ) 
					{
						float wmag = wincoef * mag * abs(1 - dbiny - rbiny) * abs(1 - dbint - rbint) ;                 							
						int obin = (bint+dbint) % nbo;
						int ybin = biny + dbiny + nbpy/2;
						int ser = ybin*nbo+obin;
						pdat[ser] = pdat[ser] + wmag;									
					}			
				}							
			}
		}
	}	
	double desNorm = norm(des, NORM_L2);
	des = des / desNorm;
	threshold(des, des, 0.2, 0.2, THRESH_TRUNC);
	desNorm = norm(des, NORM_L2);
	des = des / desNorm;
}

void CLineMatching::buildZoomPyramid( Mat& base, vector<Mat>& pyr, int nOctaves, float downsampleRatio)
{		
	pyr.resize(nOctaves);
	for( int o = 0; o < nOctaves; o++ )
	{
		Mat& dst = pyr[o];
		if( o == 0)
			dst = base;
		else
		{
			const Mat& src = pyr[o-1];
			resize(src, dst, Size(src.cols*downsampleRatio, src.rows*downsampleRatio), 0, 0, INTER_NEAREST);				
		}		
	}	
}

void CLineMatching::buildGaussianPyramid( Mat& base, vector<Mat>& pyr, int nOctaves, int nOctaveLayers, float downsampleRatio)
{	
	float sigma = 1.6;
	vector<double> sig(nOctaveLayers);
	pyr.resize(nOctaves*nOctaveLayers);
	sig[0] = sigma;
	double k = pow( 2., 1. / nOctaveLayers );
	for( int i = 1; i < nOctaveLayers; i++ )
	{
		double sig_prev = pow(k, (double)(i-1))*sigma;
		double sig_total = sig_prev*k;
		sig[i] = sqrt(sig_total*sig_total - sig_prev*sig_prev);
	}

	for( int o = 0; o < nOctaves; o++ )
	{
		for( int i = 0; i < nOctaveLayers; i++ )
		{
			Mat& dst = pyr[o*(nOctaveLayers) + i];			
			if( o == 0  &&  i == 0 )
				dst = base;

			else if( i == 0 )
			{
				const Mat& src = pyr[(o-1)*(nOctaveLayers)+nOctaveLayers - 1];
				resize(src, dst, Size(src.cols*downsampleRatio, src.rows*downsampleRatio), 0, 0, INTER_NEAREST);
			}			
			else
			{
				const Mat& src = pyr[o*(nOctaveLayers) + i-1];
				GaussianBlur(src, dst, Size(), sig[i], sig[i]);
			}
		}
	}	
}

void CLineMatching::extendIntensityValue(Mat &rimg, Mat &qimg)
{	
    double min1, min2, max1, max2;
 	minMaxLoc(rimg, &min1, &max1);
 	minMaxLoc(qimg, &min2, &max2);
 	rimg = (rimg - min1) / max1;
 	qimg = (qimg - min2) / max2;	
}

void CLineMatching::initialize_selfGenJunctions(Mat img, strLine* strline, int nline,  Mat &node, vector<strFanSection>  &vstrFanSection)
{
	intensityDirection(img, strline, nline);

	int nNode = node.rows;		
	Mat mNodePos = node.colRange(0, 2).clone();
	Mat mCLinesSer = node.colRange(2, 4);	
	int num = 0;
	vector<strBranch> vstrbranch;		
	strBranch tstrbranches;
	for (int i = 0; i < nNode; i++)
	{
		const float* curNodePos = mNodePos.ptr<float>(i);
		const float* curCLinesSer =  mCLinesSer.ptr<float>(i);
		vstrbranch.clear();
		for (int j = 0; j < 2; j++)
		{	
			int lineSerail = (int) curCLinesSer[j];
			float dx1 = strline[lineSerail].ps.x - curNodePos[0];
			float dy1 = strline[lineSerail].ps.y - curNodePos[1];
			float dx2 = strline[lineSerail].pe.x - curNodePos[0];
			float dy2 = strline[lineSerail].pe.y - curNodePos[1];								
			int flag = sign(	dx1*dx2 + dy1 * dy2);
			if (flag == -1)
			{
				if (abs(dx1) < 3 && abs(dy1) < 3)
				{
					tstrbranches.bpt = strline[lineSerail].pe;												
					tstrbranches.ang = fast_mod(atan2(dy2, dx2));	
					tstrbranches.lineSerial = lineSerail;
					vstrbranch.push_back(tstrbranches);
					continue;
				}
				if (abs(dx2) < 3 && abs(dy2) < 3)
				{
					tstrbranches.bpt = strline[lineSerail].ps;												
					tstrbranches.ang = fast_mod(atan2(dy1, dx1));				
					tstrbranches.lineSerial = lineSerail;
					vstrbranch.push_back(tstrbranches);
					continue;
				}
				tstrbranches.bpt = strline[lineSerail].pe;												
				tstrbranches.ang = fast_mod(atan2(dy2, dx2));				
				tstrbranches.lineSerial = lineSerail;
				vstrbranch.push_back(tstrbranches);
				tstrbranches.bpt = strline[lineSerail].ps;												
				tstrbranches.ang = fast_mod(atan2(dy1, dx1));								
				vstrbranch.push_back(tstrbranches);
			}			
			else
			{
				if (abs(dx2) >= abs(dx1))
				{
					tstrbranches.bpt = strline[lineSerail].pe;												
					tstrbranches.ang = fast_mod(atan2(dy2, dx2));	
					tstrbranches.lineSerial = lineSerail;
					vstrbranch.push_back(tstrbranches);					
				}
				else
				{
					tstrbranches.bpt = strline[lineSerail].ps;												
					tstrbranches.ang = fast_mod(atan2(dy1, dx1));				
					tstrbranches.lineSerial = lineSerail;
					vstrbranch.push_back(tstrbranches);					
				}
			}
		}		

		int nbranch = vstrbranch.size();
		for (int j = 0; j < nbranch; j++)
		{
			float ang1 = vstrbranch[j].ang;
			int lineSerial1 = vstrbranch[j].lineSerial;
			for(int k = j+1; k < nbranch; k++)
			{				
				int lineSerial2 = vstrbranch[k].lineSerial;
				float ang2 = vstrbranch[k].ang;			
				float dang = ang2 - ang1;
				if (lineSerial1 == lineSerial2 ) 
					continue;

				strFanSection tstrFanSection;
				strBranch tstrBranch1 = vstrbranch[j];								
				strBranch tstrBranch2 = vstrbranch[k];
				strBranch tmp;
				if (dang < 0)
				{
					tstrBranch1 = vstrbranch[k];
					tstrBranch2 = vstrbranch[j];
					dang = -dang;
				}
				if (dang > CV_PI)
				{
					tmp = tstrBranch1;
					tstrBranch1 = tstrBranch2;
					tstrBranch2 = tmp;
				}
				tstrFanSection.nodeSerail = i;
				tstrFanSection.intsection.x = curNodePos[0];
				tstrFanSection.intsection.y = curNodePos[1];
				tstrFanSection.strbranch1 = tstrBranch1;
				tstrFanSection.strbranch2=  tstrBranch2;
				tstrFanSection.fanSerial = num;
				vstrFanSection.push_back(tstrFanSection);
				num++;
			}
		}	
	}
	node = mNodePos;	
}

void CLineMatching::initialize_providedJunctions(Mat img, strLine* strline, int nline,  Mat &node, vector<strFanSection>  &vstrFanSection)
{	
	intensityDirection(img, strline, nline);

	int nNode = node.rows;		
	int ncLine = node.cols - 2;
	Mat mNodePos = node.colRange(0, 2).clone();
	Mat mCLinesSer = node.colRange(2, node.cols);	
	int num = 0;
	for (int i = 0; i < nNode; i++)
	{
		const float* curNodePos = mNodePos.ptr<float>(i);
		const float* curCLinesSer =  mCLinesSer.ptr<float>(i);
		vector<strBranch> vstrbranch;		
		for (int j = 0; j < ncLine; j++)
		{	
			int lineSerail = curCLinesSer[j]-1;
			if (lineSerail == -1) 
				break;			
			float dx1 = strline[lineSerail].ps.x - curNodePos[0];
			float dy1 = strline[lineSerail].ps.y - curNodePos[1];
			float dist1 = abs(dx1) + abs(dy1);
			float dx2 = strline[lineSerail].pe.x - curNodePos[0];
			float dy2 = strline[lineSerail].pe.y - curNodePos[1];			
			float dist2 = abs(dx2) + abs(dy2);
			strBranch tstrbranches;		
			if (dist1 == 0 && dist2 != 0)
			{
				tstrbranches.bpt = strline[lineSerail].pe;												
				tstrbranches.ang = fast_mod(atan2(dy2, dx2));	
				tstrbranches.lineSerial = lineSerail;
				vstrbranch.push_back(tstrbranches);
			}				
			else if (dist1 != 0 && dist2 == 0)
			{
				tstrbranches.bpt = strline[lineSerail].ps;												
				tstrbranches.ang = fast_mod(atan2(dy1, dx1));				
				tstrbranches.lineSerial = lineSerail;
				vstrbranch.push_back(tstrbranches);
			}				
			else
			{
				tstrbranches.bpt = strline[lineSerail].pe;												
				tstrbranches.ang = fast_mod(std::atan2(dy2, dx2));				
				tstrbranches.lineSerial = lineSerail;
				vstrbranch.push_back(tstrbranches);

				tstrbranches.bpt = strline[lineSerail].ps;												
				tstrbranches.ang = fast_mod(std::atan2(dy1, dx1));				
				tstrbranches.lineSerial = lineSerail;
				vstrbranch.push_back(tstrbranches);
			}
		}

		int nbranch = vstrbranch.size();
		for (int j = 0; j < nbranch; j++)
		{
			float ang1 = vstrbranch[j].ang;
			int lineSerial1 = vstrbranch[j].lineSerial;
			for(int k = j+1; k < nbranch; k++)
			{				
				int lineSerial2 = vstrbranch[k].lineSerial;
				float ang2 = vstrbranch[k].ang;			
				float dang = ang2 - ang1;
				if (lineSerial1 == lineSerial2 ) 
					continue;

				strFanSection tstrFanSection;
				strBranch tstrBranch1 = vstrbranch[j];								
				strBranch tstrBranch2 = vstrbranch[k];
				strBranch tmp;
				if (dang < 0)
				{
					tstrBranch1 = vstrbranch[k];
					tstrBranch2 = vstrbranch[j];
					dang = -dang;
				}
				if (dang > CV_PI)
				{
					tmp = tstrBranch1;
					tstrBranch1 = tstrBranch2;
					tstrBranch2 = tmp;
				}
				// 				else
				// 				{
				// 						tstrBranch1 = vstrbranch[2];
				// 						tstrBranch2 = vstrbranch[1];
				// 				}		
				tstrFanSection.nodeSerail = i;
				tstrFanSection.intsection.x = curNodePos[0];
				tstrFanSection.intsection.y = curNodePos[1];
				tstrFanSection.strbranch1 = tstrBranch1;
				tstrFanSection.strbranch2=  tstrBranch2;
				tstrFanSection.fanSerial = num;
				vstrFanSection.push_back(tstrFanSection);
				num++;
			}
		}		
	}
	node = mNodePos;	
}

void CLineMatching::intensityDirection(Mat oriImg,  strLine* strline, int nline)
{
	int regionWidth = _intensityProfileWidth;
	CvSize imgSize = oriImg.size();
	Mat img = oriImg.clone();
 // edianBlur ( img, img, 3);
	
	for(int i = 0; i < nline; i++)
	{
		float dx = strline[i].pe.x - strline[i].ps.x;
		float dy, direction;
		Point2f tpt;
		if (dx < 0) 
		{			
			tpt = strline[i].pe;
			strline[i].pe = strline[i].ps;
			strline[i].ps = tpt;
		}
		dx = strline[i].pe.x - strline[i].ps.x;
		dy = strline[i].pe.y - strline[i].ps.y;
		if (dx == 0)
		{
			direction = dy > 0 ? CV_PI*0.5 : CV_PI*1.5;						
		}
		else
		{
			direction = fastAtan2(dy, dx)*CV_PI/180;
		}		
		strline[i].direction = direction;
		double npixels = (abs(dy)>abs(dx)) ? cvRound(abs(dy)) : cvRound(abs(dx));
		double dsin =  sin(direction);
		double dcos = cos(direction);
		Point2f midpt;
		midpt.x = (strline[i].pe.x + strline[i].ps.x) / 2;
		midpt.y = (strline[i].pe.y + strline[i].ps.y) / 2;
		int hafpix = cvRound((npixels-1) / 2);
		double upInten = 0;
		double botInten = 0;
		for (int k = -regionWidth; k < regionWidth; k++)
		{
			for (int j = -hafpix; j < hafpix; j++)
			{
				tpt.x = dcos*j - dsin*k + midpt.x;
				tpt.y = dsin*j + dcos*k + midpt.y;
				tpt.x = cvRound(tpt.x);
				tpt.y = cvRound(tpt.y);
				if (tpt.x < 0)  tpt.x = 0;
				if (tpt.y < 0)  tpt.y = 0;
				if (tpt.x >= imgSize.width)   tpt.x = imgSize.width - 1;
				if (tpt.y >= imgSize.height)  tpt.y = imgSize.height - 1;					
				float intensity = img.at<float>(tpt.y, tpt.x); 									
				if (k < 0)
					upInten += intensity;
				else
					botInten += intensity;
			}
		}
		if (upInten >= botInten) 
		{			
			strline[i].intensityDir = 1;
			strline[i].intenRatio = upInten / botInten;
		}
		else
		{
			strline[i].intensityDir = 0;
			strline[i].intenRatio = botInten / upInten;
		}		
	}
}

void CLineMatching::plotPointMatches(Mat colorImg1, Mat colorImg2, vector<strPointMatch> vStrPointMatch, string imgName)
{
	Mat combinedImg;
	concatenateTwoImgs(colorImg1, colorImg2, combinedImg);		
	int nmach = vStrPointMatch.size();	
	for(int i=0; i<nmach; i++)
	{		
		strPointMatch curStrPointMatch = vStrPointMatch[i];
		Point2f start = curStrPointMatch.point1;
		Point2f end	= curStrPointMatch.point2;
		line(combinedImg, start, end, CV_RGB(0, 255, 0), 2 );
		stringstream ss;		
		string str;
		ss<<i;
		ss>>str;
		circle(combinedImg, start, 4, CV_RGB(0, 255, 0), -1);
		putText(combinedImg, str, start, FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255,0,0), 1);
		end.x = end.x + colorImg1.cols;
		circle(combinedImg, end, 4, CV_RGB(0, 255, 0), -1);
		putText(combinedImg, str, end, FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255,0,0), 1);		
	}
	imshow(imgName, combinedImg);	
	waitKey(20);		
}

void CLineMatching::concatenateTwoImgs(Mat mImg1, Mat mImg2, Mat &outImg)
{
	IplImage img1= IplImage(mImg1); 
	IplImage img2= IplImage(mImg2);
	IplImage* stacked = cvCreateImage( cvSize( img1.width + img2.width, max(img1.height, img2.height)), IPL_DEPTH_8U, img1.nChannels); 
	cvSetImageROI( stacked, cvRect( 0, 0, img1.width, img1.height ) ); 
	cvCopy(&img1, stacked);//, stacked, NULL ); 
	cvResetImageROI(stacked); 
	cvSetImageROI( stacked, cvRect(img1.width, 0, img2.width, img2.height) ); 
	cvCopy(&img2, stacked);//, stacked, NULL ); 
	cvResetImageROI(stacked); 
	outImg = Mat(stacked,0);
}

void CLineMatching::getPointsonPolarline(vector<Point2f> &PointSet1,vector<Point2f> &PointSet2, Mat_<double> F, double T, bool *pbIsKept)
{
	vector<Point2f> tempSet1,tempSet2;
	tempSet1 = PointSet1;
	tempSet2 = PointSet2;
	PointSet1.clear();
	PointSet2.clear();
	double a,b,c,dist;
	Mat_<double> homoPoint,lineCoeffs;
	Point2f refPoint,testPoint;
	for (unsigned i = 0; i < tempSet1.size(); i ++)
	{
		refPoint  = tempSet1[i];
		testPoint = tempSet2[i];
		homoPoint = (Mat_<double>(3,1) << refPoint.x, refPoint.y, 1);
		lineCoeffs = F * homoPoint;		
		a = lineCoeffs(0);
		b = lineCoeffs(1);
		c = lineCoeffs(2);

		dist = fabs(a*testPoint.x + b*testPoint.y + c)/sqrt(a*a + b*b);
		if (dist <= T)
		{
			PointSet1.push_back(refPoint);
			PointSet2.push_back(testPoint);
			pbIsKept[i] = 1;
		}
		else
		{
			pbIsKept[i] = 0;
		}
	}
}

void CLineMatching::getResiduauls(Mat  pts1, Mat pts2, Mat FMat, Mat_<float> &residuals)
{
	Mat transPts1 = pts1.t();
	Mat transPts2 = pts2.t();
	if (pts1.cols == 2 && pts2.cols == 2)
	{
		int npairs = pts1.rows;
		Mat tmat = Mat::ones(1, npairs, CV_32F);
		transPts1.push_back(tmat);
		transPts2.push_back(tmat);
	}

   	Mat Lp2;
	Lp2 = FMat * transPts1;   
	Mat tmat1, tmat2, tmat3, tmat4;
	reduce( transPts2.mul(Lp2), tmat1, 0, CV_REDUCE_SUM);
	tmat1 = abs(tmat1);
	pow( Lp2.rowRange(0,2), 2, tmat2);
	reduce( tmat2, tmat3, 0, CV_REDUCE_SUM);
	sqrt(tmat3, tmat4);
	Mat dist1, dist2;
	divide(tmat1, tmat4, dist1);

	Lp2 = FMat.t() * transPts2;   
	reduce( transPts1.mul(Lp2), tmat1, 0, CV_REDUCE_SUM);
	tmat1 = abs(tmat1);
	pow( Lp2.rowRange(0,2), 2, tmat2);
	reduce( tmat2, tmat3, 0, CV_REDUCE_SUM);
	sqrt(tmat3, tmat4);
	divide(tmat1, tmat4, dist2);
	cout<<dist2<<endl;

	residuals = dist1 + dist2;	
}

void CLineMatching::findRobustFundamentalMat(vector<Point2f> &PointSet1,vector<Point2f> &PointSet2, Mat &FMat, bool *pbIsKept)
{
	unsigned i;
	vector<uchar> status;
	findFundamentalMat(PointSet1,PointSet2,CV_RANSAC, _fmatThr, 0.99, status);
	int time = 0;
	vector<Point2f> goodPoints1,goodPoints2;
	for (i = 0; i < status.size(); i ++)
	{
		if (status[i] == 1)
		{
			goodPoints1.push_back(PointSet1[i]);
			goodPoints2.push_back(PointSet2[i]);
		}
	}
	Mat F = findFundamentalMat(goodPoints1, goodPoints2,CV_LMEDS, _fmatThr, 0.99, status);
	getPointsonPolarline(PointSet1, PointSet2, F, _fmatThr, pbIsKept);
	FMat = findFundamentalMat(PointSet1, PointSet2,CV_LMEDS, _fmatThr, 0.99, status);
	FMat.convertTo(FMat, CV_32F);
}

void CLineMatching::drawEpipolarline(Mat image1,Mat image2,vector<Point2f> pointSet1,vector<Point2f> pointSet2, Mat_<double> Fmatrix)
{
	unsigned i,j;
	int rows = image1.rows;
	int cols = image1.cols;
	for (i = 0; i < pointSet1.size(); i ++)
	{
		circle(image1, pointSet1[i], 5, Scalar(0, 0, 255), -1);
		circle(image2, pointSet2[i], 5, Scalar(0, 0, 255), -1);
	}
	for (i = 0; i < pointSet1.size(); i ++)         //draw epipolar lines on image2
	{
		Mat_<double> feature = (Mat_<double>(3,1) << pointSet1[i].x,pointSet1[i].y, 1);
		Mat_<double> k = Fmatrix * feature;
		double a,b,c;
		a = k(0); b = k(1); c = k(2);
		double x0,y0,x1,y1;
		if (a ==0)
		{
			cv::line(image2, Point2f(0,-c/b), Point2f(cols-1,-c/b), Scalar(0, 255, 0));			
			continue;
		}
		if (b ==0)
		{
			cv::line(image2, Point2f(-c/a,0), Point2f(-c/a,rows-1), Scalar(0, 255, 0));
			continue;
		}
		x0 = 0; x1 = cols-1;
		y0 = (-c-a*x0)/b;y1 = (-c-a*x1)/b;
		cv::line(image2, Point2f(x0,y0), Point2f(x1,y1), Scalar(0, 255, 0));
	}
	for (i = 0; i < pointSet2.size(); i ++)            //draw epipolar lines on image1
	{
		Mat_<double> feature = (Mat_<double>(3,1) << pointSet2[i].x,pointSet2[i].y, 1);
		Mat_<double> k =  feature.t() * Fmatrix;
		double a,b,c;
		a = k(0); b = k(1); c = k(2);
		double x0,y0,x1,y1;
		if (a ==0)
		{
			cv::line(image1, Point2f(0,-c/b), Point2f(cols-1,-c/b), Scalar(0, 255, 0));
			continue;
		}
		if (b ==0)
		{
			cv::line(image1, Point2f(-c/a,0), Point2f(-c/a,rows-1), Scalar(0, 255, 0));
			continue;
		}
		x0 = 0; x1 = cols-1;
		y0 = (-c-a*x0)/b;y1 = (-c-a*x1)/b;
		cv::line(image1, Point2f(x0,y0), Point2f(x1,y1), Scalar(0, 255, 0));
	}
	
	cv::imshow("Epipolar lines in the first image", image1);
	//imwrite("LeftImage.jpg", image1);
	cv::imshow("Epipolar lines in the second image", image2);
	//imwrite("RightImage.jpg", image2);
	waitKey(20);		
}

CLineMatching::~CLineMatching()
{
};

CLineMatching::CLineMatching()
{
};

