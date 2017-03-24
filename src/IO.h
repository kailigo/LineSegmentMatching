#ifndef	 IO_H
#define IO_H

#include <iostream>
#include <cv.h>
#include <fstream>  
#include <iterator>  
#include <vector>  
using namespace cv;
using namespace std;  

class CIO
{
public:
		CIO();
		~CIO();
		int writeData(string fileName, cv::Mat& matData);
		int loadData(string fileName, cv::Mat& matData, int matRows = 0, int matCols = 0, int matChns = 0);
};
#endif  // IO_H