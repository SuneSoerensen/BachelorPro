#pragma once
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include "settings.hpp"
#include "Coords.hpp"

using namespace std;
using namespace cv;



class TIAFC //Take Image And Find Contour
{
public:
	static void TakeImage(Mat &aCropImage);
	static void FindContour(Mat &aCropImage, Mat &aContourImage, vector<Coords> &aContourList, vector<vector<int> > &aContourMatrix);

private:
	static Coords FindNextNeighbour(Mat &aThresImage, vector<vector<int> > &aContourMatrix, Coords aCurrPoint);
	static bool HasBlackNeighbour(Mat &aThresImage, Coords aCurrPoint);

	TIAFC();
	~TIAFC();
};

