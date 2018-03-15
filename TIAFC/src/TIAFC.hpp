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
	static void FindContour(Mat &aCropImage, Mat &aContourImage, vector<Coords> &aContour);

private:
	static Coords FindNextNeighbour(Mat &aContourImage, Coords aPrevPoint);

	TIAFC();
	~TIAFC();
};
