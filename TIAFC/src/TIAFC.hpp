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
	static void DoItAll(Mat &aCropImage, Mat &aContourImage, vector<Coords> &aContourList, vector<vector<int> > &aContourMatrix);

private:
	static void TakeImage(Mat &aCropImage);

	static void FindObject(Mat &aCropImage, Mat &anObjectImage);
	static void ThereCanOnlyBeOne(Mat &aThresImage, bool mode);
	static void CatchNeighbours(Mat &aThresImage, int aColor, Coords aCurrPoint, vector<Coords> &aListOfPoints);

	static void FindContour(Mat &anObjectImage, Mat &aContourImage, vector<Coords> &aContourList, vector<vector<int> > &aContourMatrix);
	static Coords FindNextNeighbour(Mat &anObjectImage, vector<vector<int> > &aContourMatrix, Coords aCurrPoint);
	static bool HasBlackNeighbour(Mat &anObjectImage, Coords aCurrPoint);

	static bool IsWithinBounds(Mat &anImage, int anX, int aY);

	TIAFC();
	~TIAFC();
};

