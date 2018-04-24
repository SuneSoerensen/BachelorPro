#pragma once
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include "settings.hpp"
#include "Coords.hpp"

using namespace std;
using namespace cv;

struct Contour
{
	Mat image; //image of the contour
	vector<Coords> list; //list of the points in the contour
	vector<vector<int> > matrix; //relates coordinates of contour points to indices in the list (indices are offset by +1!!!)
};

class TIAFC //Take Image And Find Contour
{
public:
	static Coords FindCOM(Contour &aContour);

	static void DoItAll(Mat &aCropImage, Contour &aContour);

private:
	static void TakeImage(Mat &aCropImage);

	static void FindObject(Mat &aCropImage, Mat &anObjectImage);
	static void ThereCanOnlyBeOne(Mat &aThresImage, bool mode);
	static void CatchNeighbours(Mat &aThresImage, int aColor, Coords aCurrPoint, vector<Coords> &aListOfPoints);

	static void FindContour(Mat &anObjectImage, Contour &aContour);
	static Coords FindNextNeighbour(Mat &anObjectImage, Contour &aContour, Coords aCurrPoint);
	static bool HasBlackNeighbour(Mat &anObjectImage, Coords aCurrPoint);
	static int NumOfWhiteNeighbours(Mat &anObjectImage, Coords aCurrPoint);

	static bool IsWithinBounds(Mat &anImage, int anX, int aY);

	TIAFC();
	~TIAFC();
};

