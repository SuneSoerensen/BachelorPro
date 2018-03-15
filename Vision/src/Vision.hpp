#pragma once
#include "TIAFC.hpp"

using namespace std;



class Vision //Vision
{
public:
	Vision();

	Coords GetObjCoords();
	void Calib();

	~Vision();

//private:
	static Coords FindCOM(vector<Coords> &aContour, Mat &aContourImage);
	static void FindGraspRegs(vector<vector<Coords> > &aListOfLines, Mat &aContourImage);

	Coords GetRealCoords(Coords coordsInPixels);

	Mat cropImage;
	Mat contourImage;
	vector<Coords> contour;

	vector<vector<Coords> > graspRegsList;

	Coords offset;
	Coords scaleFactor;
};

