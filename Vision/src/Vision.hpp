#pragma once
#include "TIAFC.hpp"
#include "AnalytGrasp.hpp"

using namespace std;

class Vision //Vision
{
public:
	Vision();

	Coords GetObjCoords();
	void Calib(); //always call "Calib()" at least once before doing anything else!!!

	/*DEBUG*/void RunFindGrasp();

	~Vision();

private:
	Coords GetRealCoords(Coords coordsInPixels);

	//================
	//   Attributes
	//================
	//taking an image
	Mat cropImage;

	//finding the contour
	Mat contourImage;
	vector<Coords> contourList; //list of points in contour
	vector<vector<int> > contourMatrix; //relates coordinates of contour points to indices in the list (indices are offset by +1!!!)

	//grasping
	vector<Coords> graspPointsList;

	//calibration
	Coords offset;
	Coords scaleFactor;
};

