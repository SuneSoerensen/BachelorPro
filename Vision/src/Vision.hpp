#pragma once
#include "TIAFC.hpp"
#include "AnalytGrasp.hpp"

using namespace std;

class Vision //Vision
{
public:
	Vision();

	void Calib(); //always call "Calib()" at least once before doing anything else!!!

	void CalcGrasp(); //call this before the other grasp related functions!
	Coords GetGraspFocus();
	vector<Coords> GetGraspPoints();

	Coords GetObjCoords(); //this is not meant for grasping!

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
	Grasp grasp;

	//calibration
	Coords offset;
	Coords scaleFactor;
};
