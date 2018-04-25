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
	Contour contour;

	//grasping
	Grasp grasp;

	//calibration
	Coords offset;
	double scaleFactor; //mm/pixels
};
