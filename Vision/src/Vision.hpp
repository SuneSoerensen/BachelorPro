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
	static Coords FindCOM(vector<Coords> &aContourList, Mat &aContourImage);
	static void FindGraspRegs(vector<vector<Coords> > &aGraspRegsList, Mat &aContourImage, vector<vector<int> > &aContourMatrix);
	static void CalcNormVecs(vector<vector<Coords> > &aGraspRegsList, vector<Coords> &aNormVecsList);
	static double CalcAngle(Coords vecA, Coords vecB);
	//static void FindPossGrasps(vector<vector<int> > &aPossGraspsList, vector<Coords> &aNormVecsList);

	Coords GetRealCoords(Coords coordsInPixels);

	//================
	//   Attributes
	//================
	//taking an image
	Mat cropImage;

	//finding the contour
	Mat contourImage;
	vector<Coords> contourList;
	vector<vector<int> > contourMatrix;

	//grasping
	vector<vector<Coords> > graspRegsList;
	vector<Coords> normVecsList;
	//vector<vector<int> > possGraspsList

	//calibration
	Coords offset;
	Coords scaleFactor;
};

