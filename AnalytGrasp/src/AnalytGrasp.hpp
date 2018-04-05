#pragma once
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include "settings.hpp"
#include "Coords.hpp"
#include <string>

using namespace std;
using namespace cv;



class AnalytGrasp //Analytical Grasping
{
public:
	static void FindGraspPoints(vector<Coords> &aGraspPointsList, Mat &aContourImage, vector<Coords> &aContourList, vector<vector<int> > &aContourMatrix);

private:
	static void FindGraspRegs(vector<vector<Coords> > &aGraspRegsList, Mat &aContourImage, vector<vector<int> > &aContourMatrix);
	static void CalcNormVecs(vector<vector<Coords> > &aGraspRegsList, vector<Coords> &aNormVecsList);
	static double CalcAngle(Coords vecA, Coords vecB);
	static void ThreeFingAngCheck(vector<vector<int> > &aPossGraspsList, vector<Coords> &aNormVecsList);
	static void TwoFingAngCheck(vector<vector<int> > &aPossGraspsList, vector<Coords> &aNormVecsList);
	//finger names!!!

	AnalytGrasp();
	~AnalytGrasp();
};

