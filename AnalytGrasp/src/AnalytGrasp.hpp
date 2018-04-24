#pragma once
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include "settings.hpp"
#include "Coords.hpp"
#include <string>
#include "TIAFC.hpp"

using namespace std;
using namespace cv;

enum Type {p1, p2, p3};

struct Grasp
{
	Type type;
	vector<Coords> points;
	Coords focus;
	Coords COM;
	double distFromCOM;
};

class AnalytGrasp //Analytical Grasping
{
public:
	static Grasp FindGrasp(Contour &aContour, double aScaleFactor);

private:
	static void FindGraspRegs(vector<vector<Coords> > &aGraspRegsList, Contour &aContour, double aScaleFactor);

	static void CalcNormVecs(vector<vector<Coords> > &aGraspRegsList, vector<Coords> &aNormVecsList);

	static void CalcP1Grasps(vector<Grasp> &aP1GraspsList, vector<vector<Coords> > &aGraspRegsList, vector<Coords> aNormVecsList, Coords aCOM);
	static void P1AngCheck(vector<vector<int> > &aPassedAngCheckList, vector<Coords> &aNormVecsList);
	static bool P1IntersecCheck(Coords pA, Coords rA, Coords pB, Coords rB, Coords pC, Coords rC);

	static void TwoFingAngCheck(vector<vector<int> > &aPossGraspsList, vector<Coords> &aNormVecsList);

	static double CalcAngle(Coords vecA, Coords vecB);
	static Coords CalcIntersection(Coords pL, Coords rL, Coords pM, Coords rM);

	AnalytGrasp();
	~AnalytGrasp();
};

