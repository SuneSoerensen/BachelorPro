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

struct GraspReg
{
	Coords point;
	Coords dirVec;
	Coords normVec;
};

struct Grasp
{
	Type type;
	vector<Coords> points;
	Coords focus;
	Coords COM;
};

class AnalytGrasp //Analytical Grasping
{
public:
	static Grasp FindGrasp(Contour &aContour, double aScaleFactor);

private:
	static void FindGraspRegs(vector<GraspReg> &aGraspRegsList, Contour &aContour, double aScaleFactor);

	static void CalcP1Grasps(vector<Grasp> &aP1GraspsList, vector<GraspReg> &aGraspRegsList, Coords aCOM, double aScaleFactor);
	static bool P1AngCheck(GraspReg a, GraspReg b, GraspReg c);
	static bool P1PosCheck(GraspReg a, GraspReg b, GraspReg c, Coords aFocus, Coords aCOM, double aScaleFactor);

	static void TwoFingAngCheck(vector<vector<int> > &aPossGraspsList, vector<Coords> &aNormVecsList);

	static double CalcAngle(Coords vecA, Coords vecB);

	AnalytGrasp();
	~AnalytGrasp();
};

