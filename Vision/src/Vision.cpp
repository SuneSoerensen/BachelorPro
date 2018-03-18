#include "Vision.hpp"
#include <iostream>



Vision::Vision()
{
}

Coords Vision::GetObjCoords()
{
	//get new data
	TIAFC::TakeImage(cropImage);
	TIAFC::FindContour(cropImage, contourImage, contourList, contourMatrix);

	//return the objects COM in real coordinates
	return GetRealCoords(FindCOM(contourList, contourImage));
}

void Vision::Calib()
{
	//get new data
	TIAFC::TakeImage(cropImage);
	TIAFC::FindContour(cropImage, contourImage, contourList, contourMatrix);

	//calculate the offset
	offset = FindCOM(contourList, contourImage).Mul(-1);

	//find height and width of calibration object
	Coords min = contourList[0];
	Coords max = contourList[0];
	for (int i = 1; i < contourList.size(); i++)
	{
		if (contourList[i].x < min.x)
			min.x = contourList[i].x;

		if (contourList[i].x > max.x)
			max.x = contourList[i].x;

		if (contourList[i].y < min.y)
			min.y = contourList[i].y;

		if (contourList[i].y > max.y)
			max.y = contourList[i].y;
	}

	//calculate the scale factor
	Coords locScale = max.Sub(min); //pixels
	Coords globScale(REAL_WIDTH*CALC_FACTOR, REAL_HEIGHT*CALC_FACTOR); //mm*CALC_FACTOR
	scaleFactor = globScale.Div(locScale); //(mm*CALC_FACTOR)/pixels
}

Vision::~Vision()
{
}

Coords Vision::FindCOM(vector<Coords> &aContourList, Mat &aContourImage)
{
	//calculate sum of all x- and y-coordinates
	Coords sum(0, 0);
	for (int i = 0; i < aContourList.size(); i++)
	{
		sum = sum.Add(aContourList[i]);
	}

	//divide by number of points to find average
	Coords res = sum.Div(aContourList.size());

	if (VISION_MODE) //DEBUG
	{
		//make image
		Mat centOfMassImage;
		aContourImage.copyTo(centOfMassImage);
		cvtColor(centOfMassImage, centOfMassImage, COLOR_GRAY2BGR);

		//relative coordinates of pixels to draw
		int xVals[17] = {0, 2,  2,  2,  1,  0, -1, -2, -2, -2, -2, -2, -1, 0, 1, 2, 2};
		int yVals[17] = {0, 0, -1, -2, -2, -2, -2, -2, -1,  0,  1,  2,  2, 2, 2, 2, 1};

		//drawing color
		Vec3b color;
		color.val[0] = 0;
		color.val[1] = 0;
		color.val[2] = 255;

		//draw pixels to mark the COM
		for (int i = 0; i < 17; i++)
		{
			centOfMassImage.at<Vec3b>(res.y + yVals[i], res.x + xVals[i]) = color;
		}

		//save image
		imwrite("Vision_centOfMass.jpg", centOfMassImage);
	}

	return res;
}

void Vision::FindGraspRegs(vector<vector<Coords> > &aGraspRegsList, Mat &aContourImage, vector<vector<int> > &aContourMatrix)
{
	//use Hough-transform to find straight lines
	vector<Vec4i> lines;
	HoughLinesP(aContourImage, lines, 1, CV_PI/180, MIN_POINTS_IN_LINE, MIN_LINE_LENGTH, MAX_LINE_GAP );

	if (lines.size() == 0) //check that lines were found
		throw("[Vision::FindGraspRegs()]: Couldn't find any grasp regions!");

	//copy found lines to the list of grasping regions
	aGraspRegsList.resize(lines.size());
	for (int i = 0; i < lines.size(); i++)
	{
		aGraspRegsList[i].resize(2);

		if (aContourMatrix[lines[i][0]][lines[i][1]] < aContourMatrix[lines[i][2]][lines[i][3]]) //make sure the order is correct
		{
			aGraspRegsList[i][0].Set(lines[i][0], lines[i][1]);
			aGraspRegsList[i][1].Set(lines[i][2], lines[i][3]);
		}
		else
		{
			aGraspRegsList[i][0].Set(lines[i][2], lines[i][3]);
			aGraspRegsList[i][1].Set(lines[i][0], lines[i][1]);
		}
	}

	if (VISION_MODE) //DEBUG
	{
		//make output image
		Mat graspRegsImage;
		aContourImage.copyTo(graspRegsImage);
		cvtColor(graspRegsImage, graspRegsImage, COLOR_GRAY2BGR);

		//draw the found lines
		for( int i = 0; i < lines.size(); i++ )
		{
			Point startP = Point(lines[i][0], lines[i][1]);
			Point endP = Point(lines[i][2], lines[i][3]);
			line( graspRegsImage, startP, endP, Scalar(0,0,255), 1, LINE_AA);
		}

		//save to image
		imwrite("Vision_graspRegs.jpg", graspRegsImage);
	}
}

void Vision::CalcNormVecs(vector<vector<Coords> > &aGraspRegsList, vector<Coords> &aNormVecsList)
{
	//init result
	aNormVecsList.resize(aGraspRegsList.size());

	//calc direction-vectors
	for (int i = 0; i < aGraspRegsList.size(); i++)
	{
		aNormVecsList[i] = aGraspRegsList[i][1].Sub(aGraspRegsList[i][0]); //r = p_end - p_start
	}

	//calc normal-vectors
	for (int i = 0; i < aNormVecsList.size(); i++)
	{
		aNormVecsList[i].Set(aNormVecsList[i].y, -aNormVecsList[i].x); //n = (y, -x), r = (x, y)
	}
}

double Vision::CalcAngle(Coords vecA, Coords vecB)
{
	return acos(vecA.Dot(vecB) / (vecA.Length()*vecB.Length()));
}

Coords Vision::GetRealCoords(Coords coordsInPixels)
{
	Coords globAxeDirs(GLOB_X_DIR, GLOB_Y_DIR);
	return (((coordsInPixels.Add(offset)).Mul(globAxeDirs)).Mul(scaleFactor)).Div(CALC_FACTOR);
	// ((pixels + pixels) * ((mm * CALC_FACTOR) / pixels)) / CALC_FACTOR
	//				=
	//	(pixels * mm * CALC_FACTOR) / (pixels * CALC_FACTOR)
	//				=
	//				mm
}

