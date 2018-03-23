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

void Vision::RunFindGraspPoints()
{
	AnalytGrasp::FindGraspPoints(graspPointsList, contourImage, contourList, contourMatrix);
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
		imwrite("DebugFiles/Vision_centOfMass.jpg", centOfMassImage);
	}

	return res;
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

