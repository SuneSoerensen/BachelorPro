#include "Vision.hpp"
#include <iostream>

Vision::Vision()
{
}

void Vision::Calib()
{
	//get new data
	TIAFC::DoItAll(cropImage, contourImage, contourList, contourMatrix);

	//calculate the offset
	offset = AnalytGrasp::FindCOM(contourList, contourImage).Mul(-1);

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

void Vision::CalcGrasp()
{
	//get new data
	TIAFC::DoItAll(cropImage, contourImage, contourList, contourMatrix);

	grasp = AnalytGrasp::FindGrasp(contourImage, contourList, contourMatrix);

	if (VISION_MODE)
	{
		//make image
		Mat graspImage;
		contourImage.copyTo(graspImage);
		cvtColor(graspImage, graspImage, COLOR_GRAY2BGR);

		//relative coordinates of pixels to draw
		int xVals[17] = {0, 2,  2,  2,  1,  0, -1, -2, -2, -2, -2, -2, -1, 0, 1, 2, 2};
		int yVals[17] = {0, 0, -1, -2, -2, -2, -2, -2, -1,  0,  1,  2,  2, 2, 2, 2, 1};

		//drawing color
		Vec3b color;
		color.val[0] = 255; //blue
		color.val[1] = 255; //green
		color.val[2] = 255; //red

		//draw pixels to mark the focus
		for (int i = 0; i < 17; i++)
		{
			graspImage.at<Vec3b>(grasp.focus.y + yVals[i], grasp.focus.x + xVals[i]) = color;
		}

		//draw pixels to mark point a
		color.val[0] = 0; //blue
		color.val[1] = 0; //green
		color.val[2] = 255; //red
		for (int i = 0; i < 17; i++)
		{
			graspImage.at<Vec3b>(grasp.points[0].y + yVals[i], grasp.points[0].x + xVals[i]) = color;
		}

		//draw pixels to mark point b
		color.val[0] = 0; //blue
		color.val[1] = 255; //green
		color.val[2] = 0; //red
		for (int i = 0; i < 17; i++)
		{
			graspImage.at<Vec3b>(grasp.points[1].y + yVals[i], grasp.points[1].x + xVals[i]) = color;
		}

		//draw pixels to mark point b
		color.val[0] = 255; //blue
		color.val[1] = 0; //green
		color.val[2] = 0; //red
		for (int i = 0; i < 17; i++)
		{
			graspImage.at<Vec3b>(grasp.points[2].y + yVals[i], grasp.points[2].x + xVals[i]) = color;
		}

		//save image
		imwrite("InfoFiles/Vision_grasp.jpg", graspImage);
	}
}

Coords Vision::GetGraspFocus()
{
	//return the grasps focus in real coordinates
	return GetRealCoords(grasp.focus);
}

vector<Coords> Vision::GetGraspPoints()
{
	//return the grasps points in real coordinates
	return {GetRealCoords(grasp.points[0]), GetRealCoords(grasp.points[1]), GetRealCoords(grasp.points[2])};
}

double Vision::GetDistFromCOM()
{
	return grasp.distFromCOM;
}

Coords Vision::GetObjCoords()
{
	//return the objects COM in real coordinates
	return GetRealCoords(AnalytGrasp::FindCOM(contourList, contourImage));
}

Vision::~Vision()
{
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

