#include "Vision.hpp"
#include <iostream>

Vision::Vision()
{
}

void Vision::Calib()
{
	//get new data
	TIAFC::DoItAll(cropImage, contour);

	//calculate the offset
	offset = TIAFC::FindCOM(contour).Mul(-1);

	//find height and width of calibration object
	Coords min = contour.list[0];
	Coords max = contour.list[0];
	for (int i = 1; i < contour.list.size(); i++)
	{
		if (contour.list[i].x < min.x)
			min.x = contour.list[i].x;

		if (contour.list[i].x > max.x)
			max.x = contour.list[i].x;

		if (contour.list[i].y < min.y)
			min.y = contour.list[i].y;

		if (contour.list[i].y > max.y)
			max.y = contour.list[i].y;
	}

	//calculate the scale factor
	Coords dims = max.Sub(min); //the calibration objects dimensions in pixels
	double scaleX = REAL_WIDTH / dims.x; //mm/pixels
	double scaleY = REAL_HEIGHT / dims.y; //mm/pixels
	scaleFactor = (scaleX + scaleY) / 2.0; //average of the scales
}

void Vision::CalcGrasp()
{
	//get new data
	TIAFC::DoItAll(cropImage, contour);

	grasp = AnalytGrasp::FindGrasp(contour, scaleFactor);

	if (VISION_MODE)
	{
		//for points:
		//make image
		Mat graspImage;
		contour.image.copyTo(graspImage);
		cvtColor(graspImage, graspImage, COLOR_GRAY2BGR);

		//drawing color
		Vec3b color;

		//draw pixels to mark point a
		color.val[0] = 0; //blue
		color.val[1] = 0; //green
		color.val[2] = 255; //red
		for (int i = -2; i <= 2; i++)
			for (int j = -2; j <= 2; j++)
				graspImage.at<Vec3b>(grasp.points[0].y + j, grasp.points[0].x + i) = color;

		//draw pixels to mark point b
		if (grasp.type != p3)
		{
			color.val[0] = 0; //blue
			color.val[1] = 255; //green
			color.val[2] = 0; //red
			for (int i = -2; i <= 2; i++)
				for (int j = -2; j <= 2; j++)
					graspImage.at<Vec3b>(grasp.points[1].y + j, grasp.points[1].x + i) = color;
		}

		//draw pixels to mark point c
		color.val[0] = 255; //blue
		color.val[1] = 0; //green
		color.val[2] = 0; //red
		for (int i = -2; i <= 2; i++)
			for (int j = -2; j <= 2; j++)
				graspImage.at<Vec3b>(grasp.points[2].y + j, grasp.points[2].x + i) = color;

		//save image
		imwrite("InfoFiles/Vision(grasp_points).jpg", graspImage);

		//for focus and COM:
		//make image
		Mat COMFocusImage;
		contour.image.copyTo(COMFocusImage);
		cvtColor(COMFocusImage, COMFocusImage, COLOR_GRAY2BGR);

		//draw pixels to mark the focus
		color.val[0] = 0; //blue
		color.val[1] = 255; //green
		color.val[2] = 0; //red
		for (int i = -2; i <= 2; i++)
			for (int j = -2; j <= 2; j++)
				graspImage.at<Vec3b>(grasp.focus.y + j, grasp.focus.x + i) = color;

		//draw pixels to mark the COM
		color.val[0] = 0; //blue
		color.val[1] = 0; //green
		color.val[2] = 255; //red
		for (int i = -2; i <= 2; i++)
			for (int j = -2; j <= 2; j++)
				COMFocusImage.at<Vec3b>(grasp.COM.y + j, grasp.COM.x + i) = color;

		//save image
		imwrite("InfoFiles/Vision(COM_and_focus).jpg", COMFocusImage);
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
	return GetRealCoords(TIAFC::FindCOM(contour));
}

Vision::~Vision()
{
}

Coords Vision::GetRealCoords(Coords coordsInPixels)
{
	Coords globAxeDirs(GLOB_X_DIR, GLOB_Y_DIR);
	Coords temp = (coordsInPixels.Add(offset)).Mul(globAxeDirs);
	return Coords(temp.x * scaleFactor, temp.y * scaleFactor); //pixels * (mm / pixels) = pixels
}

