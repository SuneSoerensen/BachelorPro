#include "Vision.hpp"
#include <iostream>



Vision::Vision()
{
}

Coords Vision::GetObjCoords()
{
	//get new data
	TIAFC::TakeImage(cropImage);
	TIAFC::FindContour(cropImage, contourImage, contour);

	//return the objects COM in real coordinates
	return GetRealCoords(FindCOM(contour, contourImage));
}

void Vision::Calib()
{
	//get new data
	TIAFC::TakeImage(cropImage);
	TIAFC::FindContour(cropImage, contourImage, contour);

	//calculate the offset
	offset = FindCOM(contour, contourImage).Mul(-1);

	//find height and width of calibration object
	Coords min = contour[0];
	Coords max = contour[0];
	for (int i = 1; i < contour.size(); i++)
	{
		if (contour[i].x < min.x)
			min.x = contour[i].x;

		if (contour[i].x > max.x)
			max.x = contour[i].x;

		if (contour[i].y < min.y)
			min.y = contour[i].y;

		if (contour[i].y > max.y)
			max.y = contour[i].y;
	}

	//calculate the scale factor
	Coords locScale = max.Sub(min); //pixels
	Coords globScale(REAL_WIDTH*CALC_FACTOR, REAL_HEIGHT*CALC_FACTOR); //mm*CALC_FACTOR
	scaleFactor = globScale.Div(locScale); //(mm*CALC_FACTOR)/pixels
}

Vision::~Vision()
{
}

Coords Vision::FindCOM(vector<Coords> &aContour, Mat &aContourImage)
{
	//calculate sum of all x- and y-coordinates
	Coords sum(0, 0);
	for (int i = 0; i < aContour.size(); i++)
	{
		sum = sum.Add(aContour[i]);
	}

	//divide by number of points to find average
	Coords res = sum.Div(aContour.size());

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

void Vision::FindGraspRegs(vector<vector<Coords> > &aListOfLines, Mat &aContourImage)
{
	//use Hough-transform to find straight lines
	vector<Vec4i> lines;
	HoughLinesP(aContourImage, lines, 1, CV_PI/180, MIN_POINTS_IN_LINE, MIN_LINE_LENGTH, MAX_LINE_GAP );

	if (lines.size() == 0)
		throw("[Vision::FindGraspRegs()]: Couldn't find any grasp regions!");

	aListOfLines.resize(lines.size());
	for (int i = 0; i < lines.size(); i++)
	{
		aListOfLines[i].resize(2);
		aListOfLines[i][0].Set(lines[i][0], lines[i][1]);
		aListOfLines[i][1].Set(lines[i][2], lines[i][3]);
	}

	if (VISION_MODE) //DEBUG
	{
		Mat graspRegsImage;
		aContourImage.copyTo(graspRegsImage);
		cvtColor(graspRegsImage, graspRegsImage, COLOR_GRAY2BGR);
		for( int i = 0; i < lines.size(); i++ )
		{
			Point startP = Point(lines[i][0], lines[i][1]);
			Point endP = Point(lines[i][2], lines[i][3]);
			line( graspRegsImage, startP, endP, Scalar(0,0,255), 1, LINE_AA);
		}

		imwrite("Vision_graspRegs.jpg", graspRegsImage);
	}
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

