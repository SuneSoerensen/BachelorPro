#include "TIAFC.hpp"
#include <iostream>



void TIAFC::TakeImage(Mat &aCropImage)
{
	//open selected camera
	VideoCapture cap;
	cap.open(CAMERA);

	if (!cap.isOpened()) //check that camera was opened
		throw("[TIAFC::TakeImage()]: Couldn't open camera!");

	//set camera resoluton
	cap.set(CV_CAP_PROP_FRAME_WIDTH, CAM_RES_WIDTH);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, CAM_RES_HEIGHT);

	//get frame
	Mat frame;
	/*for (int i = 0; i < FRAMES_TO_GET ; i++) //comment this out when the following debug is commented in
	{
		cap.read(frame);
	}*/

	/*DEBUG*/frame = imread("TestImg/vandkande.jpg");

	//check crop
	if (CROP_TOP < 0 || CROP_TOP > CROP_BOTTOM)
		throw("[TIAFC::TakeImage()]: Invalid crop!");
	if (CROP_BOTTOM < CROP_TOP || CROP_BOTTOM > frame.rows)
		throw("[TIAFC::TakeImage()]: Invalid crop!");
	if (CROP_LEFT < 0 || CROP_LEFT > CROP_RIGHT)
		throw("[TIAFC::TakeImage()]: Invalid crop!");
	if (CROP_RIGHT < CROP_LEFT || CROP_RIGHT > frame.cols)
		throw("[TIAFC::TakeImage()]: Invalid crop!");

	//crop and save
	Mat croppedFrame(frame, Rect(CROP_LEFT, CROP_TOP, CROP_RIGHT - CROP_LEFT, CROP_BOTTOM - CROP_TOP));
	croppedFrame.copyTo(aCropImage);

	if (TIAFC_MODE) //DEBUG
	{
		imwrite("DebugFiles/TIAFC_img_proc_1(image).jpg", frame);
		imwrite("DebugFiles/TIAFC_img_proc_2(crop).jpg", aCropImage);
	}
}

void TIAFC::FindContour(Mat &aCropImage, Mat &aContourImage, vector<Coords> &aContourList, vector<vector<int> > &aContourMatrix)
{
	if (!aCropImage.data) //check if empty
		throw("[TIAFC::FindContour()]: No image!");

	//make greyscale image
	Mat greyscaleImage;
	cvtColor(aCropImage, greyscaleImage, COLOR_BGR2GRAY);
	if (TIAFC_MODE) //DEBUG
		imwrite("DebugFiles/TIAFC_img_proc_3(greyscale).jpg", greyscaleImage);

	//blur
	Mat blurImage;
	blur(greyscaleImage, blurImage, Size(3,3));
	if (TIAFC_MODE) //DEBUG
		imwrite("DebugFiles/TIAFC_img_proc_4(blur).jpg", blurImage);

	//apply threshold
	Mat thresImage;
	threshold(blurImage, thresImage, THRESHOLDING_THRES, 255, 0);
	if (TIAFC_MODE) //DEBUG
		imwrite("DebugFiles/TIAFC_img_proc_5(thres).jpg", thresImage);

	//detect contours
	Mat edgeImage;
	Canny(thresImage, edgeImage, CANNY_THRES, CANNY_THRES*2);
	if (TIAFC_MODE) //DEBUG
		imwrite("DebugFiles/TIAFC_img_proc_6(edge).jpg", edgeImage);

	//save
	//edgeImage.copyTo(aContourImage);

	//find contours
	//vector<vector<Point> > contours;
	//findContours( edgeImage, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

	//init "aContourImage"
	//Mat blank(aCropImage.rows, aCropImage.cols, CV_8UC1, Scalar(0, 0, 0));
	//blank.copyTo(aContourImage);

	//save sufficiently large contours
	/*for (int i = 0; i < contours.size(); i++) //for all contours
	{
		if (contours[i].size() >= MIN_POINTS_IN_CONTS) //if the contour is large enough
		{
			drawContours( aContourImage, contours, i, 255, 1, LINE_8);

			for (int j = 0; j < contours[i].size(); j++) //run through all the points in the contour
			{
				aContourImage.at<uchar>(contours[i][j]) = 255; //set the point
			}
		}
	}*/

	/*if (TIAFC_MODE) //DEBUG
		imwrite("DebugFiles/TIAFC_img_proc_6(mod_edge).jpg", aContourImage);*/

	//find first point
	bool foundFirst = false;
	Coords firstPoint;

	for (int i = 0; i < edgeImage.rows; i++)
	{
		for (int j = 0; j < edgeImage.cols; j++)
		{
			if (edgeImage.at<uchar>(i, j) == 255)
			{
				firstPoint.Set(j, i);
				foundFirst = true;
				break;
			}
		}

		if (foundFirst == true)
			break;
	}

	if(foundFirst != true) //check that a contour was found
		throw("[TIAFC::FindContour()]: No contour detected!");

	//init contour
	aContourList.resize(1);

	aContourMatrix.resize(edgeImage.cols);
	for (int i = 0; i < edgeImage.cols; i++)
	{
		aContourMatrix[i] = vector<int>(edgeImage.rows, 0);
	}

	//add the first point to the contour
	aContourList[0] = firstPoint;
	aContourMatrix[firstPoint.x][firstPoint.y] = 1;

	//find outer contour
	int index = 1;
	Coords nextPoint;
	while (1) //do until we return to start
	{
		//find next point
		nextPoint = FindNextNeighbour(edgeImage, aContourMatrix, aContourList[index - 1]);

		if (nextPoint.Eq(firstPoint)) //if we return to start
			break;			

		//add it to the contour
		aContourList.push_back(nextPoint);
		aContourMatrix[aContourList[index].x][aContourList[index].y] = index + 1;
		index++;
	}

	//init contour
	Mat blank(aCropImage.rows, aCropImage.cols, CV_8UC1, Scalar(0, 0, 0));
	blank.copyTo(aContourImage);

	//copy points to contour
	for (int i = 0; i < aContourList.size(); i++)
		aContourImage.at<uchar>(aContourList[i].y, aContourList[i].x) = 255;

	if (TIAFC_MODE) //DEBUG
		imwrite("DebugFiles/TIAFC_img_proc_7(outer_edge).jpg", aContourImage);
}

Coords TIAFC::FindNextNeighbour(Mat &aContourImage, vector<vector<int> > &aContourMatrix, Coords aCurrPoint)
{
	//relative coordinates (this EXACT sequence is very important!!!):
	int xVals[24] = {/*1. ring*/ 1, 0, -1, 0, 1, -1, -1, 1, /*2. ring*/ 2, 0, -2, 0, 2, 1, -1, -2, -2, -1, 1, 2, 2, -2, -2, 2};
	int yVals[24] = {/*1. ring*/ 0, -1, 0, 1, -1, -1, 1, 1, /*2. ring*/ 0, -2, 0, 2, -1, -2, -2, -1, 1, 2, 2, 1, -2, -2, 2, 2};

	for ( int i = 0; i < 24; i++) //run through all neighbours
	{
		if (aContourImage.at<uchar>(aCurrPoint.y + yVals[i], aCurrPoint.x + xVals[i]) == 255) //if part of the contour
		{
			if (aContourMatrix[aCurrPoint.x + xVals[i]][aCurrPoint.y + yVals[i]] == 0) //if not one of the previous points
			{
				return Coords(aCurrPoint.x + xVals[i], aCurrPoint.y + yVals[i]);
			}
		}
	}

	//if we couldn't find any unvisited neighbours
	for ( int i = 0; i < 24; i++) //run through all neighbours
	{
		if (aContourImage.at<uchar>(aCurrPoint.y + yVals[i], aCurrPoint.x + xVals[i]) == 255) //if part of the contour
		{
			if (aContourMatrix[aCurrPoint.x + xVals[i]][aCurrPoint.y + yVals[i]] == 1) //if the first point
			{
				return Coords(aCurrPoint.x + xVals[i], aCurrPoint.y + yVals[i]);
			}
		}
	}

	//error
	throw("[TIAFC::FindNextNeighbour()]: Contour is not closed!");
}

TIAFC::TIAFC()
{
}

TIAFC::~TIAFC()
{
}

