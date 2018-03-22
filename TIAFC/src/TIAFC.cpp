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
	for (int i = 0; i < FRAMES_TO_GET ; i++)
	{
		cap.read(frame);
	}

	/*DEBUG*/frame = imread("test_img_5.jpg");

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
		imwrite("TIAFC_imgproc_1(image).jpg", frame);
		imwrite("TIAFC_imgproc_2(crop).jpg", aCropImage);
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
		imwrite("TIAFC_imgproc_3(greyscale).jpg", greyscaleImage);

	//blur
	Mat blurImage;
	blur(greyscaleImage, blurImage, Size(3,3));
	if (TIAFC_MODE) //DEBUG
		imwrite("TIAFC_imgproc_4(blur).jpg", blurImage);

	//detect contours
	Mat edgeImage;
	Canny(blurImage, edgeImage, CANNY_THRES, CANNY_THRES*2);
	if (TIAFC_MODE) //DEBUG
		imwrite("TIAFC_imgproc_5(edge).jpg", edgeImage);

	//save
	edgeImage.copyTo(aContourImage);

	//find number of points and first point
	bool firstTime = true;
	Coords firstPoint;
	int pointCount = 0;

	for (int i = 0; i < edgeImage.rows; i++)
	{
		for (int j = 0; j < edgeImage.cols; j++)
		{
			if (edgeImage.at<uchar>(i, j) == 255)
			{
				if (firstTime)
				{
					firstPoint.Set(j, i);
				}
				pointCount++;
			}
		}
	}

	if(pointCount < 1) //check that a contour was found
		throw("[TIAFC::FindContour()]: No contour detected!");

	//init contour
	aContourList.resize(pointCount);
	aContourList[0] = firstPoint;

	aContourMatrix.resize(aContourImage.cols);
	for (int i = 0; i < aContourImage.cols; i++)
	{
		aContourMatrix[i] = vector<int>(aContourImage.rows, 0);
	}
	aContourMatrix[firstPoint.x][firstPoint.y] = 1;

	/*pointCount = 0;
	for (int i = 0; i < edgeImage.rows; i++)
	{
		for (int j = 0; j < edgeImage.cols; j++)
		{
			if (edgeImage.at<uchar>(i, j) == 255)
			{
				aContourList[pointCount].Set(j, i);
				pointCount++;
			}
		}
	}*/

	//copy points to contour
	int index = 1;
	while (index < pointCount) //while we haven't found all points
	{
		//find next point
		aContourList[index] = FindNextNeighbour(aContourImage, aContourMatrix, aContourList[index - 1]);
		aContourMatrix[aContourList[index].x][aContourList[index].y] = index;

		index++;
	}
}

Coords TIAFC::FindNextNeighbour(Mat &aContourImage, vector<vector<int> > &aContourMatrix, Coords aCurrPoint)
{
	Coords res(-42);

	//relative coordinates:
	int xVals[8] = {1, 0, -1, 0, 1, -1, -1, 1}; //this EXACT sequence is very important!!!
	int yVals[8] = {0, -1, 0, 1, -1, -1, 1, 1};

	for ( int i = 0; i < 8; i++) //run through all neighbours
	{
		if (aContourImage.at<uchar>(aCurrPoint.y + yVals[i], aCurrPoint.x + xVals[i]) == 255) //if part of the contour
		{
			if (aContourMatrix[aCurrPoint.x + xVals[i]][aCurrPoint.y + yVals[i]] == 0) //if not one of the previous points
			{
				res.Set(aCurrPoint.x + xVals[i], aCurrPoint.y + yVals[i]);
				break;
			}
		}
	}

	if(res.Eq(-42)) //check that a neighbour was found
		throw("[TIAFC::FindNextNeighbour()]: Contour is not closed!");

	return res;
}

TIAFC::TIAFC()
{
}

TIAFC::~TIAFC()
{
}

