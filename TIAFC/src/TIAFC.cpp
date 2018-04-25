#include "TIAFC.hpp"
#include <iostream>

Coords TIAFC::FindCOM(Contour &aContour)
{
	//calculate sum of all x- and y-coordinates
	Coords sum(0, 0);
	for (int i = 0; i < aContour.list.size(); i++)
	{
		sum = sum.Add(aContour.list[i]);
	}

	//divide by number of points to find average
	Coords res = sum.Div(aContour.list.size());

	return res;
}


void TIAFC::DoItAll(Mat &aCropImage, Contour &aContour)
{
	TakeImage(aCropImage);

	Mat objectImage;
	FindObject(aCropImage, objectImage);

	FindContour(objectImage, aContour);
}

void TIAFC::TakeImage(Mat &aCropImage)
{
	//open selected camera
	VideoCapture cap;
	cap.open(CAMERA);

	if (!cap.isOpened()) //check that camera was opened
		throw("[TIAFC::TakeImage()]: Couldn't open camera!");

	//set camera resolution
	cap.set(CV_CAP_PROP_FRAME_WIDTH, CAM_RES_WIDTH);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, CAM_RES_HEIGHT);

	//get frame
	Mat frame;
	if (VISION_DEV_MODE)
	{
		frame = imread("TestImg/box.jpg");
	}
	else
	{
		for (int i = 0; i < FRAMES_TO_GET ; i++)
		{
			cap.read(frame);
		}
	}

	if (!frame.data) //check if empty
		throw("[TIAFC::TakeImage()]: Couldn't take image!");

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

	if (TIAFC_MODE) //INFO
	{
		imwrite("InfoFiles/TIAFC(img_proc_1)(image).jpg", frame);
		imwrite("InfoFiles/TIAFC(img_proc_2)(crop).jpg", aCropImage);
	}
}

void TIAFC::FindObject(Mat &aCropImage, Mat &anObjectImage)
{
	//make greyscale image
	Mat greyscaleImage;
	cvtColor(aCropImage, greyscaleImage, COLOR_BGR2GRAY);
	if (TIAFC_MODE) //INFO
		imwrite("InfoFiles/TIAFC(img_proc_3)(greyscale).jpg", greyscaleImage);

	//blur
	Mat blurImage;
	blur(greyscaleImage, blurImage, Size(3,3));
	if (TIAFC_MODE) //INFO
		imwrite("InfoFiles/TIAFC(img_proc_4)(blur).jpg", blurImage);

	//apply threshold
	Mat thresImage;
	threshold(blurImage, thresImage, THRESHOLDING_THRES, 255, THRESHOLDING_MODE);
	if (TIAFC_MODE) //INFO
		imwrite("InfoFiles/TIAFC(img_proc_5)(thres).jpg", thresImage);

	//remove holes
	ThereCanOnlyBeOne(thresImage, 0);
	if (TIAFC_MODE) //INFO
		imwrite("InfoFiles/TIAFC(img_proc_6)(holes).jpg", thresImage);

	//remove outliers
	ThereCanOnlyBeOne(thresImage, 1);
	if (TIAFC_MODE) //INFO
		imwrite("InfoFiles/TIAFC(img_proc_7)(outliers).jpg", thresImage);

	//save
	thresImage.copyTo(anObjectImage);
}

void TIAFC::ThereCanOnlyBeOne(Mat &aThresImage, bool mode)
{
	//determine the color we are looking for
	int color = 0;
	if (mode)
		color = 255;

	//find all points of all blobs of the color
	vector<vector<Coords> > blobPoints;

	for (int i = 0; i < aThresImage.rows; i++) //run through the image
	{
		for (int j = 0; j < aThresImage.cols; j++) //run through the image
		{
			if (aThresImage.at<uchar>(i, j) == color) //if the point is of the color
			{
				//make a new blob
				aThresImage.at<uchar>(i, j) = 255 - color; //remove the point
				vector<Coords> temp = {Coords(j, i)}; //add it to the list

				for (int k = 0; k < temp.size(); k++) //for all points part of this blob
				{
					CatchNeighbours(aThresImage, color, temp[k], temp);
				}

				//when we get to here, we must have gotten all points part of this blob
				blobPoints.push_back(temp); //add it to the list of blobs
			}
		}
	}

	if (blobPoints.size() < 1)
		throw("[TIAFC::ThereCanOnlyBeOne()]: Didn't find any blobs!");

	//find the biggest blob
	int biggestBlobIndex;
	int currBiggestBlobSize = 0;
	for (int i = 0; i < blobPoints.size(); i++)
	{
		if (blobPoints[i].size() < 1)
			throw("[TIAFC::ThereCanOnlyBeOne()]: Empty blob!");

		if (blobPoints[i].size() > currBiggestBlobSize)
		{
			currBiggestBlobSize = blobPoints[i].size();
			biggestBlobIndex = i;
		}
	}

	//put the biggest blob back again
	for (int i = 0; i < blobPoints[biggestBlobIndex].size(); i++)
	{
		aThresImage.at<uchar>(blobPoints[biggestBlobIndex][i].y, blobPoints[biggestBlobIndex][i].x) = color;
	}
}

void TIAFC::CatchNeighbours(Mat &aThresImage, int aColor, Coords aCurrPoint, vector<Coords> &aListOfPoints)
{
	//relative coordinates:
	int xVals[4] = {1, 0, -1, 0}; //use four-connectivity. this should make the blobs less "hairy"
	int yVals[4] = {0, -1, 0, 1};

	for ( int i = 0; i < 4; i++) //run through all neighbours
	{
		if (IsWithinBounds(aThresImage, aCurrPoint.x + xVals[i], aCurrPoint.y + yVals[i]))
		{
			if (aThresImage.at<uchar>(aCurrPoint.y + yVals[i], aCurrPoint.x + xVals[i]) == aColor) //if the neighbour is the color
			{
				aThresImage.at<uchar>(aCurrPoint.y + yVals[i], aCurrPoint.x + xVals[i]) = 255 - aColor; //remove the point
				aListOfPoints.push_back(Coords(aCurrPoint.x + xVals[i], aCurrPoint.y + yVals[i])); //add it to the list
			}
		}
	}
}

void TIAFC::FindContour(Mat &objectImage, Contour &aContour)
{
	//find first point
	bool foundFirst = false;
	Coords firstPoint;

	for (int i = 0; i < objectImage.rows; i++)
	{
		for (int j = 0; j < objectImage.cols; j++)
		{
			if (objectImage.at<uchar>(i, j) == 255)
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
	aContour.list.resize(1);

	aContour.matrix.resize(objectImage.cols);
	for (int i = 0; i < objectImage.cols; i++)
	{
		aContour.matrix[i] = vector<int>(objectImage.rows, 0);
	}

	//add the first point to the contour
	aContour.list[0] = firstPoint;
	aContour.matrix[firstPoint.x][firstPoint.y] = 1;

	//find outer contour
	int index = 1;
	Coords nextPoint;
	while (1) //do until we return to start
	{
		//find next point
		nextPoint = FindNextNeighbour(objectImage, aContour, aContour.list[index - 1]);

		if (nextPoint.Eq(firstPoint)) //if we return to start
			break;

		//add it to the contour
		aContour.list.push_back(nextPoint);
		aContour.matrix[nextPoint.x][nextPoint.y] = index + 1;
		index++;
	}

	//init contour
	Mat blank(objectImage.rows, objectImage.cols, CV_8UC1, Scalar(0, 0, 0));
	blank.copyTo(aContour.image);

	//copy points to contour
	for (int i = 0; i < aContour.list.size(); i++)
		aContour.image.at<uchar>(aContour.list[i].y, aContour.list[i].x) = 255;

	if (TIAFC_MODE) //INFO
		imwrite("InfoFiles/TIAFC(img_proc_8)(outer_edge).jpg", aContour.image);
}

Coords TIAFC::FindNextNeighbour(Mat &anObjectImage, Contour &aContour, Coords aCurrPoint)
{
	//relative coordinates (this EXACT sequence is important!):
	int xVals[8] = {1, 0, -1, 0, 1, -1, -1, 1}; //sides first, otherwise we can get stuck
	int yVals[8] = {0, -1, 0, 1, -1, -1, 1, 1};

	for (int i = 0; i < 8; i++) //run through all neighbours
	{
		if (IsWithinBounds(anObjectImage, aCurrPoint.x + xVals[i], aCurrPoint.y + yVals[i]))
		{
			if (anObjectImage.at<uchar>(aCurrPoint.y + yVals[i], aCurrPoint.x + xVals[i]) == 255) //if part of the object
			{
				if (aContour.matrix[aCurrPoint.x + xVals[i]][aCurrPoint.y + yVals[i]] == 0) //if not one of the previous points
				{
					//if it has a black neighbour
					if (HasBlackNeighbour(anObjectImage, Coords(aCurrPoint.x + xVals[i], aCurrPoint.y + yVals[i])))
					{
						return Coords(aCurrPoint.x + xVals[i], aCurrPoint.y + yVals[i]);
					}
				}
			}
		}
	}

	//if we couldn't find any unvisited neighbours
	for (int i = 0; i < 8; i++) //run through all neighbours
	{
		if (IsWithinBounds(anObjectImage, aCurrPoint.x + xVals[i], aCurrPoint.y + yVals[i]))
		{
			if (anObjectImage.at<uchar>(aCurrPoint.y + yVals[i], aCurrPoint.x + xVals[i]) == 255) //if part of the object
			{
				if (aContour.matrix[aCurrPoint.x + xVals[i]][aCurrPoint.y + yVals[i]] == 1) //if the first point
				{
					return Coords(aCurrPoint.x + xVals[i], aCurrPoint.y + yVals[i]);
				}
			}
		}
	}

	//error
	throw("[TIAFC::FindNextNeighbour()]: Couldn't follow the contour all the way around!");
}

bool TIAFC::HasBlackNeighbour(Mat &anObjectImage, Coords aCurrPoint)
{
	//relative coordinates:
	int xVals[8] = {1, 0, -1, 0, 1, -1, -1, 1};
	int yVals[8] = {0, -1, 0, 1, -1, -1, 1, 1};

	for (int i = 0; i < 8; i++) //run through all neighbours
	{
		if (IsWithinBounds(anObjectImage, aCurrPoint.x + xVals[i], aCurrPoint.y + yVals[i]))
		{
			if (anObjectImage.at<uchar>(aCurrPoint.y + yVals[i], aCurrPoint.x + xVals[i]) == 0) //if the neighbour is black
			{
				return true;
			}
		}
		else
		{
			return true; //the edge of the image counts as black!
		}
	}

	return false;
}

int TIAFC::NumOfWhiteNeighbours(Mat &anObjectImage, Coords aCurrPoint)
{
	//relative coordinates:
	int xVals[8] = {1, 0, -1, 0, 1, -1, -1, 1};
	int yVals[8] = {0, -1, 0, 1, -1, -1, 1, 1};

	int res = 0;

	for (int i = 0; i < 8; i++) //run through all neighbours
	{
		if (IsWithinBounds(anObjectImage, aCurrPoint.x + xVals[i], aCurrPoint.y + yVals[i]))
		{
			if (anObjectImage.at<uchar>(aCurrPoint.y + yVals[i], aCurrPoint.x + xVals[i]) == 255) //if the neighbour is white
			{
				res++;
			}
		}
	}

	return res;
}

bool TIAFC::IsWithinBounds(Mat &anImage, int anX, int aY)
{
	if (anX < anImage.cols && anX > -1)
	{
		if (aY < anImage.rows && aY > -1)
		{
			return true;
		}
	}

	return false;
}

TIAFC::TIAFC()
{
}

TIAFC::~TIAFC()
{
}
