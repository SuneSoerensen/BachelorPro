#include "TIAFC.hpp"



void TIAFC::TakeImage(Mat &aCropImage)
{
	//open selected camera
	VideoCapture cap;
	cap.open(CAMERA);

	if (!cap.isOpened()) //check that camera was opened
		throw("[TIAFC::TakeImage()]: Couldn't open camera!");

	//get frame
	Mat frame;
	for (int i = 0; i < FRAMES_TO_GET ; i++)
	{
		cap.read(frame);
	}

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

void TIAFC::FindContour(Mat &aCropImage, Mat &aContourImage, vector<Coords> &aContour)
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

	if(pointCount == 0) //check that a contour was found
		throw("[TIAFC::FindContour()]: No contour detected!");

	aContour.resize(pointCount); //init result

	//copy points
	/*pointCount = 0;
	for (int i = 0; i < edgeImage.rows; i++)
	{
		for (int j = 0; j < edgeImage.cols; j++)
		{
			if (edgeImage.at<uchar>(i, j) == 255)
			{
				aContour[pointCount].Set(j, i);
				pointCount++;
			}
		}
	}*/

	Coords prevPoint = firstPoint;
	Coords currPoint = FindNextNeighbour(aContourImage, firstPoint);
	int index = 0;

	while (index < pointCount)
	{
		
	}
}

Coords TIAFC::FindNextNeighbour(Mat &aContourImage, Coords aPrevPoint)
{
	
}

TIAFC::TIAFC()
{
}

TIAFC::~TIAFC()
{
}
