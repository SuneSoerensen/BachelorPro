#include "AnalytGrasp.hpp"

void AnalytGrasp::FindGraspPoints(Mat &aContourImage, vector<Coords> &aContourList, vector<vector<int> > &aContourMatrix)
{
	//find grasping regions
	vector<vector<Coords> > graspRegsList;
	FindGraspRegs(graspRegsList, aContourImage, aContourMatrix);

	//calc their normal vectors
	vector<Coords> normVecsList;
	CalcNormVecs(graspRegsList, normVecsList);

	vector<vector<int> > possGraspsList;
	ThreeFingAngCheck(possGraspsList, normVecsList);
	TwoFingAngCheck(possGraspsList, normVecsList);

	if (possGraspsList.size() == 0) //check that possible sets of grasp regions were found
		throw("[AnalytGrasp::FindGraspPoints()]: Couldn't find any sets of grasp regions!");

	cout << "[AnalytGrasp::FindGraspPoints()]: Possible grasps:" << endl;
	for (int i = 0; i < possGraspsList.size(); i++)
	{
		for (int j = 0; j < possGraspsList[i].size(); j++)
		{
			cout << possGraspsList[i][j];
			if (j != possGraspsList[i].size() - 1)
				cout << ", ";
		}
		cout << endl;
	}
	cout << endl;
}

Coords AnalytGrasp::FindCOM(vector<Coords> &aContourList, Mat &aContourImage)
{
	//calculate sum of all x- and y-coordinates
	Coords sum(0, 0);
	for (int i = 0; i < aContourList.size(); i++)
	{
		sum = sum.Add(aContourList[i]);
	}

	//divide by number of points to find average
	Coords res = sum.Div(aContourList.size());

	if (ANALYT_GRASP_MODE) //DEBUG
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
		imwrite("DebugFiles/AnalytGrasp_centOfMass.jpg", centOfMassImage);
	}

	return res;
}

void AnalytGrasp::FindGraspRegs(vector<vector<Coords> > &aGraspRegsList, Mat &aContourImage, vector<vector<int> > &aContourMatrix)
{
	//use Hough-transform to find straight lines
	vector<Vec4i> lines;
	HoughLinesP(aContourImage, lines, 1, CV_PI/180, MIN_POINTS_IN_LINE, MIN_LINE_LENGTH, MAX_LINE_GAP );

	if (lines.size() == 0) //check that lines were found
		throw("[AnalytGrasp::FindGraspRegs()]: Couldn't find any grasp regions!");

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

	if (ANALYT_GRASP_MODE) //DEBUG
	{
		//draw all the lines:
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
		imwrite("DebugFiles/AnalytGrasp_graspRegs.jpg", graspRegsImage);

		//draw all the lines individually:
		/*for( int i = 0; i < lines.size(); i++ )
		{
			aContourImage.copyTo(graspRegsImage);
			cvtColor(graspRegsImage, graspRegsImage, COLOR_GRAY2BGR);

			Point startP = Point(lines[i][0], lines[i][1]);
			Point endP = Point(lines[i][2], lines[i][3]);
			line( graspRegsImage, startP, endP, Scalar(0,0,255), 1, LINE_AA);

			imwrite("DebugFiles/AnalytGrasp_graspReg_" + to_string(i) + ".jpg", graspRegsImage);
		}*/
	}
}

void AnalytGrasp::CalcNormVecs(vector<vector<Coords> > &aGraspRegsList, vector<Coords> &aNormVecsList)
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

double AnalytGrasp::CalcAngle(Coords vecA, Coords vecB)
{
	return acos(vecA.Dot(vecB) / (vecA.Length()*vecB.Length()));
}

void AnalytGrasp::ThreeFingAngCheck(vector<vector<int> > &aPossGraspsList, vector<Coords> &aNormVecsList)
{
	double angAC, angBC, angBA;
	double minAng, maxAng;

	for (int a = 0; a < aNormVecsList.size(); a++) //for all normal vectors
	{
		for (int b = 0; b < aNormVecsList.size(); b++)
		{
			if (b != a) //find another one, which is not the first
			{
				for (int c = 0; c < aNormVecsList.size(); c++)
				{
					if (c != b && c != a) //find a third, which is not one of the two others
					{
						angAC = CalcAngle(aNormVecsList[a], aNormVecsList[c]); //calc the angle between the two fingers

						if ( 0 <= angAC && angAC <= MAX_THREE_FING_ANG) //if it is valid
						{
							//calc the angles defining the tolerable range
							minAng = ((6.28 - angAC)/2) - MAX_DEV_ANG;
							maxAng = ((6.28 - angAC)/2) + MAX_DEV_ANG;

							angBC = CalcAngle(aNormVecsList[b], aNormVecsList[c]); //calc the angle between the thumb and second finger

							if (minAng <= angBC && angBC <= maxAng) //if the angle is within the range
							{
								angBA = CalcAngle(aNormVecsList[b], aNormVecsList[a]); //calc the angle between the thumb and first finger

								if (minAng <= angBA && angBA <= maxAng) //if the angle is within the range
								{
									aPossGraspsList.push_back({a, b, c}); //we have found possible grasp!
								}
							}
						}
					}
				}
			}
		}
	}
}

void AnalytGrasp::TwoFingAngCheck(vector<vector<int> > &aPossGraspsList, vector<Coords> &aNormVecsList)
{
	double angAC;
	double minAng, maxAng;

	for (int a = 0; a < aNormVecsList.size(); a++) //for all normal vectors
	{
		for (int c = 0; c < aNormVecsList.size(); c++)
		{
			if (c != a) //find another one which is not the first
			{
				angAC = CalcAngle(aNormVecsList[a], aNormVecsList[c]); //calc the angle between the two fingers

				//calc angles defining the tolerable range
				minAng = 3.14 - MAX_DEV_ANG;
				maxAng = 3.14 + MAX_DEV_ANG;

				if ( minAng <= angAC && angAC <= maxAng) //if the angle is within the range
				{
					aPossGraspsList.push_back({a, c}); //we have found a possible grasp!
				}
			}
		}
	}
}

AnalytGrasp::AnalytGrasp()
{
}

AnalytGrasp::~AnalytGrasp()
{
}

