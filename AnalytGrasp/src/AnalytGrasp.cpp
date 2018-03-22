#include "AnalytGrasp.hpp"



void AnalytGrasp::FindGraspPoints(vector<Coords> &aGraspPointsList, Mat &aContourImage, vector<Coords> &aContourList, vector<vector<int> > &aContourMatrix)
{
	vector<vector<Coords> > graspRegsList;
	FindGraspRegs(graspRegsList, aContourImage, aContourMatrix);

	vector<Coords> normVecsList;
	CalcNormVecs(graspRegsList, normVecsList);

	vector<vector<int> > possGraspsList;
	ThreeFingAngCheck(possGraspsList, normVecsList);
	TwoFingAngCheck(possGraspsList, normVecsList);

	if (possGraspsList.size() == 0) //check that possible sets of grasp regions were found
		throw("[AnalytGrasp::FindGraspPoints()]: Couldn't find any sets of grasp regions!");

	cout << "Possible grasps:" << endl;
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
		imwrite("AnalytGrasp_graspRegs.jpg", graspRegsImage);

		//draw all the lines individually:
		for( int i = 0; i < lines.size(); i++ )
		{
			aContourImage.copyTo(graspRegsImage);
			cvtColor(graspRegsImage, graspRegsImage, COLOR_GRAY2BGR);

			Point startP = Point(lines[i][0], lines[i][1]);
			Point endP = Point(lines[i][2], lines[i][3]);
			line( graspRegsImage, startP, endP, Scalar(0,0,255), 1, LINE_AA);

			imwrite("AnalytGrasp_graspReg_" + to_string(i) + ".jpg", graspRegsImage);
		}
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
	double angAB, angBC, angCA;
	double minAng, maxAng;

	for (int a = 0; a < aNormVecsList.size(); a++)
	{
		for (int b = 0; b < aNormVecsList.size(); b++)
		{
			if (b != a)
			{
				for (int c = 0; c < aNormVecsList.size(); c++)
				{
					if (c != b && c != a)
					{
						angAB = CalcAngle(aNormVecsList[a], aNormVecsList[b]);

						if ( 0 <= angAB && angAB <= MAX_THREE_FING_ANG)
						{
							minAng = ((6.28 - angAB)/2) - MAX_DEV_ANG;
							maxAng = ((6.28 - angAB)/2) + MAX_DEV_ANG;

							angBC = CalcAngle(aNormVecsList[b], aNormVecsList[c]);

							if (minAng <= angBC && angBC <= maxAng)
							{
								angCA = CalcAngle(aNormVecsList[c], aNormVecsList[a]);

								if (minAng <= angCA && angCA <= maxAng)
								{
									vector<int> temp(3, 0);
									temp[0] = a;
									temp[1] = b;
									temp[2] = c;

									aPossGraspsList.push_back(temp);
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
	double angAB;
	double minAng, maxAng;

	for (int a = 0; a < aNormVecsList.size(); a++)
	{
		for (int b = 0; b < aNormVecsList.size(); b++)
		{
			if (b != a)
			{
				angAB = CalcAngle(aNormVecsList[a], aNormVecsList[b]);

				minAng = 3.14 - MAX_DEV_ANG;
				maxAng = 3.14 + MAX_DEV_ANG;

				if ( minAng <= angAB && angAB <= maxAng)
				{
					vector<int> temp(2, 0);
					temp[0] = a;
					temp[1] = b;

					aPossGraspsList.push_back(temp);
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

