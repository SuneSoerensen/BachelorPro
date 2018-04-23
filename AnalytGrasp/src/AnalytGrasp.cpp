#include "AnalytGrasp.hpp"

Grasp AnalytGrasp::FindGrasp(Contour &aContour, double aScaleFactor)
{
	//find grasping regions
	vector<vector<Coords> > graspRegsList;
	FindGraspRegs(graspRegsList, aContour, aScaleFactor);

	//calc their normal vectors
	vector<Coords> normVecsList;
	CalcNormVecs(graspRegsList, normVecsList);

	//calc the objects center of mass
	Coords COM = TIAFC::FindCOM(aContour);

	//calc possible priority 1 grasps
	vector<Grasp> p1GraspsList;
	CalcP1Grasps(p1GraspsList, graspRegsList, normVecsList, COM);

	if (p1GraspsList.size() > 0)
	{
		//find the best p1 grasp
		double currShortestDist = INFINITY;
		int currBestP1Grasp;
		for (int i = 0; i < p1GraspsList.size(); i++)
		{
			if (p1GraspsList[i].distFromCOM < currShortestDist)
			{
				currShortestDist = p1GraspsList[i].distFromCOM;
				currBestP1Grasp = i;
			}
		}
		/*TEMP*/p1GraspsList[currBestP1Grasp].COM = COM;
		return p1GraspsList[currBestP1Grasp];
	}

	//calc possible priority 2 grasps


	//calc possible priority 3 grasps


	//if we get to here, we couldn't find any accetable grasps!
	throw("[AnalytGrasp::FindGrasP()]: Couldn't find any acceptable grasps!");
}

void AnalytGrasp::FindGraspRegs(vector<vector<Coords> > &aGraspRegsList, Contour &aContour, double aScaleFactor)
{
	int regWidth = GRASP_REG_WIDTH / aScaleFactor; //mm / (mm / pixels) = mm * (pixels / mm) = pixels

	for (int i = 0; i < aContour.list.size(); i++) //for all points in the contour
	{
		//find start and end points (of the grasp reg)
		int startIndex = (aContour.list.size() + i - (regWidth/2)) % aContour.list.size(); //wrap
		int endIndex = (i + (regWidth/2)) % aContour.list.size(); //wrap

		//calc normal vector
		Coords dirVec = (aContour.list[endIndex]).Sub(aContour.list[startIndex]); //r = p_end - p_start
		Coords normVec = Coords(dirVec.y, -dirVec.x); //n = (y, -x), r = (x, y)

		for (int j = startIndex; j <= endIndex; j++)
		{
			
		}
	}





	//use Hough-transform to find straight lines
	vector<Vec4i> lines;
	HoughLinesP(aContour.image, lines, 1, CV_PI/180, MIN_POINTS_IN_LINE, MIN_LINE_LENGTH, MAX_LINE_GAP);

	if (lines.size() == 0) //check that lines were found
		throw("[AnalytGrasp::FindGraspRegs()]: Couldn't find any grasp regions!");

	//copy found lines to the list of grasping regions
	aGraspRegsList.resize(lines.size());
	for (int i = 0; i < lines.size(); i++)
	{
		//make sure the order is correct
		int firstPointIndex;
		int secondPointIndex;

		if (aContour.matrix[lines[i][0]][lines[i][1]] < aContour.matrix[lines[i][2]][lines[i][3]])
		{
			firstPointIndex = aContour.matrix[lines[i][0]][lines[i][1]] - 1; //indices in the contour matrix is offset by +1!
			secondPointIndex = aContour.matrix[lines[i][2]][lines[i][3]] - 1;
		}
		else
		{
			firstPointIndex = aContour.matrix[lines[i][2]][lines[i][3]] - 1; //indices in the contour matrix is offset by +1!
			secondPointIndex = aContour.matrix[lines[i][0]][lines[i][1]] - 1;
		}

		int startPointIndex;
		int endPointIndex;

		if (secondPointIndex - firstPointIndex < (aContour.list.size() + firstPointIndex) - secondPointIndex) //determine the shortest way round between the points
		{
			startPointIndex = firstPointIndex;
			endPointIndex = secondPointIndex;

			aGraspRegsList[i].resize(secondPointIndex - firstPointIndex + 1);
		}
		else
		{
			startPointIndex = secondPointIndex;
			endPointIndex = firstPointIndex;

			aGraspRegsList[i].resize((aContour.list.size() + firstPointIndex) - secondPointIndex + 1);
		}

		//copy points to the list of grasping regions
		int index = 0;
		for (int j = startPointIndex; j != endPointIndex + 1; j = (j + 1) % aContour.list.size())
		{
			aGraspRegsList[i][index] = aContour.list[j];
			index++;
		}
	}

	if (ANALYT_GRASP_MODE) //INFO
	{
		//draw all the lines:
		//make output image
		Mat graspRegsImage;
		aContour.image.copyTo(graspRegsImage);
		cvtColor(graspRegsImage, graspRegsImage, COLOR_GRAY2BGR);

		//draw the found lines
		Vec3b color; //drawing color
		color.val[0] = 0; //blue
		color.val[1] = 0; //green
		color.val[2] = 255; //red

		for(int i = 0; i < aGraspRegsList.size(); i++ )
		{
			for(int j = 0; j < aGraspRegsList[i].size(); j++ )
			{
				graspRegsImage.at<Vec3b>(aGraspRegsList[i][j].y, aGraspRegsList[i][j].x) = color;
			}
		}

		//save to image
		imwrite("InfoFiles/AnalytGrasp(grasp_regs).jpg", graspRegsImage);

		//draw all the lines individually:
		for(int i = 0; i < aGraspRegsList.size(); i++ )
		{
			aContour.image.copyTo(graspRegsImage);
			cvtColor(graspRegsImage, graspRegsImage, COLOR_GRAY2BGR);

			for(int j = 0; j < aGraspRegsList[i].size(); j++ )
			{
				if (j < (aGraspRegsList[i].size() - 1) / 2)
				{
					Vec3b color; //drawing color
					color.val[0] = 0; //blue
					color.val[1] = 255; //green
					color.val[2] = 0; //red

					graspRegsImage.at<Vec3b>(aGraspRegsList[i][j].y, aGraspRegsList[i][j].x) = color;
				}
				else
				{
					Vec3b color; //drawing color
					color.val[0] = 0; //blue
					color.val[1] = 0; //green
					color.val[2] = 255; //red

					graspRegsImage.at<Vec3b>(aGraspRegsList[i][j].y, aGraspRegsList[i][j].x) = color;
				}
			}

			imwrite("InfoFiles/AnalytGrasp(grasp_reg_" + to_string(i) + ").jpg", graspRegsImage);
		}
	}
}

void AnalytGrasp::CalcNormVecs(vector<vector<Coords> > &aGraspRegsList, vector<Coords> &aNormVecsList)
{
	//calc direction-vectors
	vector<Coords> dirVecsList;
	dirVecsList.resize(aGraspRegsList.size());
	for (int i = 0; i < aGraspRegsList.size(); i++)
	{
		dirVecsList[i] = (aGraspRegsList[i][aGraspRegsList[i].size() - 1]).Sub(aGraspRegsList[i][0]); //r = p_end - p_start
	}

	//calc normal-vectors
	aNormVecsList.resize(aGraspRegsList.size());
	for (int i = 0; i < aNormVecsList.size(); i++)
	{
		aNormVecsList[i].Set(dirVecsList[i].y, -dirVecsList[i].x); //n = (y, -x), r = (x, y)
	}
}

void AnalytGrasp::CalcP1Grasps(vector<Grasp> &aP1GraspsList, vector<vector<Coords> > &aGraspRegsList, vector<Coords> aNormVecsList, Coords aCOM)
{
	vector<vector<int> > passedAngCheckList;
	P1AngCheck(passedAngCheckList, aNormVecsList);

	/*DEBUG*/cout << "found " << passedAngCheckList.size() << " sets of grasp regs that passed the angle check" << endl;

	/*DEBUG*/int passedIntersecCheck = 0;
	/*DEBUG*/int passedDistFromCOMCheck = 0;

	for (int i = 0; i < passedAngCheckList.size(); i++) //for all sets of grasp regs that passed the angle check
	{
		int a = passedAngCheckList[i][0];
		int b = passedAngCheckList[i][1];
		int c = passedAngCheckList[i][2];

		for (int ai = 0; ai < aGraspRegsList[a].size(); ai++) //for all points in grasp reg a
		{
			for (int bi = 0; bi < aGraspRegsList[b].size(); bi++) //for all points in grasp reg b
			{
				for (int ci = 0; ci < aGraspRegsList[c].size(); ci++) //for all points in grasp reg c
				{
					Coords pA = aGraspRegsList[a][ai];
					Coords rA = aNormVecsList[a];

					Coords pB = aGraspRegsList[b][bi];
					Coords rB = aNormVecsList[b];

					Coords pC = aGraspRegsList[c][ci];
					Coords rC = aNormVecsList[c];

					if (P1IntersecCheck(pA, rA, pB, rB, pC, rC)) //if this specific grasp passes the intersection check
					{
						/*DEBUG*/passedIntersecCheck++;

						Coords graspFocus = ((pA.Add(pB)).Add(pC)).Div(3);
						double graspDistFromCOM = (graspFocus.Sub(aCOM)).Length();

						if (graspDistFromCOM <= MAX_DIST_FROM_COM) //check if its focus is close enough to the objects COM
						{
							/*DEBUG*/passedDistFromCOMCheck++;

							Grasp newGrasp;

							newGrasp.type = p1;
							newGrasp.points = {pA, pB, pC};
							newGrasp.focus = graspFocus;
							newGrasp.distFromCOM = graspDistFromCOM;

							aP1GraspsList.push_back(newGrasp);
						}
					}
				}
			}
		}
	}

	/*DEBUG*/cout << "found " << passedIntersecCheck << " grasps that passed the intersection check" << endl;
	/*DEBUG*/cout << "found " << passedDistFromCOMCheck << " grasps that passed the distance from COM check" << endl;
}

void AnalytGrasp::P1AngCheck(vector<vector<int> > &aPassedAngCheckList, vector<Coords> &aNormVecsList)
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

						if (MIN_P1_ANG <= angAC && angAC <= MAX_P1_ANG) //if it is valid
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
									aPassedAngCheckList.push_back({a, b, c}); //we have found a possible grasp!
								}
							}
						}
					}
				}
			}
		}
	}
}

bool AnalytGrasp::P1IntersecCheck(Coords pA, Coords rA, Coords pB, Coords rB, Coords pC, Coords rC)
{
	Coords intersecAB = CalcIntersection(pA, rA, pB, rB);
	Coords intersecBC = CalcIntersection(pB, rB, pC, rC);
	Coords intersecCA = CalcIntersection(pC, rC, pA, rA);

	if (intersecAB.Eq(-42) || intersecAB.Eq(-42) || intersecAB.Eq(-42))
		return false;

	double dist1 = (intersecBC.Sub(intersecAB)).Length();
	double dist2 = (intersecCA.Sub(intersecBC)).Length();
	double dist3 = (intersecAB.Sub(intersecCA)).Length();

	if (dist1 <= MAX_P1_INTERSEC_DIST && dist2 <= MAX_P1_INTERSEC_DIST && dist3 <= MAX_P1_INTERSEC_DIST)
		return true;
	else
		return false;
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

double AnalytGrasp::CalcAngle(Coords vecA, Coords vecB)
{
	double num = vecA.Dot(vecB);
	double den = vecA.Length() * vecB.Length();
	return acos(num / den);
}

Coords AnalytGrasp::CalcIntersection(Coords pL, Coords rL, Coords pM, Coords rM)
{
	double num = rL.x * (pL.y - pM.y) + rL.y * (pM.x - pL.x);
	double den = rL.x * rM.y - rL.y * rM.x;
	double tM = num / den;

	if (tM < 0)
		return Coords(-42); //ERROR (for this purpose, tM < 0 is illegal)
	else
		return Coords(pM.x + rM.x * tM, pM.y + rM.y * tM);
}

AnalytGrasp::AnalytGrasp()
{
}

AnalytGrasp::~AnalytGrasp()
{
}

