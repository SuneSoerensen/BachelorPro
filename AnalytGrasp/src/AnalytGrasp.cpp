#include "AnalytGrasp.hpp"

Grasp AnalytGrasp::FindGrasp(Contour &aContour, double aScaleFactor)
{
	//find grasping regions
	vector<GraspReg> graspRegsList;
	FindGraspRegs(graspRegsList, aContour, aScaleFactor);

	//calc the objects center of mass
	Coords COM = TIAFC::FindCOM(aContour);

	//calc possible priority 1 grasps
	vector<Grasp> p1GraspsList;
	CalcP1Grasps(p1GraspsList, graspRegsList, COM);

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
		return p1GraspsList[currBestP1Grasp];
	}

	//calc possible priority 2 grasps


	//calc possible priority 3 grasps


	//if we get to here, we couldn't find any accetable grasps!
	throw("[AnalytGrasp::FindGrasP()]: Couldn't find any acceptable grasps!");
}

void AnalytGrasp::FindGraspRegs(vector<GraspReg> &aGraspRegsList, Contour &aContour, double aScaleFactor)
{
	int regWidth = GRASP_REG_WIDTH / aScaleFactor; //mm / (mm / pixels) = mm * (pixels / mm) = pixels

	for (int i = 0; i < aContour.list.size(); i++) //for all points in the contour
	{
		//make a new region
		GraspReg newReg;
		newReg.point = aContour.list[i];

		//find start and end points (of the current region)
		int startIndex = (aContour.list.size() + i - (regWidth/2)) % aContour.list.size(); //wrap
		int endIndex = (i + (regWidth/2)) % aContour.list.size(); //wrap

		//calc direction and normal vectors
		newReg.dirVec = (aContour.list[endIndex]).Sub(aContour.list[startIndex]); //r = p_end - p_start
		newReg.normVec = Coords(newReg.dirVec.y, -newReg.dirVec.x); //n = (y, -x), r = (x, y)

		vector<Coords> devs; //deviations
		devs.resize(regWidth);
		for (int j = 0; j <= regWidth; j++) //for all points in the current region
		{
			//calc its deviation
			Coords dirVec = (aContour.list[(startIndex + j) % aContour.list.size()]).Sub(aContour.list[startIndex]); //this points direction vector
			int projLength = (dirVec.Dot(newReg.normVec) / pow(newReg.normVec.Length(), 2)) * 1000; //the length of the direction vector projected onto the normal vector
			devs[j] = (newReg.normVec.Mul(projLength)).Div(1000); //projection of this points direction vector onto the normal vector of the region
		}

		//find largest (longest) deviation
		Coords largestDev(0, 0);
		for (int j = 0; j < devs.size(); j++)
			if (devs[j].Length() > largestDev.Length())
				largestDev = devs[j];

		//determine if this region is a valid grasp region
		if (largestDev.Length() * aScaleFactor <= 3.0)
		{
			//also check sum of lengths of deviations?
			
			//we have found a valid grasp region!
			aGraspRegsList.push_back(newReg);
		}
	}

	if (ANALYT_GRASP_MODE) //INFO
	{
		//make output image
		Mat graspRegsImage;
		aContour.image.copyTo(graspRegsImage);
		cvtColor(graspRegsImage, graspRegsImage, COLOR_GRAY2BGR);

		//draw the found grasp regs
		Vec3b color; //drawing color
		color.val[0] = 0; //blue
		color.val[1] = 0; //green
		color.val[2] = 255; //red

		for(int i = 0; i < aGraspRegsList.size(); i++ )
		{
			graspRegsImage.at<Vec3b>(aGraspRegsList[i].point.y, aGraspRegsList[i].point.x) = color;
		}

		//save to image
		imwrite("InfoFiles/AnalytGrasp(grasp_regs).jpg", graspRegsImage);
	}
}

void AnalytGrasp::CalcP1Grasps(vector<Grasp> &aP1GraspsList, vector<GraspReg> &aGraspRegsList, Coords aCOM)
{
	vector<vector<int> > passedAngCheckList;
	P1AngCheck(passedAngCheckList, aGraspRegsList);

	/*DEBUG*/cout << "found " << passedAngCheckList.size() << " sets of grasp regs that passed the angle check" << endl;

	/*DEBUG*/int passedIntersecCheck = 0;
	/*DEBUG*/int passedDistFromCOMCheck = 0;

	for (int i = 0; i < passedAngCheckList.size(); i++) //for all sets of grasp regs that passed the angle check
	{
		GraspReg a = aGraspRegsList[passedAngCheckList[i][0]];
		GraspReg b = aGraspRegsList[passedAngCheckList[i][1]];
		GraspReg c = aGraspRegsList[passedAngCheckList[i][2]];

		if (P1IntersecCheck(a.point, a.dirVec, b.point, b.dirVec, c.point, c.dirVec)) //if this grasp passes the intersection check
		{
			/*DEBUG*/passedIntersecCheck++;

			Coords graspFocus = ((a.point.Add(b.point)).Add(c.point)).Div(3);
			double graspDistFromCOM = (graspFocus.Sub(aCOM)).Length();

			if (graspDistFromCOM <= MAX_DIST_FROM_COM) //check if its focus is close enough to the objects COM
			{
				/*DEBUG*/passedDistFromCOMCheck++;

				Grasp newGrasp;

				newGrasp.type = p1;
				newGrasp.points = {a.point, b.point, c.point};
				newGrasp.focus = graspFocus;
				newGrasp.COM = aCOM;
				newGrasp.distFromCOM = graspDistFromCOM;

				aP1GraspsList.push_back(newGrasp);
			}
		}
	}

	/*DEBUG*/cout << "found " << passedIntersecCheck << " grasps that passed the intersection check" << endl;
	/*DEBUG*/cout << "found " << passedDistFromCOMCheck << " grasps that passed the distance from COM check" << endl;
}

void AnalytGrasp::P1AngCheck(vector<vector<int> > &aPassedAngCheckList, vector<GraspReg> &aGraspRegsList)
{
	double angAC, angBC, angBA;
	double minAng, maxAng;

	for (int a = 0; a < aGraspRegsList.size(); a++) //for all grasp regs
	{
		for (int b = 0; b < aGraspRegsList.size(); b++)
		{
			if (b != a) //find another one, which is not the first
			{
				for (int c = 0; c < aGraspRegsList.size(); c++)
				{
					if (c != b && c != a) //find a third, which is not one of the two others
					{
						angAC = CalcAngle(aGraspRegsList[a].normVec, aGraspRegsList[c].normVec); //calc the angle between the two fingers

						if (MIN_P1_ANG <= angAC && angAC <= MAX_P1_ANG) //if it is valid
						{
							//calc the angles defining the tolerable range
							minAng = ((6.28 - angAC)/2) - MAX_DEV_ANG;
							maxAng = ((6.28 - angAC)/2) + MAX_DEV_ANG;

							angBC = CalcAngle(aGraspRegsList[b].normVec, aGraspRegsList[c].normVec); //calc the angle between the thumb and second finger

							if (minAng <= angBC && angBC <= maxAng) //if the angle is within the range
							{
								angBA = CalcAngle(aGraspRegsList[b].normVec, aGraspRegsList[a].normVec); //calc the angle between the thumb and first finger

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

