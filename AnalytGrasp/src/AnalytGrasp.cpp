#include "AnalytGrasp.hpp"

Grasp AnalytGrasp::FindGrasp(Contour &aContour, double aScaleFactor)
{
	//find grasping regions
	vector<GraspReg> graspRegsList;
	FindGraspRegs(graspRegsList, aContour, aScaleFactor);

	//calc the objects center of mass
	Coords COM = TIAFC::FindCOM(aContour);

	//calc possible priority 1 grasps
	/*INFO*/cout << "calculating p1 grasps..." << endl;
	vector<Grasp> p1GraspsList;
	CalcP1Grasps(p1GraspsList, graspRegsList, COM, aScaleFactor);

	if (p1GraspsList.size() > 0)
	{
		//find the best p1 grasp
		double currShortestDist = INFINITY;
		int currBestP1Grasp;
		for (int i = 0; i < p1GraspsList.size(); i++)
		{
			double distFromCOM = (p1GraspsList[i].focus.Sub(p1GraspsList[i].COM)).Length();
			if (distFromCOM < currShortestDist)
			{
				currShortestDist = distFromCOM;
				currBestP1Grasp = i;
			}
		}

		return p1GraspsList[currBestP1Grasp];
	}

	//calc possible priority 2 grasps


	//calc possible priority 3 grasps
	/*INFO*/cout << "calculating p3 grasps..." << endl;
	vector<Grasp> p3GraspsList;
	CalcP3Grasps(p3GraspsList, graspRegsList, COM, aScaleFactor);

	if (p3GraspsList.size() > 0)
	{
		//find the best p1 grasp
		double currShortestDist = INFINITY;
		int currBestP3Grasp;
		for (int i = 0; i < p3GraspsList.size(); i++)
		{
			double distFromCOM = (p3GraspsList[i].focus.Sub(p3GraspsList[i].COM)).Length();
			if (distFromCOM < currShortestDist)
			{
				currShortestDist = distFromCOM;
				currBestP3Grasp = i;
			}
		}

		return p3GraspsList[currBestP3Grasp];
	}

	//if we get to here, we couldn't find any accetable grasps!
	throw("[AnalytGrasp::FindGrasP()]: Couldn't find any acceptable grasps!");
}

void AnalytGrasp::FindGraspRegs(vector<GraspReg> &aGraspRegsList, Contour &aContour, double aScaleFactor)
{
	int halfRegWidth = (GRASP_REG_WIDTH / aScaleFactor) / 2; //mm / (mm / pixels) = mm * (pixels / mm) = pixels

	for (int i = 0; i < aContour.list.size(); i++) //for all points in the contour
	{
		//make a new region
		GraspReg newReg;
		newReg.point = aContour.list[i];

		//find start and end points (of the current region)
		int startIndex = (aContour.list.size() + i - halfRegWidth) % aContour.list.size(); //wrap
		int endIndex = (i + halfRegWidth) % aContour.list.size(); //wrap

		//calc direction and normal vectors
		newReg.dirVec = (aContour.list[endIndex]).Sub(aContour.list[startIndex]); //r = p_end - p_start
		newReg.normVec = Coords(newReg.dirVec.y, -newReg.dirVec.x); //n = (y, -x), r = (x, y)

		//make sure the normal vector is pointing into the object
		if (CalcAngle(newReg.normVec, aContour.inDirs[i]) > CalcAngle(newReg.normVec.Mul(-1), aContour.inDirs[i]))
			newReg.normVec = newReg.normVec.Mul(-1);

		vector<Coords> devs; //deviations
		devs.resize(halfRegWidth * 2 + 1);
		int index = 0;
		for (int j = 0; j < (halfRegWidth * 2 + 1); j++) //for all points in the current region
		{
			index = (startIndex + j) % aContour.list.size();
			//calc its deviation
			Coords dirVec = (aContour.list[index]).Sub(aContour.list[startIndex]); //this points direction vector
			int projLength = (dirVec.Dot(newReg.normVec) / pow(newReg.normVec.Length(), 2)) * 1000; //the length of the direction vector projected onto the normal vector
			devs[j] = (newReg.normVec.Mul(projLength)).Div(1000); //projection of this points direction vector onto the normal vector of the region
		}

		//find largest (longest) deviation
		Coords largestDev(0, 0);
		for (int j = 0; j < devs.size(); j++)
			if (devs[j].Length() > largestDev.Length())
				largestDev = devs[j];

		//determine if this region is a valid grasp region
		if (largestDev.Length() * aScaleFactor <= 1.0)
		{
			//also check sum of lengths of deviations?
			
			//we have found a valid grasp region!
			aGraspRegsList.push_back(newReg);
		}
	}

	if (ANALYT_GRASP_MODE) //INFO
	{
		cout << "found " << aGraspRegsList.size() << " valid grasp regions" << endl;

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

void AnalytGrasp::CalcP1Grasps(vector<Grasp> &aP1GraspsList, vector<GraspReg> &aGraspRegsList, Coords aCOM, double aScaleFactor)
{
	/*INFO*/int passedAngCheck = 0;
	/*INFO*/int passedPosCheck = 0;

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
						if (P1AngCheck(aGraspRegsList[a], aGraspRegsList[b], aGraspRegsList[c])) //if they pass the angle check
						{
							/*INFO*/passedAngCheck++;

							Coords graspFocus = ((aGraspRegsList[a].point.Add(aGraspRegsList[b].point)).Add(aGraspRegsList[c].point)).Div(3);

							if (P1PosCheck(aGraspRegsList[a], aGraspRegsList[b], aGraspRegsList[c], graspFocus, aCOM, aScaleFactor)) //if they pass the position check
							{
								/*INFO*/passedPosCheck++;

								Grasp newGrasp;
								newGrasp.type = p1;
								newGrasp.points = {aGraspRegsList[a], aGraspRegsList[b], aGraspRegsList[c]};
								newGrasp.focus = graspFocus;
								newGrasp.COM = aCOM;

								aP1GraspsList.push_back(newGrasp);
							}
						}
					}
				}
			}
		}
	}

	if (ANALYT_GRASP_MODE) //INFO
	{
		cout << "found " << passedAngCheck << " p1 grasps that passed the angle check" << endl;
		cout << "found " << passedPosCheck << " p1 grasps that passed the position check" << endl;
	}
}

bool AnalytGrasp::P1AngCheck(GraspReg a, GraspReg b, GraspReg c)
{
	//calc the angle between the two fingers
	double angAC = CalcAngle(a.normVec, c.normVec);

	//calc the angles defining the tolerable range
	double minAng = ((6.28 - angAC)/2) - MAX_DEV_ANG;
	double maxAng = ((6.28 - angAC)/2) + MAX_DEV_ANG;

	double angBC = CalcAngle(b.normVec, c.normVec); //calc the angle between the thumb and second finger
	double angBA = CalcAngle(b.normVec, a.normVec); //calc the angle between the thumb and first finger

	if (MIN_P1_ANG <= angAC && angAC <= MAX_P1_ANG) //if the angle between the two fingers is valid
	{
		if (minAng <= angBC && angBC <= maxAng) //if the angle between the thumb and second finger is within the range
		{
			if (minAng <= angBA && angBA <= maxAng) //if the angle between the thumb and first finger is within the range
			{
				return true; //we have found a possible grasp!
			}
		}
	}

	return false; //if we get to here, the regions didn't pass the angle check!
}

bool AnalytGrasp::P1PosCheck(GraspReg a, GraspReg b, GraspReg c, Coords aFocus, Coords aCOM, double aScaleFactor)
{
	//check if the focus is too far from the COM
	if ((aCOM.Sub(aFocus)).Length() * aScaleFactor > MAX_DIST_FROM_COM)
		return false;

	//check if the points are too close to each other
	if ((a.point.Sub(b.point)).Length() * aScaleFactor < MIN_GRASP_POINT_DIST)
		return false;

	if ((b.point.Sub(c.point)).Length() * aScaleFactor < MIN_GRASP_POINT_DIST)
		return false;

	if ((c.point.Sub(a.point)).Length() * aScaleFactor < MIN_GRASP_POINT_DIST)
		return false;

	//check that the grasp regions are oriented correctly
	Coords aToFocus = aFocus.Sub(a.point);
	Coords bToFocus = aFocus.Sub(b.point);
	Coords cToFocus = aFocus.Sub(c.point);

	if (CalcAngle(aToFocus, a.normVec) > MAX_DEV_ANG)
		return false;

	if (CalcAngle(bToFocus, b.normVec) > MAX_DEV_ANG)
		return false;

	if (CalcAngle(cToFocus, c.normVec) > MAX_DEV_ANG)
		return false;

	return true; //if we get to here, we have found a possible grasp!
}

void AnalytGrasp::CalcP3Grasps(vector<Grasp> &aP3GraspsList, vector<GraspReg> &aGraspRegsList, Coords aCOM, double aScaleFactor)
{
	/*INFO*/int passedAngCheck = 0;
	/*INFO*/int passedPosCheck = 0;

	for (int a = 0; a < aGraspRegsList.size(); a++) //for all grasp regs
	{
		for (int c = 0; c < aGraspRegsList.size(); c++)
		{
			if (c != a) //find another one, which is not the first
			{
				if (P3AngCheck(aGraspRegsList[a], aGraspRegsList[c])) //if they pass the angle check
				{
					/*INFO*/passedAngCheck++;

					Coords graspFocus = (aGraspRegsList[a].point.Add(aGraspRegsList[c].point)).Div(2);

					if (P3PosCheck(aGraspRegsList[a], aGraspRegsList[c], graspFocus, aCOM, aScaleFactor)) //if they pass the position check
					{
						/*INFO*/passedPosCheck++;

						Grasp newGrasp;
						newGrasp.type = p3;
						newGrasp.points = {aGraspRegsList[a], aGraspRegsList[c]};
						newGrasp.focus = graspFocus;
						newGrasp.COM = aCOM;

						aP3GraspsList.push_back(newGrasp);
					}
				}
			}
		}
	}

	if (ANALYT_GRASP_MODE) //INFO
	{
		cout << "found " << passedAngCheck << " p3 grasps that passed the angle check" << endl;
		cout << "found " << passedPosCheck << " p3 grasps that passed the position check" << endl;
	}
}

bool AnalytGrasp::P3AngCheck(GraspReg a, GraspReg c)
{
	//calc the angle between the two fingers
	double angAC = CalcAngle(a.normVec, c.normVec);

	if (3.14 - MAX_DEV_ANG <= angAC && angAC <= 3.14 + MAX_DEV_ANG) //if the angle between the two fingers is valid
	{
		return true; //we have found a possible grasp!
	}

	return false; //if we get to here, the regions didn't pass the angle check!
}

bool AnalytGrasp::P3PosCheck(GraspReg a, GraspReg c, Coords aFocus, Coords aCOM, double aScaleFactor)
{
	//check if the focus is too far from the COM
	if ((aCOM.Sub(aFocus)).Length() * aScaleFactor > MAX_DIST_FROM_COM)
		return false;

	//check if the points are too close to each other
	if ((a.point.Sub(c.point)).Length() * aScaleFactor < MIN_GRASP_POINT_DIST)
		return false;

	//check that the grasp regions are oriented correctly
	Coords aToFocus = aFocus.Sub(a.point);
	Coords cToFocus = aFocus.Sub(c.point);

	if (CalcAngle(aToFocus, a.normVec) > MAX_DEV_ANG)
		return false;

	if (CalcAngle(cToFocus, c.normVec) > MAX_DEV_ANG)
		return false;

	return true; //if we get to here, we have found a possible grasp!
}

double AnalytGrasp::CalcAngle(Coords vecA, Coords vecB)
{
	double num = vecA.Dot(vecB);
	double den = vecA.Length() * vecB.Length();
	return acos(num / den);
}

AnalytGrasp::AnalytGrasp()
{
}

AnalytGrasp::~AnalytGrasp()
{
}

