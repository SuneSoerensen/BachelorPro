//
//  SDHControl.cpp
//  SDHControl
//
//  Created by <author> on 08/03/2018.
//
//

#include "SDHControl.hpp"
#include <iostream>

SDHControl::SDHControl()
{
}

SDHControl::SDHControl(string canDev)
{
  sdh = new SDHDriver();
  sdh->connect( canDev, 1000000 );

  if(!sdh->isConnected())
    throw("[SDHControl::SDHControl]: Could not connect to hardware!");

  connected = true;

  goToInit();
}

void SDHControl::goToQ(Q aQ)
{
  if(connected)
    sdh->moveCmd(aQ, true);
  else
    throw("[SDHControl::goToQ]: SDH is not connected!.");
}

void SDHControl::goToInit()
{
  if(connected)
    sdh->moveCmd(initQ, true);
  else
    throw("[SDHControl::goToInit]: SDH is not connected!.");
}

void SDHControl::fullStop()
{
  if(connected)
    sdh->emergencyStop();
  else
    throw("[SDHControl::fullStop]: SDH is not connected!.");
}

bool SDHControl::isConnected()
{
  return connected;
}

void SDHControl::grasp(double distA, double distB, double distC)
{
  //Distances are given relative to center of SDH, so substract offset:
  double xA = distA-FINGEROFFSET;
  double xB = distB-FINGEROFFSET;
  double xC = distC-FINGEROFFSET;

  //Then, check if one or more distances are too large:
  if(xA > GRASPDISTLIM || xB > GRASPDISTLIM || xC > GRASPDISTLIM)
    throw("[SDHControl::grasp]: Grasp points are too far away! Check define GRASPDISTLIM in settings to see the limit.");

  //Result vectors:
  vector<double> anglesA;
  vector<double> anglesB;
  vector<double> anglesC;

  //To make sure that for-loop is run first time, insert invalid y's:
  anglesA = calcFingerAngle(xA, 400.0);
  anglesB = calcFingerAngle(xB, 400.0);
  anglesC = calcFingerAngle(xC, 400.0);

  //Find biggest distance from SDH base, where all fingers kan reach a the given dists.:
  double y;
  for(y = 0; checkSolution(anglesA, anglesB, anglesC); y += GRASPFINDSTEPSIZE)
  {
    anglesA = calcFingerAngle(xA, y);
    anglesB = calcFingerAngle(xB, y);
    anglesC = calcFingerAngle(xC, y);

    if(y > 155.0)
    {
      throw("[SDHControl::grasp]: No solution was found (height exceeded 155.0mm)!");
    }

  }

  //Correct angles for finger-bases:
  anglesA[0] = -(90*deg2rad-anglesA[0]);
  anglesB[0] = -(90*deg2rad-anglesB[0]);
  anglesC[0] = -(90*deg2rad-anglesC[0]);

  if(SDHCONTROL_MODE)
  {
    cout << "Found solution! y = " << y << endl;
    cout << "Finger A: angles = (" << anglesA[0]*rad2deg << ";" << anglesA[1]*rad2deg << ") pos = (" << xA << ";" << y << ")" << endl;
    cout << "Finger B: angles = (" << anglesB[0]*rad2deg << ";" << anglesB[1]*rad2deg << ") pos = (" << xB << ";" << y << ")" << endl;
    cout << "Finger C: angles = (" << anglesC[0]*rad2deg << ";" << anglesC[1]*rad2deg << ") pos = (" << xC << ";" << y << ")" << endl;
  }

  //Finally, move to found config.:
  if(SDHCONTROL_MODE)
    cout << "Going to configuration: " << Q(7, anglesB[0], anglesB[1], 45*deg2rad, anglesA[0],anglesA[1], anglesC[0], anglesC[1]) << endl;

  if(connected)
    goToQ(Q(7, anglesB[0], anglesB[1], 45*deg2rad, anglesA[0],anglesA[1], anglesC[0], anglesC[1]));
  else
    throw("[SDHControl::grasp]: SDH is not connected!.");
}

vector<double> SDHControl::calcFingerAngle(double x, double y)
{
  vector<double> res(2);

  res[1] = acos(((x*x)+(y*y)-(LENGTH1*LENGTH1)-(LENGTH2*LENGTH2))/(2*LENGTH1*LENGTH2)); //v1 (finger-tip)
  res[0] = atan2(y,x)-atan2(LENGTH2*sin(res[1]),LENGTH1+LENGTH2*cos(res[1])); //V0 (finger base)

  return res;
}

SDHControl::~SDHControl()
{
}

bool SDHControl::isThereAnan(double a,double b,double c,double d,double e,double f)
{
  return (isnan(a) || isnan(b) || isnan(c) || isnan(d) || isnan(e) || isnan(f));
}

bool SDHControl::areThereInvalidAngles(double a,double b,double c,double d,double e,double f)
{
  return (abs(a) > SDH_MAX_ABS_ANGLE || abs(b) > SDH_MAX_ABS_ANGLE || abs(c) > SDH_MAX_ABS_ANGLE || abs(d) > SDH_MAX_ABS_ANGLE || abs(e) > SDH_MAX_ABS_ANGLE || abs(f) > SDH_MAX_ABS_ANGLE);
}

bool SDHControl::isNotWithinThresh(double diffA, double diffB, double diffC)
{
  return (abs(diffA) > SDH_ANGLE_THRESH || abs(diffB) > SDH_ANGLE_THRESH || abs(diffC) > SDH_ANGLE_THRESH );
}

bool SDHControl::checkSolution(vector<double> anglesA, vector<double> anglesB, vector<double> anglesC)
{
  bool res = false;

  if(isNotWithinThresh(-(90*deg2rad-anglesA[0])+ anglesA[1], -(90*deg2rad-anglesB[0])+ anglesB[1], -(90*deg2rad-anglesC[0])+ anglesC[1]))
    res = true;

  if(isThereAnan(anglesA[0], anglesA[1], anglesB[0], anglesB[1], anglesC[0], anglesC[1]))
    res = true;

  if(areThereInvalidAngles(anglesA[0], anglesA[1], anglesB[0], anglesB[1], anglesC[0] , anglesC[1]))
    res = true;

  return res;
}
