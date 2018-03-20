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
  //sdh = new SDHDriver();
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
  sdh->emergencyStop();
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
  for(y = GRASPSTARTHEIGHT; isThereAnan(anglesA[0], anglesA[1], anglesB[0], anglesB[1], anglesC[0], anglesC[1]); y-=GRASPFINDSTEPSIZE)
  {
    /*if(DEBUG_MODE)
      cout << "Currently trying y = " << y << endl;*/

    anglesA = calcFingerAngle(xA, y);
    anglesB = calcFingerAngle(xB, y);
    anglesC = calcFingerAngle(xC, y);

  }

  //Correct angles for finger-bases:
  anglesA[0] = -(90*deg2rad-anglesA[0]);
  anglesB[0] = -(90*deg2rad-anglesB[0]);
  anglesC[0] = -(90*deg2rad-anglesC[0]);

  if(DEBUG_MODE)
  {
    cout << "Found solution! y = " << y << endl;
    cout << "Finger A: angles = (" << anglesA[0]*rad2deg << ";" << anglesA[1]*rad2deg << ") pos = (" << xA << ";" << y << ")" << endl;
    cout << "Finger B: angles = (" << anglesB[0]*rad2deg << ";" << anglesB[1]*rad2deg << ") pos = (" << xB << ";" << y << ")" << endl;
    cout << "Finger C: angles = (" << anglesC[0]*rad2deg << ";" << anglesC[1]*rad2deg << ") pos = (" << xC << ";" << y << ")" << endl;
  }

  //Finally, move to found config.:
  if(DEBUG_MODE)
    cout << "Going to configuration: " << Q(7, anglesB[0], anglesB[1], 45*deg2rad, anglesA[0],anglesA[1], anglesC[0], anglesC[1]) << endl;

  goToQ(Q(7, anglesB[0], anglesB[1], 45*deg2rad, anglesA[0],anglesA[1], anglesC[0], anglesC[1]));
}

void SDHControl::grasp(double distA, double distB, double distC, double howMuchLower)
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
  for(y = GRASPSTARTHEIGHT; isThereAnan(anglesA[0], anglesA[1], anglesB[0], anglesB[1], anglesC[0], anglesC[1]); y-=GRASPFINDSTEPSIZE)
  {
    /*if(DEBUG_MODE)
      cout << "Currently trying y = " << y << endl;*/

    anglesA = calcFingerAngle(xA, y);
    anglesB = calcFingerAngle(xB, y);
    anglesC = calcFingerAngle(xC, y);

  }

  //Check if y-howMuchLower results in valid angles:
  y -= howMuchLower;
  anglesA = calcFingerAngle(xA, y);
  anglesB = calcFingerAngle(xB, y);
  anglesC = calcFingerAngle(xC, y);

  if(isThereAnan(anglesA[0], anglesA[1], anglesB[0], anglesB[1], anglesC[0], anglesC[1]))
    throw("[SDHControl::grasp (debug)]: Size of input 'howMuchLower' resulted in invalid finger configuration!");

  //Correct angles for finger-bases:
  anglesA[0] = -(90*deg2rad-anglesA[0]);
  anglesB[0] = -(90*deg2rad-anglesB[0]);
  anglesC[0] = -(90*deg2rad-anglesC[0]);

  if(DEBUG_MODE)
  {
    cout << "Found solution! y = " << y << endl;
    cout << "Finger A: angles = (" << anglesA[0]*rad2deg << ";" << anglesA[1]*rad2deg << ") pos = (" << xA << ";" << y << ")" << endl;
    cout << "Finger B: angles = (" << anglesB[0]*rad2deg << ";" << anglesB[1]*rad2deg << ") pos = (" << xB << ";" << y << ")" << endl;
    cout << "Finger C: angles = (" << anglesC[0]*rad2deg << ";" << anglesC[1]*rad2deg << ") pos = (" << xC << ";" << y << ")" << endl;
  }

  //Finally, move to found config.:
  if(DEBUG_MODE)
    cout << "Going to configuration: " << Q(7, anglesB[0], anglesB[1], 45*deg2rad, anglesA[0],anglesA[1], anglesC[0], anglesC[1]) << endl;

  goToQ(Q(7, anglesB[0], anglesB[1], 45*deg2rad, anglesA[0],anglesA[1], anglesC[0], anglesC[1]));
}

vector<double> SDHControl::calcFingerAngle(double x, double y)
{
  vector<double> res(2);

  res[1] = acos(((x*x)+(y*y)-(LENGTH1*LENGTH1)-(LENGTH2*LENGTH2))/(2*LENGTH1*LENGTH2)); //v1 (finger-tip)
  res[0] = atan2(y,x)-atan2(LENGTH2*sin(res[1]),LENGTH1+LENGTH2*cos(res[1])); //V0 (finger base)

  return res;
}

bool SDHControl::isThereAnan(double a,double b,double c,double d,double e,double f)
{
  return (isnan(a) || isnan(b) || isnan(c) || isnan(d) || isnan(e) || isnan(f));
}

SDHControl::~SDHControl()
{
}
