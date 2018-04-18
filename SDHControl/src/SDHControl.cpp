//
//  SDHControl.cpp
//  SDHControl
//
//  Created by <author> on 08/03/2018.
//
//

#include "SDHControl.hpp"
#include <iostream>
#include <unistd.h>

/**************
*   Public
**************/

SDHControl::SDHControl()
{
}

SDHControl::SDHControl(string canDev)
{
  sdh = new SDHDriver();
  sdh->connect( canDev, 1000000 );

  if(!sdh->isConnected())
    throw("[SDHControl::SDHControl]: Could not connect to hardware (CAN)!");

  connected = true;
}

SDHControl::SDHControl(int port, unsigned long baudrate, double timeout)
{
  sdh = new SDHDriver();
  sdh->connect(port, baudrate, timeout, "/dev/ttyUSB%d");

  if(!sdh->isConnected())
    throw("[SDHControl::SDHControl]: Could not connect to hardware (RS232)!");

  connected = true;
}

void SDHControl::goToPreQ(Q aQ)
{
  Q firstQ = aQ;
  for(int i = 0; i < 7; i++)
  {
    if(i != 2)
      firstQ[i] -= SDH_ANGLE_DIFF_FIRST*deg2rad;
  }
  goToQ(firstQ);
}

void SDHControl::goToQ(Q aQ)
{
  if(connected)
    if(!areThereInvalidAngles(aQ))
    {
      sdh->moveCmd(aQ, true);
    }
    else
      throw("[SDHControl::goToQ]: One or more input angles are invalid!");
  else
    throw("[SDHControl::goToQ]: SDH is not connected!");
}

void SDHControl::goToInit()
{
  if(connected)
    sdh->moveCmd(initQ, true);
  else
    throw("[SDHControl::goToInit]: SDH is not connected!");
}

void SDHControl::fullStop()
{
  if(connected)
    sdh->emergencyStop();
  else
    throw("[SDHControl::fullStop]: SDH is not connected!");
}

bool SDHControl::isConnected()
{
  return connected;
}

double SDHControl::grasp(double fingerAX, double fingerAY, double fingerBX, double fingerBY, double fingerCX, double fingerCY, bool isPreGrasp)
{
  //Calculate sides for goal-triangle:
  goalSides[0] = sqrt(pow((fingerAX-fingerBX),2) + pow((fingerAY-fingerBY),2));
  goalSides[1] = sqrt(pow((fingerBX-fingerCX),2) + pow((fingerBY-fingerCY),2));
  goalSides[2] = sqrt(pow((fingerCX-fingerAX),2) + pow((fingerCY-fingerAY),2));

  //Calculate distances from hand-center to finger-contact-points and substract length to ensure that points are inside the object:
  double distA = sqrt(fingerAX*fingerAX + fingerAY*fingerAY) - SDH_DIST_INTO_OBJECT;
  double distB = sqrt(fingerBX*fingerBX + fingerBY*fingerBY) - SDH_DIST_INTO_OBJECT;
  double distC = sqrt(fingerCX*fingerCX + fingerCY*fingerCY) - SDH_DIST_INTO_OBJECT;

  if(isPreGrasp)
  {
    distA += PREGRASP_SCALE;
    distB += PREGRASP_SCALE;
    distC += PREGRASP_SCALE;
  }

  if(SDHCONTROL_MODE)
  {
    cout << "distA: " << distA << " ; " << "distB: " << distB << " ; distC: " << distC << endl;
  }

  double angleAC = abs(atan2(fingerAX, fingerAY));

  if(SDHCONTROL_MODE)
    cout << "angleAC (deg): " << angleAC*rad2deg << endl;

  vector<double> fingertipPosA = calcFingertipPos(distA, angleAC);
  vector<double> fingertipPosC = calcFingertipPos(distC, angleAC);

  double xA = fingertipPosA[0];
  double xB = distB-FINGEROFFSET; //Distances are given relative to center of hand, so substract offset (dist. from center of hand)
  double xC = fingertipPosC[0];

  if(SDHCONTROL_MODE)
    cout << "finger A base-to-tip: " << xA << "; finger B base-to-tip: " << xB << " ; finger C base-to-tip: " << xC << endl;

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

  //Find biggest distance from SDH base, where all checks in checkSolution() are satisfied:
  double y;
  for(y = 0.0; checkSolution(anglesA, anglesB, anglesC); y += GRASPFINDSTEPSIZE)
  {
    anglesA = calcFingerAngle(xA, y);
    anglesB = calcFingerAngle(xB, y);
    anglesC = calcFingerAngle(xC, y);

    if(y > GRASPDISTLIM)
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
    cout << "Found solution! height = " << y << endl;
    cout << "Finger A: angles = (" << anglesA[0]*rad2deg << ";" << anglesA[1]*rad2deg << ") pos = (" << xA << ";" << y << ")" << endl;
    cout << "Finger B: angles = (" << anglesB[0]*rad2deg << ";" << anglesB[1]*rad2deg << ") pos = (" << xB << ";" << y << ")" << endl;
    cout << "Finger C: angles = (" << anglesC[0]*rad2deg << ";" << anglesC[1]*rad2deg << ") pos = (" << xC << ";" << y << ")" << endl;
  }

  //Finally, move to found config.:
  if(SDHCONTROL_MODE)
    cout << "Going to configuration: " << Q(7, anglesB[0]*rad2deg, anglesB[1]*rad2deg, fingertipPosA[1]*rad2deg, anglesA[0]*rad2deg, anglesA[1]*rad2deg, anglesC[0]*rad2deg, anglesC[1]*rad2deg) << endl;

    //Then, go to target:
    goToQ(Q(7, anglesB[0], anglesB[1], fingertipPosA[1], anglesA[0],anglesA[1], anglesC[0], anglesC[1]));
    if(isPreGrasp)
      fullStop();

    if(!isPreGrasp)
      controlGrasp(xA, xB, xC);

    return y + WRIST_HEIGHT + TARGET_Z + SEPERATOR_HEIGHT; //Z-coordinate to give directly to URControl (moveAbs())

}

void SDHControl::adjustVel(double joint0, double joint1, double joint2, double joint3, double joint4, double joint5, double joint6)
{

  Q velLim = sdh->getVelLimits();

  for(int i = 0; i < velLim.size(); i++)
    velLim[i] *= 1.19;

  //Q velLim(7, temp);

  velLim[0] *= joint0;
  velLim[1] *= joint1;
  velLim[2] *= joint2;
  velLim[3] *= joint3;
  velLim[4] *= joint4;
  velLim[5] *= joint5;
  velLim[6] *= joint6;

  sdh->setTargetQVel(velLim);
}

SDHControl::~SDHControl()
{
}

/**************
*   Private
**************/
vector<double> SDHControl::calcFingertipPos(double visDist, double anAngle)
{
  vector<double> res(2);
  double theta = (anAngle) - (60*deg2rad);
  double graspDist = sqrt((visDist*visDist)+(FINGEROFFSET*FINGEROFFSET)-(2*visDist*FINGEROFFSET*cos(theta)));

  if(SDHCONTROL_MODE)
    cout << "[calcFingertipPos]: theta (deg): " << theta*rad2deg << endl;

  if(theta <= (-28.0*deg2rad) /*&& theta > -SDH_PRECISION*/) //If fingers are parrallel (or very close to)
    theta = -60.0*deg2rad;

  res[0] = graspDist;
  res[1] = theta+(60*deg2rad);

  return res;
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

bool SDHControl::areThereInvalidAngles(Q aQ)
{
  //Note that joint #2 (between finger A and C) can only be between 90 and 0 degrees!
  return (abs(aQ[0]) > SDH_MAX_ABS_ANGLE || abs(aQ[1]) > SDH_MAX_ABS_ANGLE || aQ[2] > SDH_MAX_ABS_ANGLE || aQ[2] < 0 || abs(aQ[3]) > SDH_MAX_ABS_ANGLE || abs(aQ[4]) > SDH_MAX_ABS_ANGLE || abs(aQ[5]) > SDH_MAX_ABS_ANGLE || abs(aQ[5]) > SDH_MAX_ABS_ANGLE);
}

bool SDHControl::areThereInvalidAngles(vector<double> anglesA, vector<double> anglesB, vector<double> anglesC)
{
  return (abs(anglesA[0]) > SDH_MAX_ABS_ANGLE || abs(anglesA[1]) > SDH_MAX_ABS_ANGLE || abs(anglesB[0]) > SDH_MAX_ABS_ANGLE || abs(anglesB[1]) > SDH_MAX_ABS_ANGLE || abs(anglesC[0]) > SDH_MAX_ABS_ANGLE || abs(anglesC[1]) > SDH_MAX_ABS_ANGLE);
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

  if(areThereInvalidAngles(anglesA, anglesB, anglesC))
    res = true;

  return res;
}

// for controlling Grasp

vector<double> SDHControl::calcFingerDist(double angleBase, double angleTop)
{
  vector<double> res(2);

  res[0]= -(sin(angleBase)*LENGTH1 + cos(180*deg2rad-(angleBase+angleTop+(90*deg2rad)))*LENGTH2);
  res[1]= cos(angleBase)*LENGTH1 + sin(180*deg2rad-(angleBase+angleTop+(90*deg2rad)))*LENGTH2;
  return res;
}

bool SDHControl::controlGrasp(double goalDistA, double goalDistB, double goalDistC)
{
  bool res = false;
  Q currQ = Q(7);
  currQ = sdh->getQ(); // get the confiquration of the hand
  vector<double> currDistA;
  vector<double> currDistB;
  vector<double> currDistC;

  if(SDHCONTROL_MODE)
    cout << "Current configuration: " << currQ << endl;

  currDistA = calcFingerDist(currQ[3],currQ[4]);
  currDistB = calcFingerDist(currQ[0],currQ[1]);
  currDistC = calcFingerDist(currQ[5],currQ[6]);

  //Check if the grasp is successful
  if(SDH_PRECISION < abs(goalDistA-currDistA[0]) || SDH_PRECISION < abs(goalDistB-currDistB[0]) || SDH_PRECISION < abs(goalDistC-currDistC[0]))
  {
    res = true;
  }


  if(SDHCONTROL_MODE)
  {
    if(SDH_PRECISION > abs(currDistA[0]-goalDistA))
    {
      std::cout << "finger A close to goal" << endl;
    }

    if(SDH_PRECISION > abs(currDistB[0]-goalDistB))
    {
      std::cout << "finger B close to goal" << endl;
    }

    if(SDH_PRECISION > abs(currDistC[0]-goalDistC))
    {
      std::cout << "finger C close to goal" << endl;
    }
  }


  if(res == true)
  {
    //check for controlGraspPlacment:
    res = controlGraspPlacment(currQ[2], currDistA[0] - SDH_HALF_FINGER_WIDTH, currDistB[0] - SDH_HALF_FINGER_WIDTH, currDistC[0] - SDH_HALF_FINGER_WIDTH);
  }

  return res;
}

bool SDHControl::controlGraspPlacment(double angleAC, double currDistA, double currDistB, double currDistC)
{
  bool res = true;

  double xA = SDH_FINGER_BASE_OFF_X + cos(angleAC - (90*deg2rad))*currDistA;
  double yA = SDH_FINGER_BASE_OFF_Y - sin(angleAC - (90*deg2rad))*currDistA;

  double xB = 0.0;
  double yB = -(FINGEROFFSET + currDistB);

  double xC = -SDH_FINGER_BASE_OFF_X - cos(angleAC - (90*deg2rad))*currDistC;
  double yC = SDH_FINGER_BASE_OFF_Y - sin(angleAC - (90*deg2rad))*currDistC;

  if(SDHCONTROL_MODE)
  {
    cout << "xA: " << xA << "; yA: " << yA << endl << "xB: " << xB << "; yB: " << yB << endl << "xC: " << xC << "; yC: " << yC << endl;
    cout << "angleAC (deg): " << angleAC*rad2deg << endl;
    cout << "currDistA: " << currDistA << " ; currDistB: " << currDistB << " ; currDistC: " << currDistC << endl;
  }

  double currentSides[3];

  currentSides[0] = sqrt(pow((xA-xB),2) + pow((yA-yB),2));
  currentSides[1] = sqrt(pow((xB-xC),2) + pow((yB-yC),2));
  currentSides[2] = sqrt(pow((xC-xA),2) + pow((yC-yA),2));

  for(int i = 0; i < 3; i++)
  {
    if(abs(currentSides[i]-goalSides[i]) > SDH_PRECISION && abs(goalSides[i]) < abs(currentSides[i]))
    {
      cout << "goal-current side no." << i << " is too big (grasp is not as expected)!" << endl;

      if(SDHCONTROL_MODE)
        cout << "side" << i << ". Goal = " << goalSides[i] << ". Current = " << currentSides[i] <<  ". Diff = " << abs(currentSides[i]-goalSides[i]) << endl;

      res = false;
    }
    else if(abs(currentSides[i]-goalSides[i]) > SDH_PRECISION && abs(goalSides[i]) > abs(currentSides[i]))
    {
      cout << "goal-current side no." << i << " is too small (grasp is not as expected)!" << endl;

      if(SDHCONTROL_MODE)
        cout << "side" << i << ". Goal = " << goalSides[i] << ". Current = " << currentSides[i] <<  ". Diff = " << abs(currentSides[i]-goalSides[i]) << endl;

      res = false;
    }
  }

  //TODO: Check if current grip could be valid

  return res;
}
