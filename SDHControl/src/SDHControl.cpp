//
//  SDHControl.cpp
//  SDHControl
//
//  Created by <author> on 08/03/2018.
//
//

#include "SDHControl.hpp"
#include <iostream>

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

void SDHControl::goToQ(Q aQ)
{
  if(connected)
    if(!areThereInvalidAngles(aQ))
      sdh->moveCmd(aQ, true);
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

void SDHControl::grasp(double distA, double distB, double distC, double anAngle)
{
  //Distances are given relative to center of hand, so substract offset (dist. from center of hand):
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

  //Find biggest distance from SDH base, where all checks in checkSolution() are satisfied:
  double y;
  for(y = 0; checkSolution(anglesA, anglesB, anglesC); y += GRASPFINDSTEPSIZE)
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
    cout << "Going to configuration: " << Q(7, anglesB[0], anglesB[1], 45*deg2rad, anglesA[0],anglesA[1], anglesC[0], anglesC[1]) << endl;

    goToQ(Q(7, anglesB[0], anglesB[1], anAngle, anglesA[0],anglesA[1], anglesC[0], anglesC[1]));
}

void SDHControl::grasp(double distA, double distC)
{
  //Distances are given relative to center of hand, so substract offset (dist. from center of hand):
  double xA = distA-FINGEROFFSET;
  double xC = distC-FINGEROFFSET;

  //Then, check if one or more distances are too large:
  if(xA > GRASPDISTLIM || xC > GRASPDISTLIM)
    throw("[SDHControl::grasp]: Grasp points are too far away! Check define GRASPDISTLIM in settings to see the limit.");

    //Result vectors:
    vector<double> anglesA;
    vector<double> anglesC;

    //To make sure that for-loop is run first time, insert invalid y's:
    anglesA = calcFingerAngle(xA, 400.0);
    anglesC = calcFingerAngle(xC, 400.0);

    //Find biggest distance from SDH base, where all checks in checkSolution() are satisfied:
    double y;
    for(y = 0; checkSolution(anglesA, anglesC); y += GRASPFINDSTEPSIZE)
    {
      anglesA = calcFingerAngle(xA, y);
      anglesC = calcFingerAngle(xC, y);

      if(y > GRASPDISTLIM)
      {
        throw("[SDHControl::grasp]: No solution was found (height exceeded 155.0mm)!");
      }
    }

    if(SDHCONTROL_MODE)
    {
      cout << "Found solution! height = " << y << endl;
      cout << "Finger A: angles = (" << anglesA[0]*rad2deg << ";" << anglesA[1]*rad2deg << ") pos = (" << xA << ";" << y << ")" << endl;
      cout << "Finger C: angles = (" << anglesC[0]*rad2deg << ";" << anglesC[1]*rad2deg << ") pos = (" << xC << ";" << y << ")" << endl;
    }

    //Finally, move to found config.:
    if(SDHCONTROL_MODE)
      cout << "Going to configuration: " << Q(7, initQ[0], initQ[1], 90.0*deg2rad, anglesA[0],anglesA[1], anglesC[0], anglesC[1]) << endl;

      goToQ(Q(7, initQ[0], initQ[1], 90.0*deg2rad, anglesA[0],anglesA[1], anglesC[0], anglesC[1]));
}


/**************
*   Private
**************/
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

<<<<<<< HEAD
// for controlling Grasp

vector<double> SDHControl::calcFingerDist(double angleBase, double angleTop)
{
  vector<double> res(2);

  res[0]= -(sin(angleBase)*LENGTH1 + cos(180*deg2rad-(angleBase+angleTop+(90*deg2rad)))*LENGTH2)+FINGEROFFSET;
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

  currDistA = calcFingerDist(currQ[3],currQ[4]);
  currDistB = calcFingerDist(currQ[0],currQ[1]);
  currDistC = calcFingerDist(currQ[5],currQ[6]);

  if(0.1<abs(goalDistA-currDistA[0])&&0.1<abs(goalDistB-currDistB[0])&&0.1<abs(goalDistC-currDistC[0]))
  {
    // the grasp did not reach the goal, therfore it is gripping and object
    res = true;
  }


  if(0.1>abs(goalDistA-currDistA[0]) && res == false)
  {
    std::cout << "!grip failed finger A to close" << '\n';
  }

  if(0.1>abs(goalDistB-currDistB[0]) && res == false)
  {
    std::cout << "!grip failed finger B to close" << '\n';
  }

  if(0.1>abs(goalDistC-currDistC[0]) && res == false)
  {
    std::cout << "!grip failed finger C to close" << '\n';
  }


  if(res == true)
  {
    // check for controlGraspPlacment
    res = controlGraspPlacment(goalDistA,goalDistB,goalDistC,currDistA[0],currDistB[0],currDistC[0]);
  }

  return res;
}

bool SDHControl::controlGraspPlacment(double goalDistA, double goalDistB, double goalDistC, double CurrDistA, double CurrDistB, double CurrDistC)
{
  bool res = false;


  double diffDistA = goalDistA - CurrDistA;
  double diffDistB = goalDistB - CurrDistB;
  double diffDistC = goalDistC - CurrDistC;

  bool diffAB = true;
  bool diffAC = true;
  bool diffCB = true;

  //check if the placment has been passe therfor wrong;
  if (diffDistA<0);
  {
    res = false;
    std::cout << "!gripPlacmentWrong finger A has passed the goal postion" << '\n';
  }

  if (diffDistB<0);
  {
    res = false;
    std::cout << "!gripPlacmentWrong finger B has passed the goal postion" << '\n';
  }

  if (diffDistC<0);
  {
    res = false;
    std::cout << "!gripPlacmentWrong finger C has passed the goal postion" << '\n';
  }

  //check if fingers are in position
  if (0.1<abs(diffDistA-diffDistB)) //check if the diffrence of A B are close
  {
    std::cout << "!gripPlacmentWrong too large diff of finger A and B diff" << '\n';
    diffAB = false;
  }

  if(0.1<abs(diffDistA-diffDistC)) //check if the diffrence of A C are close
  {
    std::cout << "!gripPlacmentWrong too large diff of finger A and C diff" << '\n';
    diffAB = false;
  }

  if (0.1<abs(diffDistB-diffDistC)) //check if the diffrence of C B are close
  {
    std::cout << "!gripPlacmentWrong too large diff of finger C and B diff" << '\n';
    diffAB = false;
  }

  if(diffAB && diffAC && diffCB)
  {
    res = true;
  }
  else
  {
    // TODO code for cecking cuurent configuration
    std::cout << "!gripPlacmentWrong not the expect grip" << '\n';
  }

  return res;


=======
bool SDHControl::checkSolution(vector<double> anglesA, vector<double> anglesC)
{
  bool res = false;

  if(isNotWithinThresh(-(90*deg2rad-anglesA[0])+ anglesA[1], SDH_ANGLE_THRESH, -(90*deg2rad-anglesC[0])+ anglesC[1]))
    res = true;

  if(isThereAnan(anglesA[0], anglesA[1], 0, 0, anglesC[0], anglesC[1]))
    res = true;

  if(areThereInvalidAngles(anglesA, vector<double>{0,0}, anglesC))
    res = true;

  return res;
>>>>>>> 692e37eeb2f2bdfb8df42107446d346feb4f6c0c
}
