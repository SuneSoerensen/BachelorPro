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
  {
    sdh->moveCmd(Q(7, -80*deg2rad, 80*deg2rad, 60*deg2rad, -80*deg2rad, 80*deg2rad, -80*deg2rad, 80*deg2rad), true);
    fullStop();
  }
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
  //Check if coordinates of B are correct:
  if(fingerBY > 0.0)
    throw("[SDHControl::grasp]: y-coordinate for finger B should be negative!");

  //Reset grasp checks:
  isValidGrasp = false;
  isExpectedGrasp = false;

  //Calculate sides for goal-triangle:
  goalSides[0] = sqrt(pow((fingerAX-fingerBX),2) + pow((fingerAY-fingerBY),2));
  goalSides[1] = sqrt(pow((fingerBX-fingerCX),2) + pow((fingerBY-fingerCY),2));
  goalSides[2] = sqrt(pow((fingerCX-fingerAX),2) + pow((fingerCY-fingerAY),2));

  //Calculate distances from hand-center to finger-contact-points and substract small length to ensure that points are inside the object:
  double distA = sqrt(fingerAX*fingerAX + fingerAY*fingerAY) - SDH_DIST_INTO_OBJECT;
  double distB = sqrt(fingerBX*fingerBX + fingerBY*fingerBY) - SDH_DIST_INTO_OBJECT;
  double distC = sqrt(fingerCX*fingerCX + fingerCY*fingerCY) - SDH_DIST_INTO_OBJECT;

  if(isPreGrasp) //If it is a pre-grasp, move each finger outwards by PREGRASP_SCALE mm:
  {
    distA += PREGRASP_SCALE;
    distB += PREGRASP_SCALE;
    distC += PREGRASP_SCALE;
  }

  if(SDHCONTROL_MODE)
  {
    cout << "\033[1;33m [SDHControl::grasp] DEBUG: \033[0m" << "distA: " << distA << " ; " << "distB: " << distB << " ; distC: " << distC << endl;
  }

  double angleAC = abs(atan2((fingerAX), (fingerAY)));
  double fingerAngleA = abs(atan2((fingerAX-SDH_FINGER_BASE_OFF_X), (fingerAY-SDH_FINGER_BASE_OFF_Y)));
  double fingerAngleC = abs(atan2((fingerCX+SDH_FINGER_BASE_OFF_X), (fingerCY-SDH_FINGER_BASE_OFF_Y)));
  double fingerAngleAC = (fingerAngleA + fingerAngleC) / 2.0;

  if(abs(fingerAngleA- fingerAngleC) > FINGERAC_ANGLE_THRESH)
    throw("[SDHControl::grasp]: Angle between A and C is not correct!");

  if(SDHCONTROL_MODE)
    cout << "\033[1;33m [SDHControl::grasp] DEBUG: \033[0m" << "Angle between finger A and C = fingerAngleAC (deg) = " << fingerAngleAC*rad2deg << endl;

  vector<double> fingertipPosA = calcFingertipPos(distA, angleAC);
  vector<double> fingertipPosC = calcFingertipPos(distC, angleAC);

  double xA = fingertipPosA[0];
  double xB = distB-FINGEROFFSET; //Distances are given relative to center of hand, so substract offset (dist. from center of hand)
  double xC = fingertipPosC[0];



  if(SDHCONTROL_MODE)
    cout << "\033[1;33m [SDHControl::grasp] DEBUG: \033[0m" << "finger A base-to-tip: " << xA << "; finger B base-to-tip: " << xB << " ; finger C base-to-tip: " << xC << endl;

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
    cout << "\033[1;33m [SDHControl::grasp] DEBUG: \033[0m" << "Found solution! height = " << y << endl;
    cout << "\033[1;33m [SDHControl::grasp] DEBUG: \033[0m" << "Finger A: angles = (" << anglesA[0]*rad2deg << ";" << anglesA[1]*rad2deg << ") pos = (" << xA << ";" << y << ")" << endl;
    cout << "\033[1;33m [SDHControl::grasp] DEBUG: \033[0m" << "Finger B: angles = (" << anglesB[0]*rad2deg << ";" << anglesB[1]*rad2deg << ") pos = (" << xB << ";" << y << ")" << endl;
    cout << "\033[1;33m [SDHControl::grasp] DEBUG: \033[0m" << "Finger C: angles = (" << anglesC[0]*rad2deg << ";" << anglesC[1]*rad2deg << ") pos = (" << xC << ";" << y << ")" << endl;
  }

  //Finally, move to found config.:
  if(SDHCONTROL_MODE)
    cout << "\033[1;33m [SDHControl::grasp] DEBUG: \033[0m" << "Going to configuration: " << Q(7, anglesB[0]*rad2deg, anglesB[1]*rad2deg, fingerAngleAC*rad2deg, anglesA[0]*rad2deg, anglesA[1]*rad2deg, anglesC[0]*rad2deg, anglesC[1]*rad2deg) << endl;

    //Then, go to target:
    /*FOR Q test*/

    if(POSTION_TEST)
      forTest = Q(7, anglesB[0], anglesB[1], fingerAngleAC, anglesA[0],anglesA[1], anglesC[0], anglesC[1]);
    else
     goToQ(Q(7, anglesB[0], anglesB[1], fingerAngleAC, anglesA[0],anglesA[1], anglesC[0], anglesC[1]));

    if(isPreGrasp)
      fullStop();

    if(!isPreGrasp)
      controlGrasp(xA, xB, xC);

    if(isPreGrasp && SDHCONTROL_MODE)
      cout << "\033[1;33m [SDHControl::grasp] DEBUG: \033[0m" << "Found height is = " << y + WRIST_HEIGHT + TARGET_Z + SEPERATOR_HEIGHT << endl;

    return y + WRIST_HEIGHT + TARGET_Z + SEPERATOR_HEIGHT;

}

double SDHControl::grasp(double fingerAX, double fingerAY, double fingerCX, double fingerCY, bool isPreGrasp)
{
   //reset grasp checks
   isValidGrasp = false;
   isExpectedGrasp = false;

   //calc finger dist
   double distA = abs(fingerAX) - SDH_FINGER_BASE_OFF_X;
   double distC = abs(fingerCX) + SDH_FINGER_BASE_OFF_X;

   if(isPreGrasp)
   {
     distA +=PREGRASP_SCALE;
     distC +=PREGRASP_SCALE;
   }

   //Result vectors:
   vector<double> anglesA;
   vector<double> anglesC;

   double xA = distA;
   double xC = distC;

   if(SDHCONTROL_MODE)
   {
    cout << "\033[1;33m [SDHControl::grasp] DEBUG: \033[0m" << "Finger A: dist = " << xA  << endl;
    cout << "\033[1;33m [SDHControl::grasp] DEBUG: \033[0m" << "Finger C: dist = " << xC << endl;
   }

   //To make sure that for-loop is run first time, insert invalid y's:
   anglesA = calcFingerAngle(xA, 400.0);
   anglesC = calcFingerAngle(xC, 400.0);

   //Find biggest distance from SDH base, where all checks in checkSolution() are satisfied:
   double y;
   for(y = 0.0; checkSolution(anglesA, anglesC); y += GRASPFINDSTEPSIZE)
   {
     anglesA = calcFingerAngle(xA, y);
     anglesC = calcFingerAngle(xC, y);

     if(y > GRASPDISTLIM)
     {
       throw("[SDHControl::grasp]: No solution was found (height exceeded 155.0mm)!");
     }
   }

   anglesA[0] = -(90*deg2rad-anglesA[0]);
   anglesC[0] = -(90*deg2rad-anglesC[0]);

   if(SDHCONTROL_MODE)
   {
     cout << "\033[1;33m [SDHControl::grasp] DEBUG: \033[0m" << "Found solution! height = " << y << endl;
     cout << "\033[1;33m [SDHControl::grasp] DEBUG: \033[0m" << "Finger A: angles = (" << anglesA[0]*rad2deg << ";" << anglesA[1]*rad2deg << ") pos = (" << xA << ";" << y << ")" << endl;
     cout << "\033[1;33m [SDHControl::grasp] DEBUG: \033[0m" << "Finger C: angles = (" << anglesC[0]*rad2deg << ";" << anglesC[1]*rad2deg << ") pos = (" << xC << ";" << y << ")" << endl;
   }

   if(SDHCONTROL_MODE)
     cout << "\033[1;33m [SDHControl::grasp] DEBUG: \033[0m" << "Going to configuration: " << Q(7, -80, -80, 90, anglesA[0]*rad2deg, anglesA[1]*rad2deg, anglesC[0]*rad2deg, anglesC[1]*rad2deg) << endl;

     //Then, go to target:
     goToQ(Q(7, -80*deg2rad, -80*deg2rad, 90*deg2rad, anglesA[0],anglesA[1], anglesC[0], anglesC[1]));
     if(isPreGrasp)
       fullStop();

     if(!isPreGrasp)
       controlGrasp(xA, xC);

     return y + WRIST_HEIGHT + TARGET_Z + SEPERATOR_HEIGHT;

}

bool SDHControl::getIsValidGrasp()
{
  return isValidGrasp;
}

bool SDHControl::getIsExpectedGrasp()
{
  return isExpectedGrasp;
}

Q SDHControl::getQ()
{
  return sdh->getQ();
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

bool SDHControl::checkSolution(vector<double> anglesA, vector<double> anglesC)
{
  bool res = false;

  if(abs(-(90*deg2rad-anglesA[0])+ anglesA[1]) > SDH_ANGLE_THRESH || abs(-(90*deg2rad-anglesC[0])+ anglesC[1]) > SDH_ANGLE_THRESH) //Check angle thresholds
    res = true;

  if(isnan(anglesA[0]) || isnan(anglesA[1]) || isnan(anglesC[0]) || isnan(anglesC[1])) //Check if any angles a nan
    res = true;

  if(abs(anglesA[0]) > SDH_MAX_ABS_ANGLE || abs(anglesA[0]) > SDH_MAX_ABS_ANGLE || abs(anglesC[0]) > SDH_MAX_ABS_ANGLE || abs(anglesC[1]) > SDH_MAX_ABS_ANGLE) // Are there invalid angles
    res = true;

  return res;
}

vector<double> SDHControl::calcFingerDist(double angleBase, double angleTop)
{
  vector<double> res(2);

  res[0]= -(sin(angleBase)*LENGTH1 + cos(180*deg2rad-(angleBase+angleTop+(90*deg2rad)))*LENGTH2);
  res[1]= cos(angleBase)*LENGTH1 + sin(180*deg2rad-(angleBase+angleTop+(90*deg2rad)))*LENGTH2;
  return res;
}

void SDHControl::controlGrasp(double goalDistA, double goalDistB, double goalDistC)
{
  Q currQ;

  if(POSTION_TEST)
    currQ =  forTest;
  else
    currQ =  sdh->getQ();

  vector<double> currDistA;
  vector<double> currDistB;
  vector<double> currDistC;

  if(SDHCONTROL_MODE)
    cout << "\033[1;33m [SDHControl::controlGrasp] DEBUG: \033[0m" << "Current configuration: " << currQ << endl;

  currDistA = calcFingerDist(currQ[3],currQ[4]);
  currDistB = calcFingerDist(currQ[0],currQ[1]);
  currDistC = calcFingerDist(currQ[5],currQ[6]);


  //Check if the grasp is a valid grasp:
  if(SDH_PRECISION < abs(goalDistA-currDistA[0]) || SDH_PRECISION < abs(goalDistB-currDistB[0]) || SDH_PRECISION < abs(goalDistC-currDistC[0]))
  {
    isValidGrasp = true;
  }

  if(SDHCONTROL_MODE)
  {
    //For each finger, check if it is too close to goal:
    if(SDH_PRECISION > abs(currDistA[0]-goalDistA))
      cout << "\033[1;33m [SDHControl::controlGrasp] DEBUG: \033[0m" << "finger A is very close to goal." << endl;

    if(SDH_PRECISION > abs(currDistB[0]-goalDistB))
      cout << "\033[1;33m [SDHControl::controlGrasp] DEBUG: \033[0m" << "finger B is veryclose to goal." << endl;

    if(SDH_PRECISION > abs(currDistC[0]-goalDistC))
      cout << "\033[1;33m [SDHControl::controlGrasp] DEBUG: \033[0m" << "finger C is very close to goal." << endl;
  }

  if(isValidGrasp || POSTION_TEST)
  {
    if(SDHCONTROL_MODE)
      cout << "\033[1;33m [SDHControl::controlGrasp] DEBUG: \033[0m" << "The current grasp is valid. Checking grasp-placement." << currQ << endl;

    //Check if the grasp is as expected:
    if(POSTION_TEST)
      controlGraspPlacement(currQ[2], currDistA[0], currDistB[0], currDistC[0]);
    else
      controlGraspPlacement(currQ[2], currDistA[0] - SDH_HALF_FINGER_WIDTH, currDistB[0] - SDH_HALF_FINGER_WIDTH, currDistC[0] - SDH_HALF_FINGER_WIDTH);
  }
}

void SDHControl::controlGrasp(double goalDistA, double goalDistC)
{
  Q currQ =  sdh->getQ();
  vector<double> currDistA;
  vector<double> currDistC;

  if(SDHCONTROL_MODE)
    cout << "\033[1;33m [SDHControl::controlGrasp]DEBUG: \033[0m" << "Current configuration: " << currQ << endl;

  currDistA = calcFingerDist(currQ[3],currQ[4]);
  currDistC = calcFingerDist(currQ[5],currQ[6]);

  //Check if the grasp is a valid grasp:
  if(SDH_PRECISION < abs(goalDistA-currDistA[0]) || SDH_PRECISION < abs(goalDistC-currDistC[0]))
    isValidGrasp = true;

  if(SDHCONTROL_MODE)
  {
    //For each finger, check if it is too close to goal:
    if(SDH_PRECISION > abs(currDistA[0]-goalDistA))
      cout << "\033[1;33m [SDHControl::controlGrasp]DEBUG: \033[0m" << "finger A is very close to goal." << endl;

    if(SDH_PRECISION > abs(currDistC[0]-goalDistC))
      cout << "\033[1;33m [SDHControl::controlGrasp]DEBUG: \033[0m" << "finger C is very close to goal." << endl;
  }

  if(isValidGrasp == true)
  {
    if(SDHCONTROL_MODE)
      cout << "\033[1;33m [SDHControl::controlGrasp]DEBUG: \033[0m" << "The current grasp is valid. Checking grasp-placement." << currQ << endl;

    //Check if the grasp is as expected:
    if(abs((goalDistA+goalDistC)-(currDistA[0]+currDistC[0]))< SDH_PRECISION)
      isExpectedGrasp = true;
  }
}


void SDHControl::controlGraspPlacement(double angleAC, double currDistA, double currDistB, double currDistC)
{
  isExpectedGrasp = true;

  //Calculate coordinates for finger-tips, when looking from finger-tips down to wrist:
  double xA = SDH_FINGER_BASE_OFF_X + cos(angleAC - (90*deg2rad))*currDistA;
  double yA = SDH_FINGER_BASE_OFF_Y - sin(angleAC - (90*deg2rad))*currDistA;

  double xB = 0.0;
  double yB = -(FINGEROFFSET + currDistB);

  double xC = -SDH_FINGER_BASE_OFF_X - cos(angleAC - (90*deg2rad))*currDistC;
  double yC = SDH_FINGER_BASE_OFF_Y - sin(angleAC - (90*deg2rad))*currDistC;

  if(POSTION_TEST)
  {
    cout << xA << ";" << yA << ";";
    cout << xB << ";" << yB << ";";
    cout << xC << ";" << yC << endl;
  }

  if(SDHCONTROL_MODE)
  {
    cout << "\033[1;33m [SDHControl::controlGraspPlacement] DEBUG: \033[0m" << "xA: " << xA << "; yA: " << yA << endl << "xB: " << xB << "; yB: " << yB << endl << "xC: " << xC << "; yC: " << yC << endl;
    cout << "\033[1;33m [SDHControl::controlGraspPlacement] DEBUG: \033[0m" << "angleAC (deg): " << angleAC*rad2deg << endl;
    cout << "\033[1;33m [SDHControl::controlGraspPlacement] DEBUG: \033[0m" << "currDistA: " << currDistA << " ; currDistB: " << currDistB << " ; currDistC: " << currDistC << endl;
  }

  //Calculate distances between finger-tips (which is the sides of the triangle):
  double currentSides[3];
  currentSides[0] = sqrt(pow((xA-xB),2) + pow((yA-yB),2)); //Side AB
  currentSides[1] = sqrt(pow((xB-xC),2) + pow((yB-yC),2)); //Side BC
  currentSides[2] = sqrt(pow((xC-xA),2) + pow((yC-yA),2)); //Side CA

  //Compare the expected sides (calculated from coordinates given to grasp()) to current sides, for each finger:
  if(!POSTION_TEST)
    for(int i = 0; i < 3; i++)
    {
      if(abs(currentSides[i]-goalSides[i]) > SDH_PRECISION && abs(goalSides[i]) < abs(currentSides[i])) //If the side is too big
      {
        cout << "\033[1;34m [SDHControl::controlGraspPlacement]: INFO: \033[0m" << "goal-current side no." << i << " is too big (grasp is not as expected)!" << endl;

        if(SDHCONTROL_MODE)
          cout << "\033[1;33m DEBUG: \033[0m" << "side" << i << ". Goal = " << goalSides[i] << ". Current = " << currentSides[i] <<  ". Diff = " << abs(currentSides[i]-goalSides[i]) << endl;

        isExpectedGrasp = false;
      }
      else if(abs(currentSides[i]-goalSides[i]) > SDH_PRECISION && abs(goalSides[i]) > abs(currentSides[i])) //If the side is too small
      {
        cout << "\033[1;34m [SDHControl::controlGraspPlacement]: INFO: \033[0m" << "goal-current side no." << i << " is too small (grasp is not as expected)!" << endl;

        if(SDHCONTROL_MODE)
          cout << "\033[1;33m [SDHControl::controlGraspPlacement] DEBUG: \033[0m" << "side" << i << ". Goal = " << goalSides[i] << ". Current = " << currentSides[i] <<  ". Diff = " << abs(currentSides[i]-goalSides[i]) << endl;

        isExpectedGrasp = false;
      }
    }

  //TODO: Check if current grasp could be another valid grasp
}
