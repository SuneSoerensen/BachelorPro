#include <iostream>
#include "SDHControl.hpp"
#include <string>
#include <unistd.h>


using namespace std;

bool checkSolution(vector<double> anglesA, vector<double> anglesB, vector<double> anglesC);

int main()
{
  double distA;
  double distB;
  double distC;

  double xA;
  double xB;
  double xC;

  //Result vectors:
  vector<double> anglesA;
  vector<double> anglesB;
  vector<double> anglesC;

  double y;

  SDHControl sdh;


  while(true)
  {
    cout << "Enter distA: ";
    cin >> distA;
    cout << "Enter distB: ";
    cin >> distB;
    cout << "Enter distC: ";
    cin >> distC;
    //Distances are given relative to center of SDH, so substract offset:
    xA = distA-FINGEROFFSET;
    xB = distB-FINGEROFFSET;
    xC = distC-FINGEROFFSET;

    //Then, check if one or more distances are too large:
    if(xA > GRASPDISTLIM || xB > GRASPDISTLIM || xC > GRASPDISTLIM)
      throw("[SDHControl::grasp]: Grasp points are too far away! Check define GRASPDISTLIM in settings to see the limit.");


    //To make sure that for-loop is run first time, insert invalid y's:
    anglesA = sdh.calcFingerAngle(xA, 400.0);
    anglesB = sdh.calcFingerAngle(xB, 400.0);
    anglesC = sdh.calcFingerAngle(xC, 400.0);

    //Find biggest distance from SDH base, where all fingers kan reach a the given dists.:
    for(y = 0;  checkSolution(anglesA, anglesB, anglesC); y += GRASPFINDSTEPSIZE)
    {
      anglesA = sdh.calcFingerAngle(xA, y);
      anglesB = sdh.calcFingerAngle(xB, y);
      anglesC = sdh.calcFingerAngle(xC, y);

      if(y > 155.0)
      {
        throw("[SDHControl::grasp]: No solution was found!");
      }

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
  }


  return 0;
}

bool checkSolution(vector<double> anglesA, vector<double> anglesB, vector<double> anglesC)
{
  SDHControl sdh;
  bool res = false;

  if(sdh.isNotWithinThresh(-(90*deg2rad-anglesA[0])+ anglesA[1], -(90*deg2rad-anglesB[0])+ anglesB[1], -(90*deg2rad-anglesC[0])+ anglesC[1]))
    res = true;

  if(sdh.isThereAnan(anglesA[0], anglesA[1], anglesB[0], anglesB[1], anglesC[0], anglesC[1]))
    res = true;

  if(sdh.areThereInvalidAngles(anglesA[0], anglesA[1], anglesB[0], anglesB[1], anglesC[0] , anglesC[1]))
    res = true;

  return res;
}
