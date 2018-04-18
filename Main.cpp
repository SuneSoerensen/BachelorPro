#include <iostream>

#include "SDHControl.hpp"
#include "URControl.hpp"
#include "Vision.hpp"

using namespace std;

int main()
{
  //Instatiate objects:
  SDHControl hand(0, 115200, 0.5);
  URControl ur("192.168.100.4",30002);
  Vision vis;

  //Declare variables:
  Coords objCoords;
  vector<Coords> graspPoints; //In real-worl coordinates
  vector<Coords> handLocalGraspPoints; //In SDH-coordinates (where grasp-focus is (0,0))
  double wristRotation;

  cout << "Welcome to grasp-utility! Place calibration-marker and press ENTER to continue." << endl;
  cin.get();
  cout << "Calibrating..." << endl;
  vis.Calib();
  cout << "Done calibrating. Remove calibration-marker and place target-object. Then, press ENTER to compute grasp." << endl;
  cin.get();
  cout << "Computing grasp.." << endl;
  //TODO: graspPoints = insert function(s) when ready..
  //...
  objCoords = vis.GetObjCoords();
  cout << "Found real-world coordinates for target-object: (" << objCoords.x << ";" <<  objCoords.y << ")." << "Press ENTER to initialize hand and arm.." << endl;
  cin.get();
  cout << "Initializing hand and arm..." << endl;
  ur.moveToInit();
  hand.goToInit();

  //Compute handLocalGraspPoints from graspPoints and objCoords:
  Coords handLocalCoord;
  for(int i = 0; i < graspPoints.size(); i++)
  {
    handLocalCoord.x = graspPoints[i].x - objCoords.x;
    handLocalCoord.y = graspPoints[i].y - objCoords.y;
    handLocalGraspPoints.push_back(handLocalCoord);
  }

  if(handLocalGraspPoints.size() == 3) //If it is a three-finger-grasp
  {
    //Find angle to rotate with, such that x-coord. of finger B is ~0:
    wristRotation = -90.0*deg2rad - atan2(handLocalGraspPoints[1].y, handLocalGraspPoints[1].x); // -90[deg] - angleOfVectorB[deg] is how much to rotate vectors and wrist of UR

    //Rotate vectors:
    handLocalGraspPoints[0].x = cos(wristRotation) * handLocalGraspPoints[0].x - sin(wristRotation) * handLocalGraspPoints[0].y;
    handLocalGraspPoints[1].x = cos(wristRotation) * handLocalGraspPoints[0].x - sin(wristRotation) * handLocalGraspPoints[0].y;
    handLocalGraspPoints[2].x = cos(wristRotation) * handLocalGraspPoints[0].x - sin(wristRotation) * handLocalGraspPoints[0].y;
  }
  else if(handLocalGraspPoints.size() == 2)
  {
    //Find angle to rotate with, such that y-coord. of finger A is ~0:
    wristRotation = - atan2(handLocalGraspPoints[0].y, handLocalGraspPoints[0].x);
    handLocalGraspPoints[0].x = cos(wristRotation) * handLocalGraspPoints[0].x - sin(wristRotation) * handLocalGraspPoints[0].y;
    handLocalGraspPoints[1].x = cos(wristRotation) * handLocalGraspPoints[0].x - sin(wristRotation) * handLocalGraspPoints[0].y;
  }

  cout << "Ready to grasp. Press ENTER to grasp." << endl;
  cin.get();

  




  return 0;
}
