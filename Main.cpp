#include <iostream>

#include "SDHControl.hpp"
#include "URControl.hpp"
#include "Vision.hpp"

using namespace std;

int main()
{
  try
  {
    bool running = true;

    //Instatiate objects:
    SDHControl hand(0, 115200, 0.5);
    URControl ur("192.168.100.4",30002);
    Vision vis;

    //Declare variables:
    Coords objCoords;
    vector<Coords> graspPoints; //In real-worl coordinates
    vector<Coords> handLocalGraspPoints; //In SDH-coordinates (where grasp-focus is (0,0))
    double wristRotation;
    double absWristRotation;
    Coords handLocalCoord;
    double SDHheight;
    double rotX;
    double rotY;

    cout << "Welcome to grasp-utility! Place calibration-marker and press ENTER to continue." << endl;
    cin.get();
    cout << "Homing arm and calibrating vision..." << endl;
    ur.moveToHome();
    vis.Calib();
    while(running)
    {
      cout << "Done calibrating. Remove calibration-marker and place target-object. Then, press ENTER to compute grasp." << endl;
      cin.get();
      cin.get();
      ur.moveToHome();
      cout << "Computing grasp.." << endl;
      vis.CalcGrasp();
      objCoords = vis.GetGraspFocus();
      graspPoints = vis.GetGraspPoints();
      cout << "Found real-world coordinates for target-object: (" << objCoords.x << ";" <<  objCoords.y << ")." << "Press ENTER to initialize hand and arm.." << endl;
      cin.get();
      cout << "Initializing hand and arm..." << endl;
      ur.moveToInit();
      hand.goToInit();

      //Compute handLocalGraspPoints from graspPoints and objCoords:
      handLocalGraspPoints.resize(0);
      for(int i = 0; i < graspPoints.size(); i++)
      {
        handLocalCoord.x = graspPoints[i].x - objCoords.x;
        handLocalCoord.y = graspPoints[i].y - objCoords.y;
        handLocalGraspPoints.push_back(handLocalCoord);
      }


      if(handLocalGraspPoints.size() == 3) //If it is a three-finger-grasp
      {
        //Find angle to rotate with, such that x-coord. of finger B is ~0:
        wristRotation = 270.0*deg2rad - atan2(handLocalGraspPoints[1].y, handLocalGraspPoints[1].x); // -90[deg] - angleOfVectorB[deg] is how much to rotate vectors
        absWristRotation = acos(((double)handLocalGraspPoints[1].x*(-1.0)+(double)handLocalGraspPoints[1].y*(0.0))/(sqrt(pow((double)handLocalGraspPoints[1].x,2)+pow((double)handLocalGraspPoints[1].y,2))*1.0));
        absWristRotation -= 16.0*deg2rad;

        cout << "Grasp points:";
        for(int i = 0; i < graspPoints.size(); i++)
          cout << " (" << graspPoints[i].x << ";" << graspPoints[i].y << ") ";
        cout << " Rotation =" << absWristRotation*rad2deg << endl;
      }
      else if(handLocalGraspPoints.size() == 2)
      {
        //Find angle to rotate with, such that y-coord. of finger A is ~0:
        wristRotation = - atan2(handLocalGraspPoints[1].y, handLocalGraspPoints[1].x);
        absWristRotation = acos(((double)handLocalGraspPoints[1].x*(0.0)+(double)handLocalGraspPoints[1].y*(-1.0))/(sqrt(pow((double)handLocalGraspPoints[1].x,2)+pow((double)handLocalGraspPoints[1].y,2))*1.0));
        absWristRotation -= 10.0*deg2rad;

        /*cout << "handLocalGraspPoints:";
        for(int i = 0; i < handLocalGraspPoints.size(); i++)
          cout << " (" << handLocalGraspPoints[i].x << ";" << handLocalGraspPoints[i].y << ") ";
        cout << " Rotation =" << absWristRotation*rad2deg << endl;*/
      }
      else
      {
        throw("[Main:] incorrect number of grasping points!");
      }

      for(int i = 0; i < handLocalGraspPoints.size(); i++)
      {
        rotX = cos(wristRotation) * handLocalGraspPoints[i].x - sin(wristRotation) * handLocalGraspPoints[i].y;
        rotY = sin(wristRotation) * handLocalGraspPoints[i].x + cos(wristRotation) * handLocalGraspPoints[i].y;

        handLocalGraspPoints[i].x = rotX;
        handLocalGraspPoints[i].y = rotY;
      }

      //cout << "Ready to grasp. Press ENTER to move to position and pre-grasp." << endl;
      //cin.get();

      ur.moveRel(objCoords.x, objCoords.y, 0.0); //Hand above object
      ur.setWristAngle(-absWristRotation); //Rotate wrist

      if(handLocalGraspPoints.size() == 3) //If it is a three-finger-grasp
      {
        cout << "Going to pre-grasp configuration." << endl;
        SDHheight = hand.grasp(handLocalGraspPoints[2].x , handLocalGraspPoints[2].y, handLocalGraspPoints[1].x, handLocalGraspPoints[1].y, handLocalGraspPoints[0].x, handLocalGraspPoints[0].y, true) - INIT_POS_Z*1000.0; //Pregrasp and save height

        cout << "Press ENTER to grasp object." << endl;
        cin.get();
        cout << "Grasping." << endl;
        ur.moveRel(0.0, 0.0, SDHheight);
        hand.grasp(handLocalGraspPoints[2].x , handLocalGraspPoints[2].y, handLocalGraspPoints[1].x, handLocalGraspPoints[1].y, handLocalGraspPoints[0].x, handLocalGraspPoints[0].y, false);
      }
      else if(handLocalGraspPoints.size() == 2)
      {
        cout << "Going to pre-grasp configuration." << endl;
        SDHheight = hand.grasp(handLocalGraspPoints[1].x , handLocalGraspPoints[1].y, handLocalGraspPoints[0].x, handLocalGraspPoints[0].y, true) - INIT_POS_Z*1000.0; //Pregrasp and save height
        cout << "Press ENTER to grasp object." << endl;
        cin.get();
        cout << "Grasping." << endl;

        rotX = - sin(wristRotation) * SDH_FINGER_BASE_OFF_Y;
        rotY =   cos(wristRotation) * SDH_FINGER_BASE_OFF_Y;

        /*DEBUG*/ cout << "\033[1;33m [SDHControl::grasp] DEBUG: \033[0m" << "rotX = " << rotX << "; rotY = " << rotY << endl;

        ur.moveRel(rotX, rotY, SDHheight);
        hand.grasp(handLocalGraspPoints[1].x , handLocalGraspPoints[1].y, handLocalGraspPoints[0].x, handLocalGraspPoints[0].y, false);
      }

      if(hand.getIsValidGrasp())
      {
        cout << "It was a valid grasp" << endl;
        ur.moveToInit();
        //ur.moveRel(0.0, 0.0, 100.0);
        cout << "Rotating wrist to 60 deg." << endl;
        ur.setWristAngle(60.0*deg2rad);
        cout << "Putting object back down." << endl;
        ur.moveRel(objCoords.x, objCoords.y, SDHheight);
        hand.goToInit();
      }
      else
      {
        cout << "It was an invalid grasp" << endl;
        hand.fullStop();
        ur.moveRel(0.0,0.0,-SDHheight);
      }

      cout << "Do you wish to run again (1/0)?" << endl;
      cin >> running;
    }

  }
  catch(const char e[])
  {
    cout << "\033[1;31m ERROR: \033[0m" << e << endl;
  }

  return 0;
}
