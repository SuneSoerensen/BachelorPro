#include <iostream>
#include "URControl.hpp"
#include "SDHControl.hpp"
#include "Vision.hpp"
#include <string>
#include <unistd.h>


using namespace std;

int main()
{
  try
  {
    /*****************
    * URControl setup
    *****************/
    string ip = "192.168.100.4";
    int port = 30002;

    URControl ur5(ip, port);

    /*****************
    * Vision setup
    *****************/
    Vision cp;

    /******************
    * SDHControl setup
    ******************/
    SDHControl hand(0, 115200, 0.5);
    double SDHheight;


    /*****************
    *  Application
    *****************/

    //Firstly, make sure arm is in HOME position/configuration:
    cout << "Homing robot" << endl;
    ur5.moveToHome();

    cout << "Press ENTER to calibrate camera." << endl;
    cin.get();
    cout << "Calibrating camera" << endl;
    cp.Calib();

    string dummy;

    while(true)
    {
      cout << "Calibrate, move and grasp, home or init (c/m/h/i)?" << endl;
      cin >> dummy;

      if(dummy == "c")
      {
        cout << "Homing robot" << endl;
        ur5.moveToHome();
        cout << "Calibrating camera" << endl;
        cp.Calib();
      }
      else if(dummy == "m")
      {
        Coords objCoords = cp.GetObjCoords();
        cout << "Real coords of object: (" << objCoords.x << ";" << objCoords.y << ")" << endl;

        cout << "Moving to init" << endl;
        ur5.moveToInit();
        cout << "Hand is going to init" << endl;
        hand.goToInit();
        cout << "Hand is going to pre-grasp conf." << endl;
        //Yellow plastic pot:
        SDHheight = hand.grasp(60.62, 35.0, 0.0, -70, -60.62, 35.0, true) - 433.02; //Pregrasp and save height
        cout << "Moving to target" << endl;
        ur5.moveRel(objCoords.x, objCoords.y, 0.0);
        ur5.moveRel(0.0, 0.0, SDHheight);
        cout << "Grasping for real" << endl;
        hand.grasp(60.62, 35.0, 0.0, -70, -60.62, 35.0, false); //Grasp
        if(hand.getIsValidGrasp())
        {
          ur5.moveToInit();
          hand.goToInit();
        }
        else
        {
          hand.fullStop();
        }
      }
      else if(dummy == "h")
      {
        cout << "Homing robot" << endl;
        ur5.moveToHome();
      }
      else if(dummy == "i")
      {
        cout << "Moving to init" << endl;
        ur5.moveToInit();
      }
      else
      {
        cout << "Invalid command! Terminating..." << endl;
        break;
      }
    }
  }
  catch(const char e[])
  {
    cout << e << endl;
  }





  return 0;

}
