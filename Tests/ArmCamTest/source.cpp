#include <iostream>
#include "URControl.hpp"
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

    ur5.connect();

    /*****************
    * Vision setup
    *****************/
    Vision cp;

    /*****************
    *  Application
    *****************/

    //Firstly, make sure arm is in HOME position/configuration:
    cout << "Homing robot" << endl;
    ur5.moveToHome();

    usleep(1000000);
    //Calibrate camera:
    cout << "Calibrating camera" << endl;
    cp.Calib();

    string dummy;

    while(true)
    {
      cout << "Calibrate, move, home or init (c/m/h/i)?" << endl;
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
        Coords objCoords = cp.GetRealCoords(cp.GetObjCoords());
        cout << "Real coords of object: (" << objCoords.x << ";" << objCoords.y << ")" << endl;

        cout << "Moving to init" << endl;
        ur5.moveToInit();
        cout << "Moving to object with moveAbs(" << objCoords.x+REAL_TO_UR_OFFSET_X << " ; " << objCoords.y+REAL_TO_UR_OFFSET_Y << ";" <<  283 << ")"<< endl;
        ur5.moveAbs(objCoords.x+REAL_TO_UR_OFFSET_X, objCoords.y+REAL_TO_UR_OFFSET_Y, 283);
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
