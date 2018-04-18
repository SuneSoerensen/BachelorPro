#include <iostream>
#include "SDHControl.hpp"
#include "URControl.hpp"
#include <string>


using namespace std;

int main()
{
  double URHeight;
  SDHControl hand(0, 115200, 0.5);

  string ip = "192.168.100.4";
  int port = 30002;
  URControl ur5(ip, port);

  ur5.connect();

  cout << "Welcome to GraspControlTest.cpp" << '\n';

  string dummy;
  try
  {
    while(true)
    {
      cout << "Enter anything and enter to start.." << endl;
      cin >> dummy;
      cout << "Moving hand to init" << endl;
      hand.goToInit();
      cout << "Moving arm to init" << endl;
      ur5.moveToInit();

      cout << "Enter anything and enter to grasp" << endl;
      cin >> dummy;
      cout << "Pre-grasping..." << endl;
      //Yellow plastic pot:
      /*URHeight = hand.grasp(60.62, 35.0, 0.0, -70, -60.62, 35.0, true); //Pregrasp and save height
      ur5.moveAbs(-108.67, -485.36, URHeight);
      cout << "Grasping for real" << endl;
      hand.grasp(60.62, 35.0, 0.0, -70, -60.62, 35.0, false); //Grasp*/

      //Wood-brick:
      URHeight = hand.grasp(33.0, 56.5, 0.0, -56.5, -33.0, 56.5, true); //Pregrasp
      ur5.moveAbs(-108.67, -485.36, URHeight);
      cout << "Grasping for real" << endl;
      hand.grasp(33.0, 56.5, 0.0, -56.5, -33.0, 56.5, false); //Grasp

      //Difficult object:
      /*URHeight = hand.grasp(33.0, 50.0, 0.0, -50.0, -33.0, 50.0, true); //Pregrasp
      ur5.moveAbs(-108.67, -485.36, URHeight);
      cout << "Grasping for real" << endl;
      hand.grasp(33.0, 50.0, 0.0, -50.0, -33.0, 50.0, false); //Grasp*/

      cout << "Enter anything and enter to continue.." << endl;
      cin >> dummy;
      cout << "Moving arm to init" << endl;
      ur5.moveToInit();
      cout << "Enter anything and enter to put down object.." << endl;
      cin >> dummy;
      cout << "Moving down again" << endl;
      ur5.moveAbs(-108.67, -485.36, URHeight);
      cout << "Moving hand to init" << endl;
      hand.goToInit();

    }
  }
  catch(const char e[])
  {
    cout << e << endl;
  }

  return 0;
}
