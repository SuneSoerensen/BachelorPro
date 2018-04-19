#include <iostream>
#include "SDHControl.hpp"
#include "URControl.hpp"
#include <string>


using namespace std;

int main()
{
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
      cout << "Moving arm to init..." << endl;
      ur5.moveToInit();

      cout << "Enter anything and enter to continue.." << endl;
      cin >> dummy;
      cout << "Moving to position..." << endl;
      ur5.moveRel(0,0,-80);
      cout << "Grasping..." << endl;
      hand.grasp(60.62, 35.0, 0.0, -70, -60.62, 35.0); //Yellow plastic pot
      //hand.grasp(33.0, 56.5, 0.0, -56.5, -33.0, 56.5); //Wood-brick
    }
  }
  catch(const char e[])
  {
    cout << e << endl;
  }

  return 0;
}
