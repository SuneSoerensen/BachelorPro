#include <iostream>
#include "SDHControl.hpp"
#include "URControl.hpp"
#include <string>
#include <unistd.h>


using namespace std;

int main()
{
  string wishToCont;

  //For UR-arm
  string ip = "192.168.100.4";
  int port = 30002;
  double armRelZ;

  //For hand
  double distA;
  double distB;
  double distC;
  double howMuchLower;



  cout << "Welcome to CustGraspHeight.cpp" << endl;

  try
  {
    SDHControl hand("/dev/pcanusb32");

    URControl ur(ip, port);
    ur.connect();
  }
  catch(const char e[])
  {
    cout << e << endl;
  }

  cout << "Enter distance for finger A: ";
  cin >> distA;
  cout << "Enter distance for finger B: ";
  cin >> distB;
  cout << "Enter distance for finger C: ";
  cin >> distC;

  while(true)
  {
    cout << "Moving arm to init" << endl;
    ur.moveToInit();
    cout << "Moving hand to init" << endl;
    sdh.goToInit();
    cout << "Enter a grasping height offset: ";
    cin >> howMuchLower;
    cout << "Enter relative z-coordinate for arm: ";
    cin >> armRelZ;

    try
    {
      cout << "Moving arm down" << endl;
      ur.moveRel(0.0, 0.0, armRelZ);
      cout << "Grasping" << endl;
      hand.grasp(distA, distB, distC, howMuchLower);
      cout << "Moving arm up to init + 100mm" << endl;
      ur.moveRel(0.0, 0.0, -(armRelZ+100));
      cout << "Fully stopping hand" << endl;
      hand.fullStop();
    }
    catch(const char e[])
    {
      cout << e << endl;
    }

    cout << "Do you wish to repeat? (y/n): ";
    cin >> wishToCont;

    if(wishToCont != "y")
    {
      cout << "Terminating" << endl;
      break;
    }
  }


  return 0;
}
