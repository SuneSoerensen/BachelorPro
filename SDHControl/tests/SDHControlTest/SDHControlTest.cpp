#include <iostream>
#include "SDHControl.hpp"
#include <string>
#include <unistd.h>


using namespace std;

int main()
{
  double xA = 103.92;
  double yA = 60;

  double xB = 0;
  double yB = -120.0;

  double xC = -xA;
  double yC = yA;


  cout << "Welcome to URControlTest.cpp" << endl;
  try
  {
    SDHControl hand/*(0, 115200, 0.5)*/;
    //cout << "Going to init." << endl;
    //hand.goToInit();

    //cout << "Press any key and ENTER to continue" << endl;
    //string dummy;
    //cin >> dummy;

    cout << "Grasping" << endl;
    hand.grasp(xA, yA, xB, yB, xC, yC, false);
  }
  catch(const char e[])
  {
    cout << e << endl;
  }




  return 0;
}
