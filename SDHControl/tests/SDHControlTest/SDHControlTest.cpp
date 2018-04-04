#include <iostream>
#include "SDHControl.hpp"
#include <string>
#include <unistd.h>


using namespace std;

int main()
{
  double A = 52.5;
  double B = 52.5;
  double C = 52.5;

  cout << "Welcome to URControlTest.cpp" << endl;
  cout << "Calculating a concentric grasp for distances:" << endl;
  cout << "Finger A: " << A << endl << "Finger B: " << B << endl << "Finger C: " << C << endl;

  try
  {
    SDHControl hand(0, 115200, 0.5);
    cout << "Going to init." << endl;
    hand.goToInit();

    cout << "Press any key and ENTER to continue" << endl;
    string dummy;
    cin >> dummy;

    cout << "Grasping" << endl;
    hand.grasp(A, B, C, 45*deg2rad);
  }
  catch(const char e[])
  {
    cout << e << endl;
  }


  return 0;
}
