#include <iostream>
#include "SDHControl.hpp"
#include <string>
#include <unistd.h>


using namespace std;

int main()
{
  double dist;
  string dummyVal;
  cout << "Welcome to URControlTest.cpp" << endl;

  try
  {
    SDHControl hand(0);

    while (true)
    {
      cout << "Going to init." << endl;
      hand.goToInit();
      cout << "Enter dist (>= 38.105):";
      cin >> dist;

      cout << "Grasping..." << endl;
      hand.grasp(dist, dist, dist);
      hand.fullStop();
      cout << "Enter anything and ENTER to continue.." << endl;
      cin >> dummyVal;
    }
  }
  catch(const char e[])
  {
    cout << e << endl;
  }


  return 0;
}
