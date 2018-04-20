#include <iostream>
#include "URControl.hpp"
#include "settings.hpp"
#include <string>
#include <unistd.h>


using namespace std;

int main()
{
  string ip = "192.168.100.4";
  int port = 30002;
  double angle;

  try
  {
    URControl ur5(ip, port);

    ur5.moveToInit();

    cout << "Welcome to WristRotateTest.cpp. Enter a desired wrist-angle (degrees):";
    cin >> angle;
    angle *= deg2rad;
    ur5.setWristAngle(angle);
    cout << "Wrist is now rotated" << endl;
  }
  catch(const char e[])
  {
    cout << e << endl;
  }

  return 0;
}
