#include <iostream>
#include "URControl.hpp"
#include <string>
#include <unistd.h>


using namespace std;

int main()
{
  cout << "Welcome to UR precision test." << endl;

  try
  {
    string ip = "192.168.100.4";
    int port = 30002;

    URControl ur5(ip, port);

    ur5.moveToInit();

    cout << "Press ENTER to continue" << endl;
    cin.get();
    ur5.moveRel(0.0,0.0,-389.0);
    ur5.moveRel(0.0,0.0, 50.0);

    for(int i = 0; i < 8; i++)
    {
      cout << "Press ENTER to continue" << endl;
      cin.get();
      ur5.moveRel(20.0, 0.0, 0.0);
      ur5.moveRel(0.0, 0.0, -50.0);
      ur5.moveRel(0.0, 0.0, 50.0);
    }

    cout << "Press ENTER to continue" << endl;
    cin.get();
    ur5.moveRel(-160.0, 0.0, 0.0);
    ur5.moveRel(0.0, 0.0, -50.0);
    ur5.moveRel(0.0, 0.0, 50.0);

    for(int i = 0; i < 8; i++)
    {
      cout << "Press ENTER to continue" << endl;
      cin.get();
      ur5.moveRel(-20.0, 0.0, 0.0);
      ur5.moveRel(0.0, 0.0, -50.0);
      ur5.moveRel(0.0, 0.0, 50.0);
    }

    ur5.moveRel(160.0, 0.0, 0.0);
    ur5.moveRel(0.0, 0.0, -50.0);
    ur5.moveRel(0.0, 0.0, 50.0);

  }
  catch(const char e[])
  {
    cout << "\033[1;31m ERROR: \033[0m" << e << endl;
  }




  return 0;
}
