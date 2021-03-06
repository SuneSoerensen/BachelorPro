#include <iostream>
#include "URControl.hpp"
#include <string>
#include <unistd.h>


using namespace std;

int main()
{
  cout << "Welcome to URControlTest.cpp" << endl;
  cout << "Valid commands (command | description): " << endl;
  cout << "tp   | Print current tool position" << endl;
  cout << "mov  | Run move-program" << endl;
  cout << "home | Run home-program" << endl;
  cout << "init | Run init-program" << endl;
  cout << "rel  | Run moveRel()" << endl;
  cout << "rot  | Run setWristAngle()" << endl;
  cout << "Invalid command will terminate program" << endl;

  string ip = "192.168.100.4";
  int port = 30002;

  URControl ur5(ip, port);

  //ur5.connect();

  string command;

  while(1)
  {
    cout << "Enter command:";
    cin >> command;

    if(command == "tp")
    {
      //ur5.updateCurrToolPos();
      ur5.printcurrToolPos();
    }
    else if(command =="mov")
    {
      ur5.moveToInit();
      ur5.moveRel(0.3,0,0);
      ur5.moveRel(-0.3,0,0);
      ur5.moveRel(0,0.3,0);
      ur5.moveRel(0,-0.3,0);
      ur5.moveRel(0,0.3,0.2);
      ur5.moveRel(0,-0.3,-0.2);
      ur5.moveToHome();
    }
    else if(command == "init")
    {
      ur5.moveToInit();
    }
    else if(command == "home")
    {
      ur5.moveToHome();
    }
    else if(command == "rel")
    {
      double x;
      double y;
      double z;

      cout << "Enter x:";
      cin >> x;
      cout << "Enter y:";
      cin >> y;
      cout << "Enter z:";
      cin >> z;

      try
      {
        cout << "Moving..." << endl;
        ur5.moveRel(x,y,z);
        ur5.printcurrToolPos();

      }
      catch(const char e[])
      {
        cout << e << endl;
      }

    }
    else if(command == "rot")
    {
      cout << "Please enter an angle in deg.: ";
      double angle;
      cin >> angle;

      angle *= deg2rad;
      cout << "Rotating wrist." << endl;
      ur5.setWristAngle(angle);
   }
    else
    {
      cout << "Invalid command!" << endl;
    }
  }





  return 0;
}
