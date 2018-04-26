#include "URControl.hpp"
#include <fstream>
#include <unistd.h>
#include <rwhw/universalrobots/UniversalRobotsData.hpp>
//#include <rw/math/Vector3D.hpp>
#include <stdlib.h>
#include <array>

USE_ROBWORK_NAMESPACE

using namespace robwork;
using namespace rwhw;

using namespace std;

URControl::URControl()
{
  //Default settings
  port = 30002;
  ip = "192.168.100.4";
  haveBeenToInit = 0;
}

URControl::URControl(string anIp, int APort)
{
  port = APort;
  ip = anIp;
  haveBeenToInit = 0;
  connect();
}

void URControl::connect()
{
  ur.connect(ip, port);
  ur.start();
}

void URControl::disconnect()
{
  ur.stop();
  ur.disconnect();
}

void URControl::sendScript(string fileName)
{
  ur.sendScriptFile(fileName);
  if(URCONTROL_MODE)
  {
    cout << "\033[1;33m DEBUG: \033[0m" << "Script \"" << fileName << "\" sent!" << endl;
  }
}

void URControl::moveToInit()
{
  if(state != STATE_INIT)
  {
    /*string root(getenv("URCON_ROOT"));
    string file = "goToInit.txt";
    string fileName = root + file;*/

    string fileName = "goToInit.txt";

    //Check if fileName exists:
    ifstream fileTest(fileName);
    if(!fileTest.good())
      throw("[URControl::moveToInit]: goToInit-script does not exist!");

    sendScript(fileName);
    haveBeenToInit = 1;
    state = STATE_INIT;
    usleep(5100000);

    currToolPos[0] = INIT_POS_X;
    currToolPos[1] = INIT_POS_Y;
    currToolPos[2] = INIT_POS_Z;
  }
}

void URControl::moveToHome()
{
  if(state != STATE_HOME)
  {
    /*string root(getenv("BACH_ROOT"));
    if(URCONTROL_MODE)
        cout << "\033[1;33m DEBUG: \033[0m" << "Created fileName with success" << endl;
    string file = "goToHome.txt";
    string fileName = root + file;*/

    string fileName = "goToHome.txt";

    //Check if fileName exists:
    ifstream fileTest(fileName);
    if(!fileTest.good())
      throw("[URControl::moveToInit]: goToHome-script does not exist!");

    sendScript(fileName);
    haveBeenToInit = 0;
    state = STATE_HOME;
    usleep(5100000);
  }
}

void URControl::moveRel(double anX, double aY, double aZ)
{
  //State that UR is not in home or init:
  state = STATE_OTHER;

  //Convert from mm to m, as URScripts are in m:
  double x = anX/1000.0;
  double y = aY/1000.0;
  double z = aZ/1000.0;

  //Rotate x- and y-coordinates -45 deg around (0,0):
  double rotX = cos(OFFSET_ANGLE)*x - sin(OFFSET_ANGLE)*y;
  double rotY = sin(OFFSET_ANGLE)*x + cos(OFFSET_ANGLE)*y;

  //Calculate absolute coordinates:
  double absX = currToolPos[0] + rotX;
  double absY = currToolPos[1] + rotY;
  double absZ = currToolPos[2] + aZ;

  if(URCONTROL_MODE)
    cout << "\033[1;33m DEBUG: \033[0m" << "absX [mm] = " << absX*1000.0 << " absY [mm] = " << absY*1000.0 << endl;

  //Security checks:
  if(!haveBeenToInit)
  {
    throw("[URControl::moveRel]: Cannot use this function w/o having been to init-conf. first!");
  }

  if(!checkBounds(absX, absY, absZ))
  {
    throw("[URControl::moveRel]: New coordinates are out of bounds!");
  }

  //Generate scriptfile:
  string fileName;

  if(URCONTROL_MODE)
  {
    fileName = "moveRelScript" + to_string(numOfMoves) + ".txt";
  }
  else
  {
    fileName = "moveRelScript.txt";
  }

  ofstream out(fileName, ofstream::out);

  out << "HOST=" << ip << "\n" << "PORT=" << port << "\n" << "def moveRel():\n";
  out << "\t pos = get_forward_kin() \n";
  out << "\t pos[0] = " << to_string(absX) << "\n";
  out << "\t pos[1] = " << to_string(absY) << "\n";
  out << "\t pos[2] = " << to_string(absZ)  << "\n";
  out << "\tmovel(pos," << ACC << ", " << VEL << "," << MOVTIME << "," << BLENDR << ")\n";
  out << "end\n";

  out.close();


  //Send scriptfile to perform movement:
  sendScript(fileName);

  //Update current tool position:
  currToolPos[0] = absX;
  currToolPos[1] = absY;
  currToolPos[2] = absZ;

  //Wait for movement to finish (MOVTIME) + 0.1s:
  usleep((MOVTIME*1000000)+100000);
}

void URControl::setWristAngle(double anAngle)
{
  if(abs(anAngle) > MAX_WRIST_ANGLE)
    throw("[URControl::setWristAngle]: Invalid angle (|angle| > 360 deg)!");

  //Generate script to rotate joint:
  ofstream out("rotateWristScript.txt", ofstream::out);

  out << "HOST=" << ip << "\n" << "PORT=" << port << "\n" << "def rotWrist():\n";
  out << "\t" << "pos = get_joint_positions()" << "\n";
  out << "\t" << "pos[5] =" << anAngle << "\n";
  out << "\t" << "textmsg(\"Rotating wrist\")" << "\n";
  out << "\t" << "movej(pos, 0.1, 0.1, 5, 0)" << "\n";
  out << "end \n";

  out.close();

  //Send script to perform movement:
  sendScript("rotateWristScript.txt");
}

bool URControl::checkBounds(double x, double y, double z)
{
  if(x < UR_MIN_X || x > UR_MAX_X)
  {
    throw("[URControl::checkBounds]: New x-coordinates are out of bounds!");
  }
  else if(y < UR_MIN_Y || y > UR_MAX_Y)
  {
    throw("[URControl::checkBounds]: New y-coordinates are out of bounds!");
  }
  else if(z < UR_MIN_Z || z > UR_MAX_Z)
  {
    throw("[URControl::checkBounds]: New z-coordinates are out of bounds!");
  }
  else if((x*x + y*y + z*z) <= R_SQUARED)
  {
    throw("[URControl::checkBounds]: Robot cannot reach so far!");
  }

  return true;
}

void URControl::printcurrToolPos()
{
  cout << "\033[1;33m DEBUG: \033[0m" << "Current tool position (x, y, z): ";
  for(int i = 0; i < (sizeof(currToolPos)/sizeof(double)); i++)
  {
    cout << currToolPos[i] << "  ";
  }
  cout << endl;
}

URControl::~URControl() {}
