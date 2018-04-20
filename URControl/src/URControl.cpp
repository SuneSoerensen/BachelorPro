//
//  URControl.cpp
//  tutorial
//
//  Created by <author> on 21/02/2018.
//
//

#include "URControl.hpp"
#include <fstream>
#include <unistd.h>
#include <rwhw/universalrobots/UniversalRobotsData.hpp>
#include <rw/math/Vector3D.hpp>

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
    string fileName = getenv("URCON_ROOT");
    fileName += "goToInit.txt";
    sendScript(fileName);
    haveBeenToInit = 1;
    state = STATE_INIT;
    usleep(5100000);

    currToolPos[0] = INIT_POS_X;
    currToolPos[1] = INIT_POS_Y;
    currToolPos[2] = INIT_POS_Z;
    currToolPos[3] = INIT_POS_RX;
    currToolPos[4] = INIT_POS_RY;
    currToolPos[5] = INIT_POS_RZ;
  }
}

void URControl::moveToHome()
{
  if(state != STATE_HOME)
  {
    string fileName = getenv("URCON_ROOT");
    fileName += "goToHome.txt";
    sendScript(fileName);
    haveBeenToInit = 0;
    state = STATE_HOME;
    usleep(5100000);
  }
}

void URControl::moveRel(double anX, double aY, double aZ)
{
  //Declare that UR is not in home or init
  state = STATE_OTHER;

  //Convert from mm to m:
  double x = anX/1000.0;
  double y = aY/1000.0;
  double z = aZ/1000.0;

  //Rotate x- and y-coordinates -45 deg around (0,0):
  double rotX = cos(OFFSET_ANGLE)*x - sin(OFFSET_ANGLE)*y;
  double rotY = sin(OFFSET_ANGLE)*x + cos(OFFSET_ANGLE)*y;

  //Calculate absolute coordinates:
  double absX = currToolPos[0] + rotX;
  double absY = currToolPos[1] + rotY;

  if(URCONTROL_MODE)
    cout << "\033[1;33m DEBUG: \033[0m" << "absX [mm] = " << absX*1000.0 << " absY [mm] = " << absY*1000.0 << endl;

  //Security check:
  if(!haveBeenToInit)
  {
    throw("[URControl::moveRel]: Cannot use this function w/o having been to init-conf. first!");
  }

  //Security check:
  if(absX < UR_MIN_X || absX > UR_MAX_X)
  {
    if(URCONTROL_MODE)
      cout << "\033[1;33m DEBUG: \033[0m" << "absX = " << absX << endl;

    throw("[URControl::moveRel]: New x-coordinates are out of bounds!");
  }
  else if(absY < UR_MIN_Y || absY > UR_MAX_Y)
  {
    throw("[URControl::moveRel]: New y-coordinates are out of bounds!");
  }
  else if((currToolPos[2]+z) < UR_MIN_Z || (currToolPos[2]+z) > UR_MAX_Z)
  {
    throw("[URControl::moveRel]: New z-coordinates are out of bounds!");
  }
  else if(!checkBounds((currToolPos[0]+x),(currToolPos[1]+y),(currToolPos[2]+z)))
  {
    throw("[URControl::moveRel]: Robot cannot reach so far!");
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
  out << "\tpos=p[" << to_string(absX) << ", " << to_string(absY) << ", " << to_string(currToolPos[2]+z) << ", " << currToolPos[3] << ", " << currToolPos[4] << ", " << currToolPos[5] << "]\n";
  out << "\tmovel(pos," << ACC << ", " << VEL << "," << MOVTIME << "," << BLENDR << ")\n";
  //out << "\ttextmsg(\"Moved to position:" << to_string(currToolPos[0]+x) << ", " << to_string(currToolPos[1]+y) << ", " << to_string(currToolPos[2]+z) << ", " << currToolPos[3] << ", " << currToolPos[4] << ", " << currToolPos[5] << "\")\n";
  out << "end\n";

  out.close();


  //Send scriptfile to peform movement:
  sendScript(fileName);

  //Update current tool position:
  currToolPos[0] = absX;
  currToolPos[1] = absY;
  currToolPos[2] += z;

  usleep((MOVTIME*1000000)+100000); //Wait for movement to finish (MOVTIME) + 100000 µs (0.1 s)
  //updateCurrToolPos();
}

/*void URControl::updateCurrToolPos()
{
  UniversalRobotsData URdata;
  math::Vector3D<> toolPos;

  if(ur.hasData())
  {
    URdata = ur.getLastData();
    toolPos = URdata.toolPosition;
    currToolPos[0] = toolPos[0];
    currToolPos[1] = toolPos[1];
    currToolPos[2] = toolPos[2];*/

    ///*DEBUG*/ cout << "Toolpos: " << toolPos[0] << " " << toolPos[1] << " " << toolPos[2] << endl;
    ///*DEBUG*/ cout << "masterTemperature: " << URdata.masterTemperature << endl;
/*  }
  else
  {
    cout << "{WARNING} [URControl::updateCurrToolPos()]: UR had no data!" << endl;
  }
}*/

/*void URControl::moveAbs(double anX, double aY, double aZ)
{
  //Declare that UR is not in home or init
  state = STATE_OTHER;

  double relX = anX - currToolPos[0]*1000.0;
  double relY = aY - currToolPos[1]*1000.0;
  double relZ = aZ - currToolPos[2]*1000.0;

  moveRel(relX, relY, relZ);
}*/

void URControl::setWristAngle(double anAngle)
{
  if(abs(anAngle) > 360.0*deg2rad)
    throw("[URControl::setWristAngle]: Invalid angle (|angle| > 360 deg)!");

  ofstream out("rotateWristScript.txt", ofstream::out);

  out << "HOST=" << ip << "\n" << "PORT=" << port << "\n" << "def rotWrist():\n";
  out << "\t" << "pos = get_joint_positions()" << "\n";
  out << "\t" << "pos[5] =" << anAngle << "\n";
  out << "\t" << "textmsg(\"Rotating wrist\")" << "\n";
  out << "\t" << "movej(pos, 0.1, 0.1, 5, 0)" << "\n";
  out << "end\n";

  out.close();

  sendScript("rotateWristScript.txt");
}

bool URControl::checkBounds(double x, double y, double z)
{
  return ((x*x + y*y + z*z) <= R_SQUARED);
}

//Helpful debug functions:
void URControl::printcurrToolPos()
{
  cout << "Current tool position (x, y, z, rotX, rotY, rotZ): ";
  for(int i = 0; i < 6; i++)
  {
    cout << currToolPos[i] << "  ";
  }
  cout << endl;
}

URControl::~URControl() {}
