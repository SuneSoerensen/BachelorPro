#include <iostream>
#include "SDHControl.hpp"
#include <string>
#include <unistd.h>


using namespace std;

int main()
{
  cout << "Welcome to PostionTest.cpp" << endl;

  double xA = 40;
  double yA = 40;

  double xB = 0.0;
  double yB = -40;

  double xC = -40;
  double yC = 40;

  cout << "\033[1;33m DEBUG: \033[0m" << "(" << xA << ";" << yA<< ")";
  cout << "(" << xB << ";" << yB << ")";
  cout << "(" << xC << ";" << yB << ")" << endl;

  SDHControl hand;

  hand.grasp(xA,yA,xB,yB,xC,yC,false);


  return 0;
}
