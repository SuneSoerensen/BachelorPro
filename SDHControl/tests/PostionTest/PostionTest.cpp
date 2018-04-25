#include <iostream>
#include "SDHControl.hpp"
#include <string>
#include <unistd.h>


using namespace std;

int main()
{
  cout << "Welcome to PostionTest.cpp" << endl;

  double xA = 70.0;
  double yA = 70.0;

  double xB = 0.0;
  double yB = -70.0;

  double xC = -70.0;
  double yC = 70.0;

  SDHControl hand;

  try
  {
    for(int i = 40; i <= 50; i += 5)
    {
      for(int j = 40; j <= 50; j += 5)
      {
        for(int k = 40; k <= 50; k += 5)
        {
          for(int l = 40; l <= 50; l += 5)
          {
            xA = i;
            yA = j;

            yB = -l;

            xC = -i;
            yC = k;
            cout << xA << ";" << yA << ";";
            cout << xB << ";" << yB << ";";
            cout << xC << ";" << yC << endl;

            hand.grasp(xA,yA,xB,yB,xC,yC,false);
          }
        }
      }
    }
  }
  catch(const char e[])
  {
    cout << "\033[1;31m ERROR: \033[0m" << e << endl;
  }


  return 0;
}
