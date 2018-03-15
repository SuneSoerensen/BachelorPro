#include <iostream>
#include "Vision.hpp"
#include <string>

using namespace std;



int main()
{
	try
	{
		Vision test;
		string run = "y";
		string mode;

		while (run == "y")
		{
			cout << "select ''Calib'', ''GraspRegs'' or ''GetCoords'' (c / r / g): ";
			cin >> mode;

			if (mode == "c")
			{
				test.Calib();
			}
			if (mode == "g")
			{
				Coords objCoords = test.GetObjCoords();
				cout << "real coords: " << objCoords.x << ";" << objCoords.y << endl;
			}
			if (mode == "r")
			{
				test.FindGraspRegs(test.graspRegsList, test.contourImage);
			}

			cout << "Do again?" << endl;
			cout << "(y / n): ";
			cin >> run;
		}
	}
	catch (const char errMess[])
	{
		cout << errMess << endl;
	}

	return 0;
}

