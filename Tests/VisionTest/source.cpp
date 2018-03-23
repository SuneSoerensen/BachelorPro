#include <iostream>
#include "Vision.hpp"
#include <string>

using namespace std;



int main()
{
	try
	{
		cout << "================" << endl;
		cout << "   VisionTest" << endl;
		cout << "================" << endl;
		cout << endl;

		Vision test;
		string run = "y";
		string mode;

		while (run == "y")
		{
			cout << "select between:" << endl;
			cout << "(1): Calib()" << endl;
			cout << "(2): GetObjCoords()" << endl;
			cout << "(3): FindGraspPoints()" << endl;
			cout << "type input: ";
			cin >> mode;
			cout << endl;

			if (mode == "1")
			{
				test.Calib();
			}
			if (mode == "2")
			{
				Coords objCoords = test.GetObjCoords();
				cout << "real coords: " << objCoords.x << ";" << objCoords.y << endl;
				cout << endl;
			}
			if (mode == "3")
			{
				test.RunFindGraspPoints();
			}

			cout << "Do again?" << endl;
			cout << "select between:" << endl;
			cout << "(y): yes" << endl;
			cout << "(whatever): no" << endl;
			cout << "type input: ";
			cin >> run;
			cout << endl;
		}
	}
	catch (const char errMess[])
	{
		cout << errMess << endl;
	}

	return 0;
}

