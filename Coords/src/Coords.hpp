#pragma once

using namespace std;



class Coords //Coordinates (2D)
{
public:
	//==================
	//   Constructors
	//==================
	Coords();
	Coords(int anX, int aY);

	//====================
	//   "set"-function
	//====================
	void Set(int anX, int aY);

	//=====================
	//   math-functions
	//=====================
	//-------------
	// With Coords
	//-------------
	Coords Add(Coords aCoords);
	Coords Sub(Coords aCoords);
	Coords Mul(Coords aCoords);
	Coords Div(Coords aCoords);

	//-------------
	// With scalar
	//-------------
	Coords Add(int aScalar);
	Coords Sub(int aScalar);
	Coords Mul(int aScalar);
	Coords Div(int aScalar);

	//===================
	//   Deconstructor
	//===================
	~Coords();

	//================
	//   Attributes
	//================
	int x;
	int y;
};

