#include "Coords.hpp"



//==================
//   Constructors
//==================
Coords::Coords()
{
	x = 0;
	y = 0;
}

Coords::Coords(int anX, int aY)
{
	x = anX;
	y = aY;
}

Coords::Coords(int aScalar)
{
	x = aScalar;
	y = aScalar;
}

//====================
//   "set"-function
//====================
void Coords::Set(int anX, int aY)
{
	x = anX;
	y = aY;
}

void Coords::Set(int aScalar)
{
	x = aScalar;
	y = aScalar;
}

//=====================
//   math-functions
//=====================
bool Coords::Eq(Coords aCoords)
{
	bool res = false;

	if (x == aCoords.x && y == aCoords.y)
		res = true;

	return res;
}

bool Coords::Eq(int aScalar)
{
	bool res = false;

	if (x == aScalar && y == aScalar)
		res = true;

	return res;
}

double Coords::Length()
{
	return sqrt(pow(x, 2) + pow(y, 2));
}

int Coords::Dot(Coords aCoords)
{
	return x*aCoords.x + y*aCoords.y;
}

//-------------
// With Coords
//-------------
Coords Coords::Add(Coords aCoords)
{
	Coords res(x + aCoords.x, y + aCoords.y);
	return res;
}

Coords Coords::Sub(Coords aCoords)
{
	Coords res(x - aCoords.x, y - aCoords.y);
	return res;
}

Coords Coords::Mul(Coords aCoords)
{
	Coords res(x * aCoords.x, y * aCoords.y);
	return res;
}

Coords Coords::Div(Coords aCoords)
{
	if (aCoords.x == 0 || aCoords.y == 0)
		throw("[Coords::Div()]: Divide by zero!");

	Coords res(x / aCoords.x, y / aCoords.y);
	return res;
}

//-------------
// With scalar
//-------------
Coords Coords::Add(int aScalar)
{
	Coords res(x + aScalar, y + aScalar);
	return res;
}

Coords Coords::Sub(int aScalar)
{
	Coords res(x - aScalar, y - aScalar);
	return res;
}

Coords Coords::Mul(int aScalar)
{
	Coords res(x * aScalar, y * aScalar);
	return res;
}

Coords Coords::Div(int aScalar)
{
	if (aScalar == 0)
		throw("[Coords::Div()]: Divide by zero!");

	Coords res(x / aScalar, y / aScalar);
	return res;
}

//===================
//   Deconstructor
//===================
Coords::~Coords()
{
}

