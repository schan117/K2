#ifndef _MATH_H
#define _MATH_H

#include <math.h>

const float PI=3.141592654;

class point2{
public:
	point2();
	point2(float x,float y);
	float x,y;
	
};

class point3{
public:
	point3();
	point3(float x,float y,float z);
	float x,y,z;
	
};

class vector3{
public:
	vector3();
	vector3(point3 p1,point3 p2);
	vector3(float x,float y,float z);
	float x,y,z;
};

vector3 crossproduct(vector3 b,vector3 c);
point3 rotatePoint(point3 p,point3 o,vector3 v,float alpha);
float distance(point3 a,point3 b);
float norm(vector3 v);
vector3 unitVector(vector3 v);
vector3 pointToVector(point3 p);

#endif