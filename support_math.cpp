#include "support_math.h"

vector3::vector3(){
	x = 0;
	y = 0;
	z = 0;
}

vector3::vector3(point3 p1,point3 p2){
	x = p2.x - p1.x;
	y = p2.y - p1.y;
	z = p2.z - p1.z;
}

vector3::vector3(float x,float y,float z){
	this->x = x;
	this->y = y;
	this->z = z;
}

point3::point3(){
	x=0;
	y=0;
	z=0;
}

point3::point3(float x,float y,float z){
	this->x = x;
	this->y = y;
	this->z = z;
}

vector3 crossproduct(vector3 b,vector3 c){
	vector3 result;
	result.x = b.y*c.z - b.z*c.y;
	result.y = b.z*c.x - b.x*c.z;
	result.z = b.x*c.y - b.y*c.x;
	return result;
};

point3 rotatePoint(point3 p,point3 o,vector3 v,float alpha){
	point3 newP;
	float C,s,c;

	C = 1 - cos(alpha);
	s = sin(alpha);
	c = cos(alpha);

	
	float Ax,Ay,Az,Bx,By,Bz,Cx,Cy,Cz;
	Ax = v.x*v.x*C + c;
	Ay = v.y*v.x*C + v.z*s;
	Az = v.z*v.x*C - v.y*s;
	Bx = v.x*v.y*C - v.z*s;
	By = v.y*v.y*C + c;
	Bz = v.z*v.y*C + v.x*s;
	Cx = v.x*v.z*C + v.y*s;
	Cy = v.y*v.z*C - v.x*s;
	Cz = v.z*v.z*C + c;

	
	newP.x = o.x + Ax*(p.x-o.x)+Bx*(p.y-o.y)+Cx*(p.z-o.z);
	newP.y = o.y + Ay*(p.x-o.x)+By*(p.y-o.y)+Cy*(p.z-o.z);
	newP.z = o.z + Az*(p.x-o.x)+Bz*(p.y-o.y)+Cz*(p.z-o.z);

	return newP;
};

float distance(point3 a,point3 b){
	return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z));

};

float norm(vector3 v){
	return sqrt(v.x*v.x+v.y*v.y+v.z*v.z);
};

vector3 unitVector(vector3 v){
	vector3 result;
	result.x = v.x/norm(v);
	result.y = v.y/norm(v);
	result.z = v.z/norm(v);
	return result;
};

vector3 pointToVector(point3 p){
	vector3 result;
	result.x = p.x;
	result.y = p.y;
	result.z = p.z;

	return result;
};
