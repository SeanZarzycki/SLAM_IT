#include "pch.h"
#include "PCHelp.h"

#include <vector>
#include <string>



int staticPattern[8][2] = {{0,-2},	  {-1,-1},	   {1,-1},		{-2,0},		 {0,0},		  {2,0},	   {-1,1},		{0,2}};

/*
Frame Class
*/
Frame::Frame(std::string str)
{
	std::vector<size_t>* ind = new std::vector<size_t>();
	textscan(str, ", ", ind);


	if (ind->size() == 22)
	{
		// id
		size_t temp = ind->at(0) + 2;
		id_v = std::stoi(str.substr(0, temp - 2));
		// inc id ?? not entirely sure what this is
		inc_id = std::stoi(str.substr(temp, ind->at(1) - temp));
		temp = ind->at(1) + 2;

		// time stamp
		time_v = std::stol(str.substr(temp, ind->at(2) - temp));
		temp = ind->at(2) + 2;

		// number of points
		sz = std::stoi(str.substr(temp, ind->at(3) - temp));
		temp = ind->at(3) + 2;
		// number of points (HSsz?)
		sz2 = std::stoi(str.substr(temp, ind->at(4) - temp));
		temp = ind->at(4) + 2;
		// number of points (IPsz?  Incomplete points?)
		sz3 = std::stoi(str.substr(temp, ind->at(5) - temp));
		temp = ind->at(5) + 2;

		// Camera to World Transformation Matrix
		for (int i = 0;i < 3;i++)
			for (int j = 0;j < 4;j++)
			{
				c2w_v[i][j] = std::atof(str.substr(temp, ind->at(6 + i * 4 + j) - temp).c_str());
				temp = ind->at(6 + i * 4 + j) + 2;
			}

		// focal parameters
		fx_v = std::atof(str.substr(temp, ind->at(18) - temp).c_str());
		temp = ind->at(18) + 2;
		fy_v = std::atof(str.substr(temp, ind->at(19) - temp).c_str());
		temp = ind->at(19) + 2;
		cx_v = std::atof(str.substr(temp, ind->at(20) - temp).c_str());
		temp = ind->at(20) + 2;
		cy_v = std::atof(str.substr(temp, ind->at(21) - temp).c_str());
	}

	delete ind;
}
Frame::~Frame()  { }
int Frame::id() { return id_v; }
long Frame::time() { return time_v; }
int Frame::size() { return sz; }
float Frame::c2w(int i, int j) { return c2w_v[i][j]; }
float Frame::fx() { return fx_v; }
float Frame::fy() { return fy_v; }
float Frame::cx() { return cx_v; }
float Frame::cy() { return cy_v; }

/*

	int id;
	int inc_id;
	long time;

	int sz, sz2, sz3;

	float c2w[3][4];

	float fx, fy, cx, cy;

*/




/*
Point Class
*/
Point::Point()
{

}
Point::Point(std::string str)
{
	init(str);
}
Point::Point(std::string str, Frame* frame)
{
	init(str);
	applyFrame(frame);
}
Point::~Point()  { }
void Point::init(std::string str)
{
	asfr = false;

	px = 0;
	py = 0;
	pz = 0;

	std::vector<size_t>* ind = new std::vector<size_t>();
	textscan(str, ", ", ind);


	if (ind->size() == 16)
	{
		size_t temp = ind->at(0) + 2;
		fid = std::stoi(str.substr(0, temp - 2));
		id_v = std::stoi(str.substr(temp, ind->at(1) - temp));
		temp = ind->at(1) + 2;

		time_v = std::stol(str.substr(temp, ind->at(2) - temp));
		temp = ind->at(2) + 2;

		iu = std::stoi(str.substr(temp, ind->at(3) - temp));
		temp = ind->at(3) + 2;
		iv = std::stoi(str.substr(temp, ind->at(4) - temp));
		temp = ind->at(4) + 2;
		depth = 1 / std::atof(str.substr(temp, ind->at(5) - temp).c_str());
		temp = ind->at(5) + 2;


		for (int j = 0;j < 8;j++)
		{
			col.push_back(std::atof(str.substr(temp, ind->at(6 + j) - temp).c_str()));
			temp = ind->at(6 + j) + 2;
		}
		mc = col[4];

		std_v = std::atof(str.substr(temp, ind->at(14) - temp).c_str());
		temp = ind->at(14) + 2;
		sz = std::stoi(str.substr(temp, ind->at(15) - temp));
	}

	delete ind;
}

void Point::setP(int i)
{
	iu += staticPattern[i][0];
	iv += staticPattern[i][1];
	mc = col[i];
}
void Point::applyFrame(Frame* frame)
{
    applyFrame(frame, false);
}
void Point::applyFrame(Frame* frame, bool dens)
{
	asfr = true;

	fr = frame;

	float tx, ty, tz;
	tx = depth * (iu - frame->cx()) / frame->fx();
	ty = depth * (iv - frame->cy()) / frame->fy();
	tz = depth;

	float p[3];
	for (int i = 0;i < 3;i++)
		p[i] = frame->c2w(i, 0) * tx + frame->c2w(i, 1) * ty + frame->c2w(i, 2) * tz + frame->c2w(i, 3);
	px = p[0];
	py = p[1];
	pz = p[2];
}
bool Point::isFrameApplied() { return asfr; }

int Point::id() { return id_v; }
long Point::time() { return time_v; }
int Point::frame_id() { return fid; }
float Point::u() { return iu; }
float Point::v() { return iv; }
float Point::d() { return depth; }
float Point::c() { return mc; }
float Point::c(int i) { return col[i]; }
float Point::std() { return std_v; }

float Point::x() { return px; }
float Point::y() { return py; }
float Point::z() { return pz; }
Frame* Point::getFrame() { return fr; }






/*
Helper Functions
*/
void textscan(std::string str, std::string del, std::vector<size_t>* ind)
{
	size_t temp = str.find(del);
	while (temp != std::string::npos)
	{
		(*ind).push_back(temp);
		temp = str.find(del, temp + del.length());
	}
	(*ind).push_back(str.length());
}