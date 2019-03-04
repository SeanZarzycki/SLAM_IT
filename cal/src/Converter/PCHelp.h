#pragma once


#include <vector>
#include <string>
#include <random>


class Frame
{
public:
	// declaration
	Frame(std::string);
	~Frame();

	// get functions
	int id();
	long time();
	int size();
	float c2w(int, int);
	float fx();
	float fy();
	float cx();
	float cy();

private:
	// frame id
	int id_v;
	// frame shell id?
	int inc_id;
	// time stamp
	long time_v;

	// number of points associated with frame ?
	int sz, sz2, sz3;

	// camera to world transformation matrix
	float c2w_v[3][4];

	// camera parameters
	float fx_v, fy_v, cx_v, cy_v;
};




class Point
{
public:
	// declaration
	Point();
	Point(std::string);
	Point(std::string, Frame*);
	~Point();
	
	// change which pixel for dens
	void setP(int);
	// calculates world position using camera model
	void applyFrame(Frame*);
    void applyFrame(Frame*, bool);
	// returns whether "applyFrame" has been called yet
	bool isFrameApplied();

	// get functions
	int id();
	long time();
	int frame_id();
	float u();
	float v();
	float d();
	float c();
	float c(int);
	float std();

	// get functions after frame is applied
	float x();
	float y();
	float z();
	Frame* getFrame();

private:
	// constructor helper
	void init(std::string);

	// point id
	int id_v;
	// time stamp
	long time_v;
	// associated frame id
	int fid;

	// image x coordinate
	int iu;
	// image y coordinate
	int iv;
	// image depth value
	float depth;
	// color value(s)
	std::vector<float> col;
	// mean color value
	float mc;
	// uncertainty
	float std_v;
	// number of good points?
	int sz;

	// has frame been applied?
	bool asfr;
	// pointer to frame that was applied
	Frame* fr;
	// world x coordinate
	float px;
	// world y coordinate
	float py;
	// world z coordinate
	float pz;
};





void textscan(std::string str, std::string del, std::vector<size_t>* ind);