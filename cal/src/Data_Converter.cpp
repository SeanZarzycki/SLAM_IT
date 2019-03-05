#include <iostream>
#include <fstream>

#include <string>
#include <cstring>
#include <stdlib.h>
#include <limits>

#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;

string head;
int dens, cc;
float dl, dh;

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

class CustPoint
{
public:
	// declaration
	CustPoint();
	CustPoint(std::string);
	CustPoint(std::string, Frame*);
	~CustPoint();
	
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
	float r();
	float r(int);
	float g();
	float g(int);
	float b();
	float b(int);
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
	// red value(s)
	std::vector<float> cr;
	// mean red value
	float mr;
	// green value(s)
	std::vector<float> cg;
	// green red value
	float mg;
	// blue value(s)
	std::vector<float> cb;
	// blue red value
	float mb;
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

void color_correct(std::vector<CustPoint> cloud, std::string file)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile (file, *pcd);

    for(size_t i = 0;i < cloud.size();i++)
    {
        pcd->points[i].r = cloud[i].r();
        pcd->points[i].g = cloud[i].g();
        pcd->points[i].b = cloud[i].b();
    }

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB> (file, *pcd, false);
}

void parseArgument(const char* arg)
{
	int option;
	float foption;
	char buf[1000];
	
	
	if(1==sscanf(arg,"name=%s",buf))
	{
		head = buf;
		return;
	}
    if(1==sscanf(arg,"dens=%d",&option))
	{
        if(option >= 0 && option <= 1)
            dens = option;
		return;
	}
    if(1==sscanf(arg,"dl=%f",&foption))
	{
        dl = foption;
		return;
	}
    if(1==sscanf(arg,"dh=%f",&foption))
	{
        dh = foption;
		return;
	}
	if(1==sscanf(arg,"cc=%d",&option))
	{
        cc = option;
		return;
	}


	if(strcmp(arg, "-h") != 0 && strcmp(arg, "--help"))
        printf("could not parse argument \"%s\"!!!!\n", arg);
    
    cout << "Help:\n"
         << "name=\"...\" is the input name for the two text files to be converted\n..._points.txt and ..._frames.txt must both exist for the conversion to work\nCan also include location if not in current folder\n\n"
         << "dens=0 is the default which will use the centers as locations for each point in the cloud, using the mean color\ndens=1 adds some randomness to better fill out the space. Each point becomes 8 points with some added noise based on its variance and depth"
		 << "dl is the minimum acceptable depth.  Any points with lower depth will be removed.  Default is 0."
		 << "dh is the maximum acceptable depth.  Any points with higher depth will be removed.  Default is Inf.\n\n";
}


int main( int argc, char** argv )
{
	dl = 0;
	dh = numeric_limits<float>::infinity();
    dens = 0;
	cc = 1;
	for(int i=1; i<argc;i++)
		parseArgument(argv[i]);
	if(argc == 1)
    {
        string temp = "--help";
        parseArgument(temp.c_str());
    }
    
	// files to read
	ifstream pts_file, frames_file;
	// file to write
	ofstream out_file;


	// compare time values?
	bool checkt = false;
	// max time value to look at, ignored if "checkt" is false
	long etime = 0;


	vector<vector<Frame>> frames;
	vector<vector<CustPoint>> pts;

	// open input files
	pts_file.open("../dat/raw/" + head + "_points.txt");
	frames_file.open("../dat/raw/" + head + "_frames.txt");

	// check if both files are valid
	if (pts_file.is_open() && frames_file.is_open())
	{
		cout << "Input files found\nParsing Frames\n";

		string line;
		
		// Parse Frames
		{
			vector<Frame> tmp_frame;
			int max_id = 0;
			while (getline(frames_file, line))
			{
				Frame fr(line);

				if (fr.id() > max_id)
					max_id = fr.id();

				tmp_frame.push_back(fr);
			}
			frames_file.close();
			cout << "Reorganizing Frames\n";
			frames.resize(max_id + 1);
			for (size_t i = 0;i < tmp_frame.size();i++)
				frames[tmp_frame[i].id()].push_back(tmp_frame[i]);
		}
		
		cout << "Frames Completed\nParsing Points\n";

		// Parse Points
        int max_id = 0;
		{
			vector<CustPoint> tmp_pt;
			while (getline(pts_file, line))
			{
				CustPoint pt(line);

				if (pt.id() > max_id)
					max_id = pt.id();

				tmp_pt.push_back(pt);
			}
			pts_file.close();
			cout << "Reorganizing Points\n";
			pts.resize(max_id + 1);
			for (size_t i = 0;i < tmp_pt.size();i++)
				if(tmp_pt[i].id() >= 0)
					if(tmp_pt[i].d() >= dl && tmp_pt[i].d() <= dh)
						pts[tmp_pt[i].id()].push_back(tmp_pt[i]);
		}
        
        vector<CustPoint> cloud;
        vector<int> cind;
        if(dens == 1)
        {
            cout << "Increasing Point Density\n";
            
            cloud.resize((max_id + 1) * 8);
            cind.resize((max_id + 1) * 8);
            size_t it = 0;
            for(size_t i = 0;i < pts.size();i++)
                if(pts[i].size() > 0)
                    for(size_t j = 0;j < 8;j++)
                    {
                        cloud[it] = pts[i].back();
						cloud[it].setP(j);
                        cind[it] = j;
                        it++;
                    }
            cloud.resize(it);
            cind.resize(it);
        }
		else
			for(size_t i = 0;i < pts.size();i++)
				if(pts[i].size() > 0)
					cloud.push_back(pts[i].back());
        
        
		cout << "Points completed\nTransforming Points to World Coordinates\n";
        
                
        
		Frame* tmp = &frames[0].back();
		int fi = 0;
		for (size_t i = 0;i < cloud.size();i++)
		{
            if (cloud[i].frame_id() != fi)
            {
                fi = cloud[i].frame_id();
                tmp = &frames[fi].back();
            }
            cloud[i].applyFrame(tmp, dens);
		}
        
		cout << "Transformations Completed\nWriting to PCD file\n";

		out_file.open("../dat/pcl/" + head + ".pcd");

		// header
		out_file << "VERSION .7\n";
		out_file << "FIELDS x y z rgb\n";
		out_file << "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n";
		out_file << "WIDTH " << cloud.size() << "\n";
		out_file << "HEIGHT 1\n";
		out_file << "VIEWPOINT 0 0 0 1 0 0 0\n";
		out_file << "POINTS " << cloud.size() << "\n";
		out_file << "DATA ascii\n";

		// data
		//float cc;
        int tc;
		for(size_t i = 0;i < cloud.size();i++)
		{
            if(dens == 0)
            {
				tc = (int) (((int) cloud[i].r()) << 16 | ((int) cloud[i].g()) << 8 | ((int) cloud[i].b()));
			}
            else
			{
                tc = (int) (((int) cloud[i].r(cind[i])) << 16 | ((int) cloud[i].g(cind[i])) << 8 | ((int) cloud[i].b(cind[i])));
			}
            
			out_file << cloud[i].x() << " " << cloud[i].y() << " " << cloud[i].z() << " " << tc << "\n";
		}
        
        delete tmp;
        
		out_file.close();

		cout << "PCD File Completed\n";
		cout << cloud.size() << " points saved\n";


		if(cc == 1)
		{
			color_correct(cloud, "../dat/pcl/" + head + ".pcd");
		}
	}
	else
	{
		if (!pts_file.is_open() && !frames_file.is_open())
			cout << "Error: Frames and Points files not found\n";
		else
			cout << "Error: " << (pts_file.is_open() ? "Frames" : "Points") << " file not found\n";
	}


	cout << "Exiting Program\n";
}





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
CustPoint::CustPoint()
{

}
CustPoint::CustPoint(std::string str)
{
	init(str);
}
CustPoint::CustPoint(std::string str, Frame* frame)
{
	init(str);
	applyFrame(frame);
}
CustPoint::~CustPoint()  { }
void CustPoint::init(std::string str)
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

		for (int j = 0;j < 8;j++)
		{
			cr.push_back(col[j]);
			cg.push_back(col[j]);
			cb.push_back(col[j]);
		}
		mr = cr[4];
		mg = cg[4];
		mb = cb[4];

		std_v = std::atof(str.substr(temp, ind->at(14) - temp).c_str());
		temp = ind->at(14) + 2;
		sz = std::stoi(str.substr(temp, ind->at(15) - temp));
	}
	else if (ind->size() == 40)
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

		for (int j = 0;j < 8;j++)
		{
			cr.push_back(std::atof(str.substr(temp, ind->at(14 + 3*j) - temp).c_str()));
			temp = ind->at(14 + 3*j) + 2;
			cg.push_back(std::atof(str.substr(temp, ind->at(15 + 3*j) - temp).c_str()));
			temp = ind->at(15 + 3*j) + 2;
			cb.push_back(std::atof(str.substr(temp, ind->at(16 + 3*j) - temp).c_str()));
			temp = ind->at(16 + 3*j) + 2;
		}
		mr = cr[4];
		mg = cg[4];
		mb = cb[4];

		std_v = std::atof(str.substr(temp, ind->at(38) - temp).c_str());
		temp = ind->at(38) + 2;
		sz = std::stoi(str.substr(temp, ind->at(39) - temp));
	}

	delete ind;
}

void CustPoint::setP(int i)
{
	iu += staticPattern[i][0];
	iv += staticPattern[i][1];
	mc = col[i];
	mr = cr[i];
	mg = cg[i];
	mb = cb[i];
}
void CustPoint::applyFrame(Frame* frame)
{
    applyFrame(frame, false);
}
void CustPoint::applyFrame(Frame* frame, bool dens)
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
bool CustPoint::isFrameApplied() { return asfr; }

int CustPoint::id() { return id_v; }
long CustPoint::time() { return time_v; }
int CustPoint::frame_id() { return fid; }
float CustPoint::u() { return iu; }
float CustPoint::v() { return iv; }
float CustPoint::d() { return depth; }
float CustPoint::c() { return mc; }
float CustPoint::c(int i) { return col[i]; }
float CustPoint::r() { return mr; }
float CustPoint::r(int i) { return cr[i]; }
float CustPoint::g() { return mg; }
float CustPoint::g(int i) { return cg[i]; }
float CustPoint::b() { return mb; }
float CustPoint::b(int i) { return cb[i]; }
float CustPoint::std() { return std_v; }

float CustPoint::x() { return px; }
float CustPoint::y() { return py; }
float CustPoint::z() { return pz; }
Frame* CustPoint::getFrame() { return fr; }

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