#include "Converter/pch.h"
#include <iostream>
#include <fstream>

#include <string>
#include <cstring>
#include<stdlib.h>


using namespace std;

string head;
int dens;
float dl, dh;

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
	vector<vector<Point>> pts;

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
			for (int i = 0;i < tmp_frame.size();i++)
				frames[tmp_frame[i].id()].push_back(tmp_frame[i]);
		}
		
		cout << "Frames Completed\nParsing Points\n";

		// Parse Points
        int max_id = 0;
		{
			vector<Point> tmp_pt;
			while (getline(pts_file, line))
			{
				Point pt(line);

				if (pt.id() > max_id)
					max_id = pt.id();

				tmp_pt.push_back(pt);
			}
			pts_file.close();
			cout << "Reorganizing Points\n";
			pts.resize(max_id + 1);
			for (int i = 0;i < tmp_pt.size();i++)
				if(tmp_pt[i].id() >= 0)
					if(tmp_pt[i].d() >= dl && tmp_pt[i].d() <= dh)
						pts[tmp_pt[i].id()].push_back(tmp_pt[i]);
		}
        
        vector<Point> cloud;
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
		for (int i = 0;i < cloud.size();i++)
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
        long cc;
		for(int i = 0;i < cloud.size();i++)
		{
            if(dens == 0)
                cc = (((uint8_t) cloud[i].c()) << 16 | ((uint8_t) cloud[i].c()) << 8 | ((uint8_t) cloud[i].c()));
            else
                cc = (((uint8_t) cloud[i].c(cind[i])) << 16 | ((uint8_t) cloud[i].c(cind[i])) << 8 | ((uint8_t) cloud[i].c(cind[i])));
            
			out_file << cloud[i].x() << " " << cloud[i].y() << " " << cloud[i].z() << " " << cc << "\n";
		}
        
        delete tmp;
        
		out_file.close();

		cout << "PCD File Completed\n";
		cout << cloud.size() << " points saved\n";
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

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
