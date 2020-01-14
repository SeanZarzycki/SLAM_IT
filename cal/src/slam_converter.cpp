#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <iostream>

using namespace std;

int
main (int argc, char** argv)
{
    string filename1 = argv[1];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile ("../dat/pcl/" + filename1 + ".pcd", *cloud);
    string filename2 = argc < 3 ? argv[1] : argv[2];
    ofstream out_file ("../dat/ascii/" + filename2 + ".txt", ofstream::out);
    for (size_t i=0; i<cloud->points.size();i++)
    {
        pcl::PointXYZRGB p = cloud->points[i];
        // unpack rgb into r/g/b
        uint32_t rgb = *reinterpret_cast<int*>(&p.rgb);
        int r = (rgb >> 16) & 0x0000ff;
        int g = (rgb >> 8)  & 0x0000ff;
        int b = (rgb)       & 0x0000ff;

        out_file << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << " ";
        out_file << to_string(r);
        out_file << " ";
        out_file << to_string(g);
        out_file << " ";
        out_file << to_string(b);
        out_file << "\n";
    }
    out_file.close();
    return(0);
}