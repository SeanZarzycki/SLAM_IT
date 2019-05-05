#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <iostream>

int
main (int argc, char** argv)
{
    string filename1 = argv[1];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile (filename1, *cloud);
    string filename2 = argv[2];
    out_file.open(filename2 + ".txt");
    for (i=0; i<cloud->size();i++)
    {
        out_file << cloud->points[i].x << " " << cloud->points[i].y << " "cloud->points[i].z << " "cloud->points[i].r << " "cloud->points[i].g << " "cloud->points[i].b << "\n";
    }
    outfile.close();
    return(0);
}