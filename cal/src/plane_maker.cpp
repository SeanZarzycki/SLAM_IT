#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>
#include <string>

#include <pcl/filters/voxel_grid.h>

using namespace std;
int main (int argc, char** argv)
{
    ofstream out_file;
    // defaults
    
    string filename = argv[1];
    double delta = 0;
    scanf("%f", argv[2], &delta);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile ("../dat/pcl/" + filename, *cloud);

    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(3*delta, 3*delta, 3*delta);
    vox.filter(*cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud2->resize(4*cloud->size());
    //out_file.open("../dat/pcl/" + head + ".obj");
    for(int i=0; i < cloud->size(); i++)
    {
        // top
        int j = 0;
        cloud2->points[4*i+j].x = cloud->points[i].x;
        cloud2->points[4*i+j].y = cloud->points[i].y;
        cloud2->points[4*i+j].z = cloud->points[i].z + delta;

        // side 1
        j = 1;
        cloud2->points[4*i+j].x = cloud->points[i].x;
        cloud2->points[4*i+j].y = cloud->points[i].y + delta;
        cloud2->points[4*i+j].z = cloud->points[i].z;


        // side 2
        j = 2;
        cloud2->points[4*i+j].x = cloud->points[i].x + delta;
        cloud2->points[4*i+j].y = cloud->points[i].y;
        cloud2->points[4*i+j].z = cloud->points[i].z;

        // side 3
        j = 3;
        cloud2->points[4*i+j].x = cloud->points[i].x;
        cloud2->points[4*i+j].y = cloud->points[i].y;
        cloud2->points[4*i+j].z = cloud->points[i].z;

        for(j = 0;j < 4;j++)
        {
            cloud2->points[4*i+j].r = cloud->points[i].r;
            cloud2->points[4*i+j].g = cloud->points[i].g;
            cloud2->points[4*i+j].b = cloud->points[i].b;
        }
    }

     pcl::PCDWriter writer;
     writer.write<pcl::PointXYZRGB> ("../dat/pcl/exp_" + filename, *cloud2, false);

    return 0;
}