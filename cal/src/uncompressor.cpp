#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>

#include <string>

using namespace std;

int main(int argc, char** argv)
{
    string file;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    file = argv[1];
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZRGB> ("../dat/pcl/" + file, *cloud);

    pcl::PCDWriter writer;
 	writer.write<pcl::PointXYZRGB> ("../dat/ascii/" + file, *cloud, false);

    return 0;
}