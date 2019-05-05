
#pragma once
#include "boost/thread.hpp"
#include "util/MinimalImage.h"
#include "IOWrapper/Output3DWrapper.h"


#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"

#include "vector"
#include <fstream>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Dense>
#include <thread>

#include <unistd.h>


namespace dso
{

class FrameHessian;
class CalibHessian;
class FrameShell;


namespace IOWrap
{


class PCLWrapper : public Output3DWrapper
{
private:
    boost::mutex mtx_;
	float my_scaledTH, my_absTH, my_minRelBS;
	bool view, update;

	pcl::visualization::PCLVisualizer::Ptr pclviewer;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
public:
inline PCLWrapper()
{
	my_absTH = 2e-3;
	my_scaledTH = 2e-3;
	my_minRelBS = 0.2;

	update = false;
	view = false;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud = tmp;
	
	printf("OUT: Created PCL OutputWrapper\n");
}
inline PCLWrapper(pcl::visualization::PCLVisualizer::Ptr cloud_viewer)
{
	my_absTH = 2e-3;
	my_scaledTH = 2e-3;
	my_minRelBS = 0.2;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud = tmp;

	update = false;
	view = true;
	pclviewer = cloud_viewer;
	pclviewer->setBackgroundColor (0, 0, 0);
	//pclviewer->addPointCloud(cloud, "Cloud");
	pclviewer->initCameraParameters();
	
	printf("OUT: Created PCL OutputWrapper\n");
}

virtual ~PCLWrapper()
{
	pcl::PCDWriter writer;
 	writer.write<pcl::PointXYZRGB> ("../../cal/dat/pcl/output.pcd", *cloud, true);

	printf("OUT: Destroyed Custom OutputWrapper\n");
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud()
{
	return cloud;
}
bool update_needed()
{
	bool temp = update;
	update = false;
	return temp;
}

virtual void publishKeyframes( std::vector<FrameHessian*> &frames, bool isfinal, CalibHessian* HCalib) override
{
	if(isfinal)
	{
		for(FrameHessian* f : frames)
		{
			pcl::PointCloud<pcl::PointXYZRGB> tmp;
			tmp.resize(f->pointHessiansMarginalized.size());

			float fxi = 1.0 / (*HCalib).fxl();
			float fyi = 1.0 / (*HCalib).fxl();
			float cxi = -(*HCalib).cxl() * fxi;
			float cyi = -(*HCalib).cyl() * fyi;

			for(size_t i = 0;i < f->pointHessiansMarginalized.size();i++)
			{
				Eigen::Vector4f pf;
				pf[0] = (f->pointHessiansMarginalized[i]->u * fxi + cxi) / f->pointHessiansMarginalized[i]->idepth_scaled;
				pf[1] = (f->pointHessiansMarginalized[i]->v * fyi + cyi) / f->pointHessiansMarginalized[i]->idepth_scaled;
				pf[2] = 1.0 / f->pointHessiansMarginalized[i]->idepth_scaled;
				pf[3] = 1;
				Eigen::Matrix<double,3,4> temp = f->shell->camToWorld.matrix3x4();
				Eigen::Matrix4f mat = Eigen::Matrix4f::Identity(4,4);
				for(int i = 0;i < 3;i++)
					for(int j = 0;j < 4;j++)
						mat(i, j) = temp(i, j);
				Eigen::Vector4f pw = mat * pf;

				tmp.points[i].x = pw[0];
				tmp.points[i].y = pw[1];
				tmp.points[i].z = pw[2];

				tmp.points[i].r = f->pointHessiansMarginalized[i]->rc[0];
				tmp.points[i].g = f->pointHessiansMarginalized[i]->gc[0];
				tmp.points[i].b = f->pointHessiansMarginalized[i]->bc[0];
			}

			mtx_.lock();
			(*cloud) += tmp;
			update = true;
			mtx_.unlock();
		}
	}
}

};

void pcl_run(pcl::visualization::PCLVisualizer::Ptr viewer, PCLWrapper* wrap)
{
	while (!viewer->wasStopped ())
	{
		if(wrap->update_needed())
		{
			cout << "Points: " << wrap->getCloud()->points.size() << "\n";
			viewer->updatePointCloud(wrap->getCloud(), "Cloud");
		}
		viewer->spinOnce (100, true);
    	usleep(100000);
	}
}

}
}
