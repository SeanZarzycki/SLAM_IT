
#pragma once
#include "boost/thread.hpp"
#include "util/MinimalImage.h"
#include "IOWrapper/Output3DWrapper.h"


#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"
#include <util/settings.h>

#include "vector"
#include <fstream>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/cloud_viewer.h>
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
	bool view, update, dens;

	// pcl::visualization::PCLVisualizer::Ptr pclviewer;
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
// inline PCLWrapper(pcl::visualization::PCLVisualizer::Ptr cloud_viewer)
// {
// 	my_absTH = 2e-3;
// 	my_scaledTH = 2e-3;
// 	my_minRelBS = 0.2;

// 	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
// 	cloud = tmp;

// 	update = false;
// 	view = true;
// 	pclviewer = cloud_viewer;
// 	pclviewer->setBackgroundColor (0, 0, 0);
// 	//pclviewer->addPointCloud(cloud, "Cloud");
// 	pclviewer->initCameraParameters();
	
// 	printf("OUT: Created PCL OutputWrapper\n");
// }

virtual ~PCLWrapper()
{

	printf("OUT: Destroyed Custom OutputWrapper\n");
}


virtual void join()
{
	if(cloud->points.size() > 0)
	{
		pcl::PCDWriter writer;
		writer.write<pcl::PointXYZRGB> ("../../cal/dat/pcl/output.pcd", *cloud, true);
		printf("Write %lu Points to file\n", cloud->points.size());
	}
	else
		printf("No points to write\n");
}
virtual void reset()
{
	mtx_.lock();
	cloud->clear();
	mtx_.unlock();
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
void setDense(bool dense)
{
	dens = dense;
}
/*
// dump points to files
// used for debug only
bool mem_dump = false;
int fid = 0;
*/
virtual void publishKeyframes( std::vector<FrameHessian*> &frames, bool isfinal, CalibHessian* HCalib) override
{
	int factor = (dens ? 8 : 1);
	if(isfinal)
	{
		for(FrameHessian* f : frames)
		{/*
			std::ofstream out_file;
			if(mem_dump)
			{
				out_file.open("/home/steve/repos/SLAM_IT/cal/dat/dump/frame_" + std::to_string(fid) + ".csv");
				if(!out_file.is_open())
					cout << "error opening file\n";
				out_file << "u v d r g b" << std::endl;
				fid++;
			}*/

			pcl::PointCloud<pcl::PointXYZRGB> tmp;
			tmp.resize(f->pointHessiansMarginalized.size() * factor);

			float fxi = 1.0 / (*HCalib).fxl();
			float fyi = 1.0 / (*HCalib).fxl();
			float cxi = -(*HCalib).cxl() * fxi;
			float cyi = -(*HCalib).cyl() * fyi;

			Eigen::Matrix<double,3,4> temp = f->shell->camToWorld.matrix3x4();
			Eigen::Matrix4f mat = Eigen::Matrix4f::Identity(4,4);
			for(int i = 0;i < 3;i++)
				for(int j = 0;j < 4;j++)
					mat(i, j) = temp(i, j);

			std::vector<PointHessian*> points = f->pointHessiansMarginalized;
			std::vector<int> keeps;
			for(size_t i = 0;i < f->pointHessiansMarginalized.size();i++)
				if(points[i]->idepth_scaled > 0.0001 && points[i]->idepth_scaled < 10000)
					keeps.push_back(i);
			for(size_t l = 0;l < pcl_its;l++)
			{
				std::vector<bool> choose (keeps.size(), true);
				for(int i = 0;i < keeps.size();i++)
				{
					PointHessian* cp = points[keeps[i]];
					
					std::vector<float> depths;
					for(size_t j = 0;j < keeps.size();j++)
						if(abs(cp->u - points[keeps[j]]->u) <= pcl_dx && abs(cp->v - points[keeps[j]]->v) <= pcl_dy)
							depths.push_back(1.0 / points[keeps[j]]->idepth_scaled);
					if(depths.size() < pcl_ps)
					{
						choose[i] = false;
					}
					else
					{
						std::sort(depths.begin(), depths.end());
						int pidx = std::round(pcl_prc * (depths.size() - 1));
						std::vector<float> filts (depths.begin() + pidx, depths.end() - pidx);
						int N = filts.size();

						float md = 0;
						for(size_t j = 0;j < filts.size();j++)
							md += filts[j];
						md /= N;
						float dv = 0;
						for(size_t j = 0;j < filts.size();j++)
							dv += pow(md - filts[j], 2);
						dv /= N - 1;

						if(dv > pcl_var)
						{
							choose[i] = false;
						}
						else
						{
							float mn = filts[0];
							float mx = filts[N-1];
							float elt = (N * mn - mx) / (N - 1);
							float eut = (N * mx - mn) / (N - 1);
							float el = ((1 - pcl_prc) * elt - pcl_prc * eut) / (1 - 2 * pcl_prc);
							float eu = ((1 - pcl_prc) * eut - pcl_prc * elt) / (1 - 2 * pcl_prc);
							choose[i] = 1.0 >= el * cp->idepth_scaled && 1.0 <= eu * cp->idepth_scaled;
						}
					}
				}
				std::vector<int> temp_vec;
				for(size_t i = 0;i < keeps.size();i++)
					if(choose[i])
						temp_vec.push_back(keeps[i]);
				keeps = temp_vec;
			}
			for(size_t i = 0;i < keeps.size();i++)
			{
				PointHessian* cp = f->pointHessiansMarginalized[keeps[i]];
				
				for(size_t j = 0;j < factor;j++)
				{
					Eigen::Vector4f pf;
					pf[0] = ((cp->u + patternP[j][0]) * fxi + cxi) / cp->idepth_scaled;
					pf[1] = ((cp->v + patternP[j][1]) * fyi + cyi) / cp->idepth_scaled;
					pf[2] = 1.0 / cp->idepth_scaled;
					pf[3] = 1;
					
					Eigen::Vector4f pw = mat * pf;

					tmp.points[i*factor+j].x = -pw[0];
					tmp.points[i*factor+j].y = -pw[1];
					tmp.points[i*factor+j].z = pw[2];

					tmp.points[i*factor+j].r = cp->rc[j];
					tmp.points[i*factor+j].g = cp->gc[j];
					tmp.points[i*factor+j].b = cp->bc[j];
				}
					
					/*
				if(mem_dump)
				{
					out_file << cp->u << " " << cp->v << " " << (1.0 / cp->idepth_scaled) << " " << cp->rc[0] << " " << cp->gc[0] << " " << cp->bc[0] << endl;
				}*/
			}
			/*
			if(mem_dump)
			{
				out_file.close();
			}*/

			mtx_.lock();
			(*cloud) += tmp;
			update = true;
			mtx_.unlock();
		}
	}
}

};

// void pcl_run(pcl::visualization::PCLVisualizer::Ptr viewer, PCLWrapper* wrap)
// {
// 	while (!viewer->wasStopped ())
// 	{
// 		if(wrap->update_needed())
// 		{
// 			cout << "Points: " << wrap->getCloud()->points.size() << "\n";
// 			viewer->updatePointCloud(wrap->getCloud(), "Cloud");
// 		}
// 		viewer->spinOnce (100, true);
//     	usleep(100000);
// 	}
// }

}
}
