

#pragma once
#include "boost/thread.hpp"
#include "util/MinimalImage.h"
#include "IOWrapper/Output3DWrapper.h"


#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"

#include "vector"

namespace dso
{

class FrameHessian;
class CalibHessian;
class FrameShell;


namespace IOWrap
{
class CustomWrapper : public Output3DWrapper
{
private:
	float my_scaledTH, my_absTH, my_minRelBS;
public:
inline CustomWrapper()
{
	my_absTH = 2e-3;
	my_scaledTH = 2e-3;
	my_minRelBS = 0.2;
	
	printf("OUT: Created Custom OutputWrapper\n");
	outfile1.open("output_frames.txt");
	outfile2.open("output_points.txt");
}

virtual ~CustomWrapper()
{
	printf("OUT: Destroyed Custom OutputWrapper\n");
	outfile1.close();
	outfile2.close();
}

/*
virtual void publishGraph(const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>, Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i>>> &connectivity) override
{ }*/


virtual void publishKeyframes( std::vector<FrameHessian*> &frames, bool final, CalibHessian* HCalib) override
{
	if(final)
	{
		for(FrameHessian* f : frames)
		{
			outfile1 << f->frameID << ", " << f->shell->incoming_id << ", "
				<< f->shell->timestamp << ", " << (int)f->pointHessians.size()
				<< ", " << (int)f->pointHessiansMarginalized.size() << ", "
				<< (int)f->immaturePoints.size();
			Eigen::Matrix<double, 3, 4, 0, 3, 4> temp = f->shell->camToWorld.matrix3x4();
			for(int i =0;i < 3;i++)
				for(int j = 0;j < 4;j++)
					outfile1 << ", " << temp(i, j);
			outfile1 << ", " << (*HCalib).fxl() << ", " << (*HCalib).fyl() << ", "
				<< (*HCalib).cxl() << ", " << (*HCalib).cyl() << "\n";

			//int maxWrite = 5;
			for(PointHessian* p : f->pointHessiansMarginalized)
			{
				//if(p.status == 1 || p.status == 2)
				{
					float var = 1.0f / p->idepth_hessian;
					if(var / p->idepth_scaled <= my_scaledTH && var <= my_absTH)
					{
						outfile2 << f->frameID << ", " << p->id << ", "
							<< f->shell->timestamp << ", " << p->u
							<< ", "	<< p->v << ", " << p->idepth_scaled
							<< ", ";
						for(int i = 0;i < MAX_RES_PER_POINT;i++)
							outfile2 << p->color[i] << ", ";
						for(int i = 0;i < MAX_RES_PER_POINT;i++)
						{
							outfile2 << p->rc[i] << ", ";
							outfile2 << p->gc[i] << ", ";
							outfile2 << p->bc[i] << ", ";
						}

						outfile2 << sqrt(var) << ", "
							<< p->numGoodResiduals << "\n";
						
						//maxWrite--;
						//if(maxWrite==0) break;
					}
				}
			}
		}
	}
}
/*
virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib) override
{        }

virtual void pushLiveFrame(FrameHessian* f) override
{        }

virtual void pushDepthImage(MinimalImageB3* image) override
{        }
virtual bool needPushDepthImage() override
{
    return false;
}
virtual void pushDepthImageFloat(MinimalImageF* image, FrameHessian* KF ) override
{        }*/


};
}
}
