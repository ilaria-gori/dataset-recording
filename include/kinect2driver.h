#ifndef __KINECT_2_DRIVER_H__
#define __KINECT_2_DRIVER_H__

#include <thread>
#include <mutex>
#include <iostream>
#include <fstream>
#include <windows.h>
#include <driver.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <Kinect.h>

class Kinect2Driver : public Driver
{
	private:
		std::thread rec_t;
		std::mutex mtx;
		bool is_stopping;
		int depth_width;
		int depth_height;
		int width;
		int height;
		std::string rgb_folder;
		std::string depth_folder;
		std::string users_folder;
		std::string skel_folder;

		IKinectSensor*          sensor;
		ICoordinateMapper*      coordinate_mapper;
		IMultiSourceFrameReader* multi_source_frame_reader;

		cv::Vec3b users_color[BODY_COUNT];

		void start();
		void getSkeletons(IBody** body_frame, cv::Mat &skeleton, const int n_frame);
		template<class Interface>
		inline void safeRelease(Interface *& pInterfaceToRelease)
		{
			if (pInterfaceToRelease != NULL){
				pInterfaceToRelease->Release();
				pInterfaceToRelease = NULL;
			}
		}

	public:

		Kinect2Driver();
		~Kinect2Driver();
		bool initialize(const int width, const int height);
		void record(const std::string &destFolder);
		void stop();
};
#endif

