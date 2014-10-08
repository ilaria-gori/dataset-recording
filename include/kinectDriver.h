#ifndef __KINECT_DRIVER_H__
#define __KINECT_DRIVER_H__

#include <thread>
#include <mutex>
#include <iostream>
#include <fstream>
#include <windows.h>
#include <driver.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv.h>
#include <highgui.h>
#include <cxcore.h>
#include "NuiApi.h"
#include "NuiImageCamera.h"
#include "NuiSensor.h"
#include "NuiSkeleton.h"

class KinectDriver : public Driver
{
	private:
		std::thread rec_t;
		std::mutex mtx;
		bool is_stopping;
		int def_width_depth;
		int def_height_depth;
		int def_width_color;
		int def_height_color;
		int width;
		int height;
		std::string rgb_folder;
		std::string depth_folder;
		std::string users_folder;
		std::string skel_folder;

		cv::Vec3b users_color[6];
		HANDLE h1, h2, h3, h4;

		void start();

	public:

		KinectDriver();
		~KinectDriver();
		bool initialize(const int width, const int height);
		void record(const std::string &destFolder);
		void stop();
};
#endif

