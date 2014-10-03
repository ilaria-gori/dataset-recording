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

		IKinectSensor* sensor;
		IColorFrameReader* color_reader;
		IDepthFrameReader* depth_reader;
		IBodyFrameReader* body_reader;
		IBodyIndexFrameReader* users_reader;
		ICoordinateMapper* coordinate_mapper;
		IFrameDescription* color_description;
		IFrameDescription* depth_description;

		cv::Vec3b users_color[BODY_COUNT];

		void start();
		void getSkeletons(IBodyFrame* body_frame, cv::Mat &skeleton, const int n_frame);
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

