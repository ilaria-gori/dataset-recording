#ifndef __CAMERA_DRIVER_H__
#define __CAMERA_DRIVER_H__

#include <thread>
#include <mutex>
#include <windows.h>
#include <driver.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

class CameraDriver : public Driver
{
	private:
		std::thread rec_t;
		std::mutex mtx;
		bool is_stopping;
		std::string rgb_folder;
		cv::VideoCapture camera;
		int width;
		int height;

		void start();

	public:

		CameraDriver();
		~CameraDriver();
		bool initialize(const int width, const int height);
		void record(const std::string &destFolder);
		void stop();
};
#endif

