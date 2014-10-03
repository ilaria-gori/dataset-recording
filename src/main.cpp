#include <stdio.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <thread>
#include <dirent.h>
#include <windows.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <kinect2driver.h>
#include <cameraDriver.h>

using namespace cv;
using namespace std;

int main(int argc, char *argv[])
{
	Kinect2Driver driver;
	int width = 640;
	int height = 480;

	if (!driver.initialize(width, height))
	{
		printf("The camera could not open\n");
		return -1;
	}

	string dest_folder = "C:/Users/Ilaria/Documents/dataset/";
	int freq = 10;
	int n = 0;

	CreateDirectory(dest_folder.c_str(), NULL);

	bool recording = true;
	bool pause = true;
	while (recording)
	{
		namedWindow("Color");
		namedWindow("Depth");
		namedWindow("Users");
		namedWindow("Skeleton");
		int code = waitKey(freq);
		switch (code)
		{
		case 115:
			printf("start\n");
			if (pause)
			{
				n++;
				stringstream ss; ss << n;
				string id = ss.str();

				string class_folder = dest_folder + "class" + id + "/";
				CreateDirectory(class_folder.c_str(), NULL);

				driver.record(class_folder);
			}
			pause = false;			
			break;
		case 112:
			printf("pause\n");
			pause = true;
			driver.stop();
			break;
		case 27:
			printf("exit\n");
			driver.stop();
			recording = false;
			break;
		}
	}

	return 1;
}